"""
Advanced Multi-Physics Electrical Engineering Simulator with Illumination Analysis
Combines photometry calculations with dynamic electrical machine simulation
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import math
from scipy.integrate import odeint, solve_ivp
from dataclasses import dataclass
from typing import Tuple, List
import threading
import time


# ==================== ILLUMINATION CALCULATION MODULE ====================
@dataclass
class IlluminationResults:
    """Store illumination calculation results"""
    center_without_reflector: float
    edge_without_reflector: float
    center_with_reflector: float
    edge_with_reflector: float
    lamp_power: float
    disc_diameter: float
    height: float


class IlluminationCalculator:
    """Calculate illumination for lamp and disc configuration"""

    @staticmethod
    def calculate_illumination(candle_power: float, height: float, disc_diameter: float) -> IlluminationResults:
        """
        Calculate illumination at center and edge of disc with and without reflector

        Args:
            candle_power: Uniform candle power in all directions (candela)
            height: Vertical distance from lamp to disc (m)
            disc_diameter: Diameter of circular disc (m)

        Returns:
            IlluminationResults object with all calculations
        """
        radius = disc_diameter / 2

        # WITHOUT REFLECTOR - Direct illumination only
        # At center: E = I × cos(θ) / d²
        # θ = 0° (directly below), d = height
        center_without = candle_power * math.cos(0) / (height ** 2)

        # At edge:
        # Distance from lamp to edge point
        d_edge = math.sqrt(height ** 2 + radius ** 2)
        # Angle: cos(θ) = height / d_edge
        cos_theta = height / d_edge
        edge_without = candle_power * cos_theta / (d_edge ** 2)

        # WITH REFLECTOR
        # Reflector redirects 50% of total emitted light uniformly onto disc
        # Total luminous flux = 4π × I (for uniform point source)
        total_flux = 4 * math.pi * candle_power  # lumens
        reflected_flux = 0.5 * total_flux  # 50% redirected to disc
        disc_area = math.pi * radius ** 2

        # Uniform illumination from reflector across entire disc
        uniform_reflected_illumination = reflected_flux / disc_area

        # Total illumination = direct + reflected
        center_with = center_without + uniform_reflected_illumination
        edge_with = edge_without + uniform_reflected_illumination

        return IlluminationResults(
            center_without_reflector=center_without,
            edge_without_reflector=edge_without,
            center_with_reflector=center_with,
            edge_with_reflector=edge_with,
            lamp_power=candle_power,
            disc_diameter=disc_diameter,
            height=height
        )


# ==================== ELECTRICAL MACHINE MODELS ====================
class InductionMotorModel:
    """Three-phase induction motor model with dynamic equations"""

    def __init__(self, params: dict):
        # Motor parameters
        self.Rs = params.get('Rs', 0.5)  # Stator resistance (Ohm)
        self.Rr = params.get('Rr', 0.3)  # Rotor resistance (Ohm)
        self.Ls = params.get('Ls', 0.01)  # Stator inductance (H)
        self.Lr = params.get('Lr', 0.01)  # Rotor inductance (H)
        self.Lm = params.get('Lm', 0.008)  # Magnetizing inductance (H)
        self.J = params.get('J', 0.5)  # Moment of inertia (kg·m²)
        self.P = params.get('P', 4)  # Number of poles
        self.B = params.get('B', 0.01)  # Friction coefficient
        self.V_rms = params.get('V_rms', 230)  # RMS voltage (V)
        self.f = params.get('f', 50)  # Frequency (Hz)
        self.TL = params.get('TL', 10)  # Load torque (Nm)

        self.omega_sync = 4 * math.pi * self.f / self.P

    def dynamics(self, t: float, y: np.ndarray) -> np.ndarray:
        """
        State-space model of induction motor
        State vector: [ids, iqs, idr, iqr, omega_r, theta]
        """
        ids, iqs, idr, iqr, omega_r, theta = y

        # Voltage inputs (RMS converted to peak for sinusoidal)
        V_peak = self.V_rms * math.sqrt(2)
        omega_e = 2 * math.pi * self.f

        vds = V_peak * math.cos(omega_e * t)
        vqs = V_peak * math.sin(omega_e * t)

        # Flux linkages
        lambda_ds = self.Ls * ids + self.Lm * idr
        lambda_qs = self.Ls * iqs + self.Lm * iqr
        lambda_dr = self.Lr * idr + self.Lm * ids
        lambda_qr = self.Lr * iqr + self.Lm * iqs

        # Electrical equations (dq-frame)
        omega_slip = omega_e - omega_r * self.P / 2

        dids_dt = (vds - self.Rs * ids + omega_e * lambda_qs) / self.Ls
        diqs_dt = (vqs - self.Rs * iqs - omega_e * lambda_ds) / self.Ls
        didr_dt = (-self.Rr * idr + omega_slip * lambda_qr) / self.Lr
        diqr_dt = (-self.Rr * iqr - omega_slip * lambda_dr) / self.Lr

        # Electromagnetic torque
        Te = 1.5 * (self.P / 2) * self.Lm * (iqs * idr - ids * iqr)

        # Mechanical equation
        domega_dt = (Te - self.TL - self.B * omega_r) / self.J
        dtheta_dt = omega_r

        return np.array([dids_dt, diqs_dt, didr_dt, diqr_dt, domega_dt, dtheta_dt])


class SynchronousMachineModel:
    """Synchronous generator/motor model"""

    def __init__(self, params: dict):
        self.Ra = params.get('Ra', 0.5)  # Armature resistance
        self.Xd = params.get('Xd', 1.5)  # d-axis reactance
        self.Xq = params.get('Xq', 1.0)  # q-axis reactance
        self.J = params.get('J', 1.0)  # Inertia
        self.D = params.get('D', 0.05)  # Damping
        self.P = params.get('P', 4)  # Poles
        self.Vf = params.get('Vf', 50)  # Field voltage
        self.V_rms = params.get('V_rms', 230)
        self.f = params.get('f', 50)
        self.Tm = params.get('Tm', 20)  # Mechanical torque

    def dynamics(self, t: float, y: np.ndarray) -> np.ndarray:
        """
        State: [id, iq, omega, delta]
        """
        id_val, iq, omega, delta = y

        omega_e = 2 * math.pi * self.f
        V_peak = self.V_rms * math.sqrt(2)

        # Terminal voltage in dq frame
        vd = V_peak * math.sin(delta)
        vq = V_peak * math.cos(delta)

        # Electrical equations
        Eq_prime = self.Vf  # Simplified field flux linkage

        did_dt = (vd - self.Ra * id_val + omega * self.Xq * iq) / self.Xd
        diq_dt = (vq - self.Ra * iq - omega * self.Xd * id_val - Eq_prime) / self.Xq

        # Electromagnetic torque
        Te = Eq_prime * iq + (self.Xd - self.Xq) * id_val * iq

        # Mechanical equations
        domega_dt = (self.Tm - Te - self.D * omega) / self.J
        ddelta_dt = omega - omega_e

        return np.array([did_dt, diq_dt, domega_dt, ddelta_dt])


class TransformerModel:
    """Single-phase transformer with thermal modeling"""

    def __init__(self, params: dict):
        self.R1 = params.get('R1', 2.0)  # Primary resistance
        self.R2 = params.get('R2', 0.5)  # Secondary resistance
        self.L1 = params.get('L1', 0.1)  # Primary inductance
        self.L2 = params.get('L2', 0.025)  # Secondary inductance
        self.M = params.get('M', 0.045)  # Mutual inductance
        self.C_th = params.get('C_th', 1000)  # Thermal capacitance
        self.R_th = params.get('R_th', 0.5)  # Thermal resistance
        self.V_rms = params.get('V_rms', 230)
        self.f = params.get('f', 50)
        self.RL = params.get('RL', 50)  # Load resistance
        self.T_ambient = params.get('T_ambient', 25)  # Ambient temperature

    def dynamics(self, t: float, y: np.ndarray) -> np.ndarray:
        """
        State: [i1, i2, T_winding]
        """
        i1, i2, T_winding = y

        omega = 2 * math.pi * self.f
        V_peak = self.V_rms * math.sqrt(2)
        v1 = V_peak * math.sin(omega * t)

        # Electrical equations with mutual coupling
        di1_dt = (v1 - self.R1 * i1 - omega * self.M * i2) / self.L1
        di2_dt = (omega * self.M * i1 - self.R2 * i2 - self.RL * i2) / self.L2

        # Thermal model: Heat generation from copper losses
        P_loss = self.R1 * i1**2 + self.R2 * i2**2
        dT_dt = (P_loss - (T_winding - self.T_ambient) / self.R_th) / self.C_th

        return np.array([di1_dt, di2_dt, dT_dt])


# ==================== ODE SOLVERS ====================
class ODESolver:
    """Multiple ODE solver implementations"""

    @staticmethod
    def euler(func, y0, t_span, t_eval, args=()):
        """Euler method for ODE solving"""
        t0, tf = t_span
        dt = t_eval[1] - t_eval[0] if len(t_eval) > 1 else 0.01

        t_values = [t0]
        y_values = [y0]

        t = t0
        y = np.array(y0, dtype=float)

        for t_next in t_eval[1:]:
            while t < t_next:
                dy = func(t, y)
                y = y + dt * np.array(dy)
                t = t + dt
            t_values.append(t)
            y_values.append(y.copy())

        return type('obj', (object,), {'t': np.array(t_values), 'y': np.array(y_values).T})

    @staticmethod
    def rk45(func, y0, t_span, t_eval, args=()):
        """Runge-Kutta 45 method using scipy"""
        sol = solve_ivp(func, t_span, y0, method='RK45', t_eval=t_eval,
                       args=args, rtol=1e-6, atol=1e-9)
        return sol


# ==================== MAIN APPLICATION ====================
class MultiPhysicsSimulatorApp:
    """Main Tkinter application for multi-physics simulation"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Multi-Physics Electrical Engineering Simulator")
        self.root.geometry("1400x900")

        # Simulation state
        self.simulation_running = False
        self.simulation_thread = None
        self.current_model = None
        self.solver_method = 'RK45'

        # Results storage
        self.time_data = []
        self.state_data = []

        # Create main menu
        self.create_menu()

        # Create notebook for tabs
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.create_illumination_tab()
        self.create_induction_motor_tab()
        self.create_synchronous_machine_tab()
        self.create_transformer_tab()

        # Bind resize event
        self.root.bind('<Configure>', self.on_resize)

        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        self.status_bar = ttk.Label(root, textvariable=self.status_var, relief=tk.SUNKEN)
        self.status_bar.pack(side='bottom', fill='x')

    def create_menu(self):
        """Create main menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Reset All", command=self.reset_all)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Solver menu
        solver_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Solver", menu=solver_menu)
        solver_menu.add_radiobutton(label="RK45 (Recommended)",
                                    command=lambda: self.set_solver('RK45'))
        solver_menu.add_radiobutton(label="Euler Method",
                                    command=lambda: self.set_solver('Euler'))

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

    def set_solver(self, method):
        """Set ODE solver method"""
        self.solver_method = method
        self.status_var.set(f"Solver set to: {method}")

    def show_about(self):
        """Show about dialog"""
        about_text = """Advanced Multi-Physics Electrical Engineering Simulator

Features:
• Illumination Analysis (Photometry)
• Induction Motor Simulation
• Synchronous Machine Modeling
• Transformer with Thermal Analysis
• Real-time ODE Solvers (RK45, Euler)
• Dynamic Visualization

Developed for advanced electrical engineering applications."""
        messagebox.showinfo("About", about_text)

    def reset_all(self):
        """Reset all simulations"""
        self.stop_simulation()
        self.time_data = []
        self.state_data = []
        self.status_var.set("All simulations reset")

    # ==================== ILLUMINATION TAB ====================
    def create_illumination_tab(self):
        """Create illumination calculation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Illumination Analysis")

        # Create main container with grid
        main_container = ttk.Frame(tab)
        main_container.pack(fill='both', expand=True, padx=10, pady=10)
        main_container.grid_columnconfigure(0, weight=1)
        main_container.grid_columnconfigure(1, weight=2)
        main_container.grid_rowconfigure(1, weight=1)

        # Input frame
        input_frame = ttk.LabelFrame(main_container, text="Input Parameters", padding=10)
        input_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        # Candle power
        ttk.Label(input_frame, text="Candle Power (cd):").grid(row=0, column=0, sticky='w', pady=5)
        self.illum_cp_var = tk.DoubleVar(value=300)
        self.illum_cp_scale = ttk.Scale(input_frame, from_=100, to=1000,
                                        variable=self.illum_cp_var, orient='horizontal')
        self.illum_cp_scale.grid(row=0, column=1, sticky='ew', padx=5)
        self.illum_cp_label = ttk.Label(input_frame, text="300.0")
        self.illum_cp_label.grid(row=0, column=2, padx=5)
        self.illum_cp_var.trace('w', lambda *args: self.illum_cp_label.config(
            text=f"{self.illum_cp_var.get():.1f}"))

        # Height
        ttk.Label(input_frame, text="Height (m):").grid(row=1, column=0, sticky='w', pady=5)
        self.illum_height_var = tk.DoubleVar(value=20)
        self.illum_height_scale = ttk.Scale(input_frame, from_=5, to=50,
                                           variable=self.illum_height_var, orient='horizontal')
        self.illum_height_scale.grid(row=1, column=1, sticky='ew', padx=5)
        self.illum_height_label = ttk.Label(input_frame, text="20.0")
        self.illum_height_label.grid(row=1, column=2, padx=5)
        self.illum_height_var.trace('w', lambda *args: self.illum_height_label.config(
            text=f"{self.illum_height_var.get():.1f}"))

        # Diameter
        ttk.Label(input_frame, text="Disc Diameter (m):").grid(row=2, column=0, sticky='w', pady=5)
        self.illum_diam_var = tk.DoubleVar(value=20)
        self.illum_diam_scale = ttk.Scale(input_frame, from_=5, to=40,
                                         variable=self.illum_diam_var, orient='horizontal')
        self.illum_diam_scale.grid(row=2, column=1, sticky='ew', padx=5)
        self.illum_diam_label = ttk.Label(input_frame, text="20.0")
        self.illum_diam_label.grid(row=2, column=2, padx=5)
        self.illum_diam_var.trace('w', lambda *args: self.illum_diam_label.config(
            text=f"{self.illum_diam_var.get():.1f}"))

        input_frame.grid_columnconfigure(1, weight=1)

        # Calculate button
        calc_btn = ttk.Button(input_frame, text="Calculate Illumination",
                             command=self.calculate_illumination)
        calc_btn.grid(row=3, column=0, columnspan=3, pady=10)

        # Results frame
        results_frame = ttk.LabelFrame(main_container, text="Results", padding=10)
        results_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)

        self.illum_results_text = scrolledtext.ScrolledText(results_frame, height=20, width=50)
        self.illum_results_text.pack(fill='both', expand=True)

        # Visualization frame
        viz_frame = ttk.LabelFrame(main_container, text="Illumination Distribution", padding=10)
        viz_frame.grid(row=0, column=1, rowspan=2, sticky='nsew', padx=5, pady=5)

        self.illum_fig = Figure(figsize=(8, 6), dpi=100)
        self.illum_canvas = FigureCanvasTkAgg(self.illum_fig, master=viz_frame)
        self.illum_canvas.get_tk_widget().pack(fill='both', expand=True)

    def calculate_illumination(self):
        """Calculate and display illumination results"""
        cp = self.illum_cp_var.get()
        height = self.illum_height_var.get()
        diameter = self.illum_diam_var.get()

        calc = IlluminationCalculator()
        results = calc.calculate_illumination(cp, height, diameter)

        # Display results
        self.illum_results_text.delete(1.0, tk.END)
        output = f"""ILLUMINATION CALCULATION RESULTS
{'='*50}

Input Parameters:
  • Candle Power: {results.lamp_power:.1f} cd
  • Height: {results.height:.1f} m
  • Disc Diameter: {results.disc_diameter:.1f} m
  • Disc Radius: {results.disc_diameter/2:.1f} m

WITHOUT REFLECTOR (Direct Light Only):
  • Center Illumination: {results.center_without_reflector:.4f} lux
  • Edge Illumination: {results.edge_without_reflector:.4f} lux
  • Ratio (Edge/Center): {results.edge_without_reflector/results.center_without_reflector:.4f}

WITH REFLECTOR (50% Redirected Uniformly):
  • Center Illumination: {results.center_with_reflector:.4f} lux
  • Edge Illumination: {results.edge_with_reflector:.4f} lux
  • Ratio (Edge/Center): {results.edge_with_reflector/results.center_with_reflector:.4f}

Improvement Factor:
  • Center: {results.center_with_reflector/results.center_without_reflector:.2f}x
  • Edge: {results.edge_with_reflector/results.edge_without_reflector:.2f}x

Total Luminous Flux: {4*math.pi*cp:.2f} lumens
Reflected Flux to Disc: {2*math.pi*cp:.2f} lumens
Disc Area: {math.pi*(diameter/2)**2:.2f} m²
"""
        self.illum_results_text.insert(1.0, output)

        # Visualize illumination distribution
        self.plot_illumination_distribution(results)

    def plot_illumination_distribution(self, results: IlluminationResults):
        """Plot 3D illumination distribution"""
        self.illum_fig.clear()

        radius = results.disc_diameter / 2
        r = np.linspace(0, radius, 50)
        theta = np.linspace(0, 2*np.pi, 100)
        R, THETA = np.meshgrid(r, theta)

        # Calculate illumination at each point without reflector
        height = results.height
        D = np.sqrt(height**2 + R**2)
        cos_angle = height / D
        E_without = results.lamp_power * cos_angle / (D**2)

        # With reflector - add uniform component
        reflected_flux = 2 * math.pi * results.lamp_power
        disc_area = math.pi * radius**2
        uniform_component = reflected_flux / disc_area
        E_with = E_without + uniform_component

        # Convert to Cartesian
        X = R * np.cos(THETA)
        Y = R * np.sin(THETA)

        # Plot both surfaces
        ax = self.illum_fig.add_subplot(121, projection='3d')
        surf1 = ax.plot_surface(X, Y, E_without, cmap='viridis', alpha=0.8)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Illumination (lux)')
        ax.set_title('Without Reflector')
        self.illum_fig.colorbar(surf1, ax=ax, shrink=0.5)

        ax2 = self.illum_fig.add_subplot(122, projection='3d')
        surf2 = ax2.plot_surface(X, Y, E_with, cmap='plasma', alpha=0.8)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_zlabel('Illumination (lux)')
        ax2.set_title('With Reflector')
        self.illum_fig.colorbar(surf2, ax=ax2, shrink=0.5)

        self.illum_fig.tight_layout()
        self.illum_canvas.draw()

    # ==================== INDUCTION MOTOR TAB ====================
    def create_induction_motor_tab(self):
        """Create induction motor simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Induction Motor")

        main_container = ttk.Frame(tab)
        main_container.pack(fill='both', expand=True, padx=10, pady=10)
        main_container.grid_columnconfigure(1, weight=1)
        main_container.grid_rowconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(main_container, text="Motor Parameters", padding=10)
        control_frame.grid(row=0, column=0, sticky='ns', padx=5, pady=5)

        params = [
            ("Voltage RMS (V)", "im_v_rms", 230, 100, 500),
            ("Frequency (Hz)", "im_freq", 50, 30, 100),
            ("Load Torque (Nm)", "im_tl", 10, 0, 50),
            ("Stator Resistance (Ω)", "im_rs", 0.5, 0.1, 2.0),
            ("Rotor Resistance (Ω)", "im_rr", 0.3, 0.1, 2.0),
            ("Inertia (kg·m²)", "im_j", 0.5, 0.1, 2.0),
        ]

        self.im_vars = {}
        for idx, (label, var_name, default, min_val, max_val) in enumerate(params):
            ttk.Label(control_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)
            var = tk.DoubleVar(value=default)
            self.im_vars[var_name] = var
            scale = ttk.Scale(control_frame, from_=min_val, to=max_val,
                            variable=var, orient='horizontal')
            scale.grid(row=idx, column=1, sticky='ew', padx=5)
            lbl = ttk.Label(control_frame, text=f"{default:.2f}")
            lbl.grid(row=idx, column=2, padx=5)
            var.trace('w', lambda *args, l=lbl, v=var: l.config(text=f"{v.get():.2f}"))

        control_frame.grid_columnconfigure(1, weight=1)

        # Control buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.grid(row=len(params), column=0, columnspan=3, pady=10)

        self.im_start_btn = ttk.Button(btn_frame, text="Start",
                                       command=lambda: self.start_motor_simulation('induction'))
        self.im_start_btn.pack(side='left', padx=5)

        self.im_stop_btn = ttk.Button(btn_frame, text="Stop",
                                      command=self.stop_simulation, state='disabled')
        self.im_stop_btn.pack(side='left', padx=5)

        self.im_reset_btn = ttk.Button(btn_frame, text="Reset",
                                       command=self.reset_motor_simulation)
        self.im_reset_btn.pack(side='left', padx=5)

        # Visualization
        viz_frame = ttk.LabelFrame(main_container, text="Real-Time Simulation", padding=10)
        viz_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        self.im_fig = Figure(figsize=(10, 8), dpi=100)
        self.im_canvas = FigureCanvasTkAgg(self.im_fig, master=viz_frame)
        self.im_canvas.get_tk_widget().pack(fill='both', expand=True)

    # ==================== SYNCHRONOUS MACHINE TAB ====================
    def create_synchronous_machine_tab(self):
        """Create synchronous machine simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Synchronous Machine")

        main_container = ttk.Frame(tab)
        main_container.pack(fill='both', expand=True, padx=10, pady=10)
        main_container.grid_columnconfigure(1, weight=1)
        main_container.grid_rowconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(main_container, text="Machine Parameters", padding=10)
        control_frame.grid(row=0, column=0, sticky='ns', padx=5, pady=5)

        params = [
            ("Voltage RMS (V)", "sm_v_rms", 230, 100, 500),
            ("Frequency (Hz)", "sm_freq", 50, 30, 100),
            ("Field Voltage (V)", "sm_vf", 50, 10, 100),
            ("Mech. Torque (Nm)", "sm_tm", 20, 0, 100),
            ("Armature Res. (Ω)", "sm_ra", 0.5, 0.1, 2.0),
            ("Inertia (kg·m²)", "sm_j", 1.0, 0.1, 5.0),
        ]

        self.sm_vars = {}
        for idx, (label, var_name, default, min_val, max_val) in enumerate(params):
            ttk.Label(control_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)
            var = tk.DoubleVar(value=default)
            self.sm_vars[var_name] = var
            scale = ttk.Scale(control_frame, from_=min_val, to=max_val,
                            variable=var, orient='horizontal')
            scale.grid(row=idx, column=1, sticky='ew', padx=5)
            lbl = ttk.Label(control_frame, text=f"{default:.2f}")
            lbl.grid(row=idx, column=2, padx=5)
            var.trace('w', lambda *args, l=lbl, v=var: l.config(text=f"{v.get():.2f}"))

        control_frame.grid_columnconfigure(1, weight=1)

        # Control buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.grid(row=len(params), column=0, columnspan=3, pady=10)

        self.sm_start_btn = ttk.Button(btn_frame, text="Start",
                                       command=lambda: self.start_motor_simulation('synchronous'))
        self.sm_start_btn.pack(side='left', padx=5)

        self.sm_stop_btn = ttk.Button(btn_frame, text="Stop",
                                      command=self.stop_simulation, state='disabled')
        self.sm_stop_btn.pack(side='left', padx=5)

        self.sm_reset_btn = ttk.Button(btn_frame, text="Reset",
                                       command=self.reset_motor_simulation)
        self.sm_reset_btn.pack(side='left', padx=5)

        # Visualization
        viz_frame = ttk.LabelFrame(main_container, text="Real-Time Simulation", padding=10)
        viz_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        self.sm_fig = Figure(figsize=(10, 8), dpi=100)
        self.sm_canvas = FigureCanvasTkAgg(self.sm_fig, master=viz_frame)
        self.sm_canvas.get_tk_widget().pack(fill='both', expand=True)

    # ==================== TRANSFORMER TAB ====================
    def create_transformer_tab(self):
        """Create transformer simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Transformer + Thermal")

        main_container = ttk.Frame(tab)
        main_container.pack(fill='both', expand=True, padx=10, pady=10)
        main_container.grid_columnconfigure(1, weight=1)
        main_container.grid_rowconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(main_container, text="Transformer Parameters", padding=10)
        control_frame.grid(row=0, column=0, sticky='ns', padx=5, pady=5)

        params = [
            ("Primary Voltage (V)", "tf_v_rms", 230, 100, 400),
            ("Frequency (Hz)", "tf_freq", 50, 30, 100),
            ("Load Resistance (Ω)", "tf_rl", 50, 10, 200),
            ("Primary Res. (Ω)", "tf_r1", 2.0, 0.5, 10.0),
            ("Secondary Res. (Ω)", "tf_r2", 0.5, 0.1, 5.0),
            ("Ambient Temp (°C)", "tf_tamb", 25, 0, 50),
        ]

        self.tf_vars = {}
        for idx, (label, var_name, default, min_val, max_val) in enumerate(params):
            ttk.Label(control_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)
            var = tk.DoubleVar(value=default)
            self.tf_vars[var_name] = var
            scale = ttk.Scale(control_frame, from_=min_val, to=max_val,
                            variable=var, orient='horizontal')
            scale.grid(row=idx, column=1, sticky='ew', padx=5)
            lbl = ttk.Label(control_frame, text=f"{default:.2f}")
            lbl.grid(row=idx, column=2, padx=5)
            var.trace('w', lambda *args, l=lbl, v=var: l.config(text=f"{v.get():.2f}"))

        control_frame.grid_columnconfigure(1, weight=1)

        # Control buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.grid(row=len(params), column=0, columnspan=3, pady=10)

        self.tf_start_btn = ttk.Button(btn_frame, text="Start",
                                       command=lambda: self.start_motor_simulation('transformer'))
        self.tf_start_btn.pack(side='left', padx=5)

        self.tf_stop_btn = ttk.Button(btn_frame, text="Stop",
                                      command=self.stop_simulation, state='disabled')
        self.tf_stop_btn.pack(side='left', padx=5)

        self.tf_reset_btn = ttk.Button(btn_frame, text="Reset",
                                       command=self.reset_motor_simulation)
        self.tf_reset_btn.pack(side='left', padx=5)

        # Visualization
        viz_frame = ttk.LabelFrame(main_container, text="Real-Time Simulation", padding=10)
        viz_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        self.tf_fig = Figure(figsize=(10, 8), dpi=100)
        self.tf_canvas = FigureCanvasTkAgg(self.tf_fig, master=viz_frame)
        self.tf_canvas.get_tk_widget().pack(fill='both', expand=True)

    # ==================== SIMULATION CONTROL ====================
    def start_motor_simulation(self, machine_type: str):
        """Start dynamic simulation"""
        if self.simulation_running:
            messagebox.showwarning("Warning", "Simulation already running")
            return

        self.simulation_running = True
        self.status_var.set(f"Running {machine_type} simulation with {self.solver_method}...")

        # Disable start buttons, enable stop
        if machine_type == 'induction':
            self.im_start_btn.config(state='disabled')
            self.im_stop_btn.config(state='normal')
        elif machine_type == 'synchronous':
            self.sm_start_btn.config(state='disabled')
            self.sm_stop_btn.config(state='normal')
        elif machine_type == 'transformer':
            self.tf_start_btn.config(state='disabled')
            self.tf_stop_btn.config(state='normal')

        # Run simulation in thread
        self.simulation_thread = threading.Thread(
            target=self.run_simulation,
            args=(machine_type,),
            daemon=True
        )
        self.simulation_thread.start()

    def stop_simulation(self):
        """Stop running simulation"""
        self.simulation_running = False
        self.status_var.set("Simulation stopped")

        # Re-enable buttons
        self.im_start_btn.config(state='normal')
        self.im_stop_btn.config(state='disabled')
        self.sm_start_btn.config(state='normal')
        self.sm_stop_btn.config(state='disabled')
        self.tf_start_btn.config(state='normal')
        self.tf_stop_btn.config(state='disabled')

    def reset_motor_simulation(self):
        """Reset motor simulation data"""
        self.stop_simulation()
        self.time_data = []
        self.state_data = []
        self.status_var.set("Simulation reset")

    def run_simulation(self, machine_type: str):
        """Run the actual simulation in background thread"""
        try:
            # Get parameters based on machine type
            if machine_type == 'induction':
                params = {
                    'V_rms': self.im_vars['im_v_rms'].get(),
                    'f': self.im_vars['im_freq'].get(),
                    'TL': self.im_vars['im_tl'].get(),
                    'Rs': self.im_vars['im_rs'].get(),
                    'Rr': self.im_vars['im_rr'].get(),
                    'J': self.im_vars['im_j'].get(),
                }
                model = InductionMotorModel(params)
                y0 = [0, 0, 0, 0, 0, 0]  # Initial states
                fig = self.im_fig
                canvas = self.im_canvas

            elif machine_type == 'synchronous':
                params = {
                    'V_rms': self.sm_vars['sm_v_rms'].get(),
                    'f': self.sm_vars['sm_freq'].get(),
                    'Vf': self.sm_vars['sm_vf'].get(),
                    'Tm': self.sm_vars['sm_tm'].get(),
                    'Ra': self.sm_vars['sm_ra'].get(),
                    'J': self.sm_vars['sm_j'].get(),
                }
                model = SynchronousMachineModel(params)
                y0 = [0, 0, 2*math.pi*params['f'], 0]
                fig = self.sm_fig
                canvas = self.sm_canvas

            elif machine_type == 'transformer':
                params = {
                    'V_rms': self.tf_vars['tf_v_rms'].get(),
                    'f': self.tf_vars['tf_freq'].get(),
                    'RL': self.tf_vars['tf_rl'].get(),
                    'R1': self.tf_vars['tf_r1'].get(),
                    'R2': self.tf_vars['tf_r2'].get(),
                    'T_ambient': self.tf_vars['tf_tamb'].get(),
                }
                model = TransformerModel(params)
                y0 = [0, 0, params['T_ambient']]
                fig = self.tf_fig
                canvas = self.tf_canvas

            # Simulation parameters
            t_span = (0, 2.0)  # 2 seconds
            t_eval = np.linspace(0, 2.0, 500)

            # Solve based on selected method
            if self.solver_method == 'RK45':
                sol = ODESolver.rk45(model.dynamics, y0, t_span, t_eval)
            else:  # Euler
                sol = ODESolver.euler(model.dynamics, y0, t_span, t_eval)

            # Update visualization
            self.root.after(0, lambda: self.update_plots(sol, machine_type, fig, canvas))

        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Simulation Error", str(e)))
        finally:
            self.simulation_running = False
            self.root.after(0, lambda: self.stop_simulation())

    def update_plots(self, sol, machine_type, fig, canvas):
        """Update plots with simulation results"""
        fig.clear()

        if machine_type == 'induction':
            # Plot currents, speed, torque
            ax1 = fig.add_subplot(221)
            ax1.plot(sol.t, sol.y[0], label='ids')
            ax1.plot(sol.t, sol.y[1], label='iqs')
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Stator Current (A)')
            ax1.legend()
            ax1.grid(True)
            ax1.set_title('Stator Currents')

            ax2 = fig.add_subplot(222)
            ax2.plot(sol.t, sol.y[2], label='idr')
            ax2.plot(sol.t, sol.y[3], label='iqr')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Rotor Current (A)')
            ax2.legend()
            ax2.grid(True)
            ax2.set_title('Rotor Currents')

            ax3 = fig.add_subplot(223)
            speed_rpm = sol.y[4] * 60 / (2*math.pi)
            ax3.plot(sol.t, speed_rpm)
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Speed (RPM)')
            ax3.grid(True)
            ax3.set_title('Rotor Speed')

            ax4 = fig.add_subplot(224)
            # Calculate electromagnetic torque
            Lm = 0.008
            P = 4
            Te = 1.5 * (P/2) * Lm * (sol.y[1] * sol.y[2] - sol.y[0] * sol.y[3])
            ax4.plot(sol.t, Te)
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Torque (Nm)')
            ax4.grid(True)
            ax4.set_title('Electromagnetic Torque')

        elif machine_type == 'synchronous':
            ax1 = fig.add_subplot(221)
            ax1.plot(sol.t, sol.y[0], label='id')
            ax1.plot(sol.t, sol.y[1], label='iq')
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Current (A)')
            ax1.legend()
            ax1.grid(True)
            ax1.set_title('dq Currents')

            ax2 = fig.add_subplot(222)
            speed_rpm = sol.y[2] * 60 / (2*math.pi)
            ax2.plot(sol.t, speed_rpm)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Speed (RPM)')
            ax2.grid(True)
            ax2.set_title('Rotor Speed')

            ax3 = fig.add_subplot(223)
            delta_deg = np.degrees(sol.y[3])
            ax3.plot(sol.t, delta_deg)
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Power Angle (degrees)')
            ax3.grid(True)
            ax3.set_title('Power Angle δ')

            ax4 = fig.add_subplot(224)
            # Phase diagram
            ax4.plot(delta_deg, speed_rpm)
            ax4.set_xlabel('Power Angle (degrees)')
            ax4.set_ylabel('Speed (RPM)')
            ax4.grid(True)
            ax4.set_title('Phase Portrait')

        elif machine_type == 'transformer':
            ax1 = fig.add_subplot(221)
            ax1.plot(sol.t, sol.y[0])
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Primary Current (A)')
            ax1.grid(True)
            ax1.set_title('Primary Current')

            ax2 = fig.add_subplot(222)
            ax2.plot(sol.t, sol.y[1])
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Secondary Current (A)')
            ax2.grid(True)
            ax2.set_title('Secondary Current')

            ax3 = fig.add_subplot(223)
            ax3.plot(sol.t, sol.y[2])
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Temperature (°C)')
            ax3.grid(True)
            ax3.set_title('Winding Temperature')

            ax4 = fig.add_subplot(224)
            # Power analysis
            R1 = self.tf_vars['tf_r1'].get()
            R2 = self.tf_vars['tf_r2'].get()
            P_loss = R1 * sol.y[0]**2 + R2 * sol.y[1]**2
            ax4.plot(sol.t, P_loss)
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Power Loss (W)')
            ax4.grid(True)
            ax4.set_title('Copper Losses')

        fig.tight_layout()
        canvas.draw()

    def on_resize(self, event):
        """Handle window resize event for auto-scaling"""
        # This is automatically handled by pack/grid with expand=True
        pass


# ==================== MAIN ENTRY POINT ====================
def main():
    """Main entry point"""
    root = tk.Tk()
    app = MultiPhysicsSimulatorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
