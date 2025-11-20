#!/usr/bin/env python3
"""
Advanced Induction Furnace and Motor Multi-Physics Simulation Application
Features:
- Induction furnace problem solver
- Multi-physics simulation (electromagnetic, thermal, mechanical)
- Advanced ODE solvers (RK45, Euler)
- Real-time dynamic visualization
- Economic analysis
- Thermal derating and advanced controls
- Auto-scaling GUI
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import math
from scipy.integrate import solve_ivp
from datetime import datetime
import threading
import time


class InductionFurnaceSolver:
    """Solver for the induction furnace problem"""

    def __init__(self):
        self.V2 = 20  # Secondary voltage (V)
        self.P_full = 600e3  # Power at full hearth (W)
        self.pf_full = 0.6  # Power factor at full hearth

    def solve(self):
        """Solve for half-full hearth conditions"""
        # Full hearth calculations
        S_full = self.P_full / self.pf_full  # Apparent power
        I_full = S_full / self.V2  # Current
        Z_full = self.V2 / I_full  # Impedance

        # Calculate resistance and reactance
        R_full = Z_full * self.pf_full
        angle_full = math.acos(self.pf_full)
        X_full = Z_full * math.sin(angle_full)

        # Half-full hearth (resistance doubles, reactance same)
        R_half = 2 * R_full
        X_half = X_full
        Z_half = math.sqrt(R_half**2 + X_half**2)

        # Calculate half-full parameters
        I_half = self.V2 / Z_half
        P_half = I_half**2 * R_half
        pf_half = R_half / Z_half

        results = {
            'full_hearth': {
                'power': self.P_full,
                'pf': self.pf_full,
                'current': I_full,
                'resistance': R_full,
                'reactance': X_full,
                'impedance': Z_full
            },
            'half_hearth': {
                'power': P_half,
                'pf': pf_half,
                'current': I_half,
                'resistance': R_half,
                'reactance': X_half,
                'impedance': Z_half
            }
        }

        return results


class MultiPhysicsEngine:
    """Multi-physics simulation engine for electrical machines"""

    def __init__(self):
        # Electrical parameters
        self.R_stator = 0.5  # Stator resistance (Ω)
        self.R_rotor = 0.3  # Rotor resistance (Ω)
        self.L_stator = 0.05  # Stator inductance (H)
        self.L_rotor = 0.04  # Rotor inductance (H)
        self.L_mutual = 0.03  # Mutual inductance (H)

        # Mechanical parameters
        self.J = 0.1  # Moment of inertia (kg·m²)
        self.B = 0.01  # Friction coefficient (N·m·s)
        self.pole_pairs = 2

        # Thermal parameters
        self.C_thermal = 1000  # Thermal capacitance (J/K)
        self.R_thermal = 5  # Thermal resistance (K/W)
        self.T_ambient = 25  # Ambient temperature (°C)

        # Operating parameters
        self.V_supply = 400  # Supply voltage (V)
        self.frequency = 50  # Frequency (Hz)
        self.load_torque = 50  # Load torque (N·m)

        # State variables
        self.reset_state()

    def reset_state(self):
        """Reset simulation state"""
        self.time = 0
        self.i_d = 0  # d-axis current
        self.i_q = 0  # q-axis current
        self.omega = 0  # Angular velocity
        self.theta = 0  # Rotor angle
        self.temperature = self.T_ambient
        self.torque = 0

    def electromagnetic_model(self, t, y, V_d, V_q, T_load):
        """
        Electromagnetic differential equations in d-q frame
        State vector y = [i_d, i_q, omega, theta, T]
        """
        i_d, i_q, omega, theta, T = y

        # Electrical equations (RMS values)
        omega_e = self.frequency * 2 * np.pi

        # Voltage equations
        di_d_dt = (V_d - self.R_stator * i_d + omega_e * self.L_stator * i_q) / self.L_stator
        di_q_dt = (V_q - self.R_stator * i_q - omega_e * self.L_stator * i_d) / self.L_stator

        # Electromagnetic torque
        T_em = 1.5 * self.pole_pairs * self.L_mutual * (i_d * i_q)

        # Mechanical equation
        domega_dt = (T_em - T_load - self.B * omega) / self.J

        # Rotor angle
        dtheta_dt = omega

        # Thermal equation (heat generation and dissipation)
        P_copper = self.R_stator * (i_d**2 + i_q**2)
        P_core = 0.1 * (V_d**2 + V_q**2) / self.R_stator  # Simplified core loss
        P_friction = self.B * omega**2
        P_total = P_copper + P_core + P_friction

        dT_dt = (P_total - (T - self.T_ambient) / self.R_thermal) / self.C_thermal

        return [di_d_dt, di_q_dt, domega_dt, dtheta_dt, dT_dt]

    def calculate_losses(self, i_d, i_q, omega, V_d, V_q):
        """Calculate detailed loss breakdown"""
        # Copper losses
        P_copper_stator = self.R_stator * (i_d**2 + i_q**2)
        P_copper_rotor = self.R_rotor * (i_d**2 + i_q**2) * 0.5

        # Iron losses (hysteresis + eddy current)
        f = self.frequency
        B_max = (V_d**2 + V_q**2)**0.5 / (4.44 * f * 0.1)  # Simplified
        P_hysteresis = 0.05 * f * B_max**2
        P_eddy = 0.02 * (f * B_max)**2

        # Mechanical losses
        P_friction = self.B * omega**2
        P_windage = 0.001 * omega**3

        # Stray load losses
        P_stray = 0.01 * (P_copper_stator + P_copper_rotor)

        losses = {
            'copper_stator': P_copper_stator,
            'copper_rotor': P_copper_rotor,
            'hysteresis': P_hysteresis,
            'eddy_current': P_eddy,
            'friction': P_friction,
            'windage': P_windage,
            'stray': P_stray,
            'total': (P_copper_stator + P_copper_rotor + P_hysteresis +
                     P_eddy + P_friction + P_windage + P_stray)
        }

        return losses

    def calculate_mechanical_stress(self, torque, omega):
        """Calculate mechanical stress analysis"""
        # Shaft stress
        shaft_diameter = 0.05  # m
        shaft_radius = shaft_diameter / 2
        J_shaft = np.pi * shaft_radius**4 / 2

        tau_max = torque * shaft_radius / J_shaft  # Maximum shear stress

        # Bearing loads (simplified)
        bearing_radial = abs(torque) / shaft_radius
        bearing_axial = 0.1 * bearing_radial

        stress = {
            'shaft_shear_stress': tau_max,
            'bearing_radial_load': bearing_radial,
            'bearing_axial_load': bearing_axial,
            'safety_factor': 200e6 / tau_max if tau_max > 0 else float('inf')  # Assuming steel
        }

        return stress


class ODESolver:
    """ODE solver with multiple methods"""

    @staticmethod
    def euler(func, y0, t_span, t_eval, args=()):
        """Euler method"""
        t_start, t_end = t_span
        t = np.array(t_eval)
        dt = t[1] - t[0] if len(t) > 1 else 0.001

        y = np.zeros((len(t), len(y0)))
        y[0] = y0

        for i in range(1, len(t)):
            dy = func(t[i-1], y[i-1], *args)
            y[i] = y[i-1] + np.array(dy) * dt

        return type('Solution', (), {'t': t, 'y': y.T, 'success': True})

    @staticmethod
    def rk45(func, y0, t_span, t_eval, args=()):
        """Runge-Kutta 45 method (using scipy)"""
        return solve_ivp(func, t_span, y0, method='RK45', t_eval=t_eval, args=args)


class EconomicAnalyzer:
    """Economic analysis for electrical machines"""

    def __init__(self):
        self.electricity_cost = 0.12  # $/kWh
        self.maintenance_cost_per_hour = 5.0  # $/hour
        self.equipment_cost = 50000  # $
        self.lifetime_years = 15
        self.discount_rate = 0.05

    def calculate_operating_cost(self, power_kw, hours, efficiency):
        """Calculate operating cost"""
        energy_consumed = power_kw * hours
        electricity_cost = energy_consumed * self.electricity_cost
        maintenance_cost = hours * self.maintenance_cost_per_hour

        # Energy loss cost
        power_loss = power_kw * (1 - efficiency)
        loss_cost = power_loss * hours * self.electricity_cost

        total_cost = electricity_cost + maintenance_cost + loss_cost

        return {
            'energy_cost': electricity_cost,
            'maintenance_cost': maintenance_cost,
            'loss_cost': loss_cost,
            'total_cost': total_cost,
            'energy_consumed': energy_consumed
        }

    def calculate_lifecycle_cost(self, annual_operating_hours, avg_power_kw, efficiency):
        """Calculate lifecycle cost"""
        annual_cost = self.calculate_operating_cost(avg_power_kw, annual_operating_hours, efficiency)['total_cost']

        # Present value of operating costs
        pv_operating = 0
        for year in range(1, self.lifetime_years + 1):
            pv_operating += annual_cost / ((1 + self.discount_rate) ** year)

        total_lifecycle_cost = self.equipment_cost + pv_operating

        return {
            'equipment_cost': self.equipment_cost,
            'annual_operating_cost': annual_cost,
            'pv_operating_cost': pv_operating,
            'total_lifecycle_cost': total_lifecycle_cost,
            'cost_per_year': total_lifecycle_cost / self.lifetime_years
        }


class AdvancedControlSystem:
    """Advanced control system with thermal derating"""

    def __init__(self):
        # PID controller parameters
        self.Kp = 1.0
        self.Ki = 0.5
        self.Kd = 0.1

        self.integral_error = 0
        self.previous_error = 0

        # Thermal derating parameters
        self.T_rated = 100  # Rated temperature (°C)
        self.T_max = 120  # Maximum temperature (°C)
        self.derating_start = 90  # Temperature to start derating (°C)

    def pid_control(self, setpoint, measured, dt):
        """PID controller"""
        error = setpoint - measured

        self.integral_error += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0

        output = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative

        self.previous_error = error

        return output

    def thermal_derating(self, temperature, rated_power):
        """Calculate derated power based on temperature"""
        if temperature < self.derating_start:
            return rated_power
        elif temperature >= self.T_max:
            return 0
        else:
            derating_factor = 1 - (temperature - self.derating_start) / (self.T_max - self.derating_start)
            return rated_power * derating_factor

    def field_oriented_control(self, omega_ref, omega_actual, torque_ref):
        """Field-oriented control (FOC)"""
        # Speed control loop
        speed_error = omega_ref - omega_actual
        i_q_ref = self.pid_control(omega_ref, omega_actual, 0.001)

        # Flux control
        i_d_ref = 0  # For maximum torque per ampere

        return i_d_ref, i_q_ref


class MultiPhysicsGUI:
    """Main GUI application"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Induction Furnace & Multi-Physics Simulation")
        self.root.geometry("1400x900")

        # Initialize components
        self.furnace_solver = InductionFurnaceSolver()
        self.physics_engine = MultiPhysicsEngine()
        self.ode_solver = ODESolver()
        self.economic_analyzer = EconomicAnalyzer()
        self.control_system = AdvancedControlSystem()

        # Simulation state
        self.simulation_running = False
        self.simulation_thread = None
        self.time_data = []
        self.current_data = []
        self.torque_data = []
        self.speed_data = []
        self.temp_data = []
        self.power_data = []

        # Create GUI
        self.create_menu()
        self.create_main_interface()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="New Simulation", command=self.reset_simulation)
        file_menu.add_command(label="Save Results", command=self.save_results)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Tools menu
        tools_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Tools", menu=tools_menu)
        tools_menu.add_command(label="Furnace Problem Solver", command=self.show_furnace_solver)
        tools_menu.add_command(label="Loss Analysis", command=self.show_loss_analysis)
        tools_menu.add_command(label="Mechanical Stress", command=self.show_stress_analysis)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

    def create_main_interface(self):
        """Create main tabbed interface"""
        # Create notebook (tabbed interface)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Create tabs
        self.create_furnace_tab()
        self.create_simulation_tab()
        self.create_visualization_tab()
        self.create_control_tab()
        self.create_economic_tab()
        self.create_multiphysics_tab()

    def create_furnace_tab(self):
        """Create induction furnace problem tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Induction Furnace")

        # Title
        title_label = ttk.Label(frame, text="Induction Furnace Problem Solver",
                               font=('Arial', 16, 'bold'))
        title_label.pack(pady=10)

        # Problem description
        desc_frame = ttk.LabelFrame(frame, text="Problem Description", padding=10)
        desc_frame.pack(fill='x', padx=10, pady=5)

        desc_text = """A low-frequency induction furnace has a secondary voltage of 20V and takes
600 kW at 0.6 p.f. when the hearth is full. If the secondary voltage is kept constant,
determine the power absorbed and the p.f. when the hearth is half-full.
Assume that the resistance of the secondary circuit is doubled but the reactance remains the same."""

        ttk.Label(desc_frame, text=desc_text, wraplength=800, justify='left').pack()

        # Input parameters
        input_frame = ttk.LabelFrame(frame, text="Input Parameters", padding=10)
        input_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(input_frame, text="Secondary Voltage (V):").grid(row=0, column=0, sticky='w', pady=2)
        self.furnace_voltage = ttk.Entry(input_frame, width=15)
        self.furnace_voltage.insert(0, "20")
        self.furnace_voltage.grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(input_frame, text="Full Hearth Power (kW):").grid(row=1, column=0, sticky='w', pady=2)
        self.furnace_power = ttk.Entry(input_frame, width=15)
        self.furnace_power.insert(0, "600")
        self.furnace_power.grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(input_frame, text="Full Hearth P.F.:").grid(row=2, column=0, sticky='w', pady=2)
        self.furnace_pf = ttk.Entry(input_frame, width=15)
        self.furnace_pf.insert(0, "0.6")
        self.furnace_pf.grid(row=2, column=1, padx=5, pady=2)

        # Solve button
        ttk.Button(input_frame, text="Solve Problem", command=self.solve_furnace_problem).grid(
            row=3, column=0, columnspan=2, pady=10)

        # Results
        self.furnace_results = scrolledtext.ScrolledText(frame, height=20, width=100, font=('Courier', 10))
        self.furnace_results.pack(fill='both', expand=True, padx=10, pady=5)

    def create_simulation_tab(self):
        """Create simulation parameters tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Simulation Setup")

        # Left panel - Parameters
        left_frame = ttk.Frame(frame)
        left_frame.pack(side='left', fill='both', expand=True, padx=5)

        # Electrical parameters
        elec_frame = ttk.LabelFrame(left_frame, text="Electrical Parameters", padding=10)
        elec_frame.pack(fill='x', pady=5)

        params = [
            ("Supply Voltage (V):", "voltage", "400"),
            ("Frequency (Hz):", "frequency", "50"),
            ("Stator Resistance (Ω):", "r_stator", "0.5"),
            ("Rotor Resistance (Ω):", "r_rotor", "0.3"),
            ("Stator Inductance (H):", "l_stator", "0.05"),
        ]

        self.param_entries = {}
        for i, (label, key, default) in enumerate(params):
            ttk.Label(elec_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)
            entry = ttk.Entry(elec_frame, width=15)
            entry.insert(0, default)
            entry.grid(row=i, column=1, padx=5, pady=2)
            self.param_entries[key] = entry

        # Mechanical parameters
        mech_frame = ttk.LabelFrame(left_frame, text="Mechanical Parameters", padding=10)
        mech_frame.pack(fill='x', pady=5)

        mech_params = [
            ("Moment of Inertia (kg·m²):", "inertia", "0.1"),
            ("Friction Coefficient:", "friction", "0.01"),
            ("Load Torque (N·m):", "load_torque", "50"),
        ]

        for i, (label, key, default) in enumerate(mech_params):
            ttk.Label(mech_frame, text=label).grid(row=i, column=0, sticky='w', pady=2)
            entry = ttk.Entry(mech_frame, width=15)
            entry.insert(0, default)
            entry.grid(row=i, column=1, padx=5, pady=2)
            self.param_entries[key] = entry

        # Right panel - Solver settings
        right_frame = ttk.Frame(frame)
        right_frame.pack(side='right', fill='both', expand=True, padx=5)

        solver_frame = ttk.LabelFrame(right_frame, text="Solver Settings", padding=10)
        solver_frame.pack(fill='x', pady=5)

        ttk.Label(solver_frame, text="ODE Solver:").grid(row=0, column=0, sticky='w', pady=2)
        self.solver_method = ttk.Combobox(solver_frame, values=["RK45", "Euler"], state='readonly', width=13)
        self.solver_method.set("RK45")
        self.solver_method.grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(solver_frame, text="Time Span (s):").grid(row=1, column=0, sticky='w', pady=2)
        self.time_span = ttk.Entry(solver_frame, width=15)
        self.time_span.insert(0, "5.0")
        self.time_span.grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(solver_frame, text="Time Steps:").grid(row=2, column=0, sticky='w', pady=2)
        self.time_steps = ttk.Entry(solver_frame, width=15)
        self.time_steps.insert(0, "500")
        self.time_steps.grid(row=2, column=1, padx=5, pady=2)

        # Control buttons
        button_frame = ttk.Frame(right_frame)
        button_frame.pack(pady=20)

        ttk.Button(button_frame, text="Start", command=self.start_simulation,
                  width=10).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Stop", command=self.stop_simulation,
                  width=10).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Reset", command=self.reset_simulation,
                  width=10).pack(side='left', padx=5)

        # Status
        self.status_label = ttk.Label(right_frame, text="Status: Ready", font=('Arial', 10))
        self.status_label.pack(pady=10)

        self.progress = ttk.Progressbar(right_frame, mode='indeterminate')
        self.progress.pack(fill='x', padx=10, pady=5)

    def create_visualization_tab(self):
        """Create visualization tab with dynamic graphs"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Visualization")

        # Create matplotlib figure with subplots
        self.fig = Figure(figsize=(12, 8), dpi=100)

        self.ax1 = self.fig.add_subplot(3, 2, 1)
        self.ax2 = self.fig.add_subplot(3, 2, 2)
        self.ax3 = self.fig.add_subplot(3, 2, 3)
        self.ax4 = self.fig.add_subplot(3, 2, 4)
        self.ax5 = self.fig.add_subplot(3, 2, 5)
        self.ax6 = self.fig.add_subplot(3, 2, 6)

        self.fig.tight_layout(pad=3.0)

        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, frame)
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        # Initialize plots
        self.init_plots()

    def create_control_tab(self):
        """Create advanced control tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Advanced Control")

        # Control parameters
        control_frame = ttk.LabelFrame(frame, text="PID Control Parameters", padding=10)
        control_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(control_frame, text="Proportional Gain (Kp):").grid(row=0, column=0, sticky='w', pady=2)
        self.kp_slider = ttk.Scale(control_frame, from_=0, to=10, orient='horizontal', length=300)
        self.kp_slider.set(1.0)
        self.kp_slider.grid(row=0, column=1, padx=5, pady=2)
        self.kp_label = ttk.Label(control_frame, text="1.0")
        self.kp_label.grid(row=0, column=2, padx=5)
        self.kp_slider.configure(command=lambda v: self.kp_label.config(text=f"{float(v):.2f}"))

        ttk.Label(control_frame, text="Integral Gain (Ki):").grid(row=1, column=0, sticky='w', pady=2)
        self.ki_slider = ttk.Scale(control_frame, from_=0, to=5, orient='horizontal', length=300)
        self.ki_slider.set(0.5)
        self.ki_slider.grid(row=1, column=1, padx=5, pady=2)
        self.ki_label = ttk.Label(control_frame, text="0.5")
        self.ki_label.grid(row=1, column=2, padx=5)
        self.ki_slider.configure(command=lambda v: self.ki_label.config(text=f"{float(v):.2f}"))

        ttk.Label(control_frame, text="Derivative Gain (Kd):").grid(row=2, column=0, sticky='w', pady=2)
        self.kd_slider = ttk.Scale(control_frame, from_=0, to=2, orient='horizontal', length=300)
        self.kd_slider.set(0.1)
        self.kd_slider.grid(row=2, column=1, padx=5, pady=2)
        self.kd_label = ttk.Label(control_frame, text="0.1")
        self.kd_label.grid(row=2, column=2, padx=5)
        self.kd_slider.configure(command=lambda v: self.kd_label.config(text=f"{float(v):.2f}"))

        # Thermal derating
        thermal_frame = ttk.LabelFrame(frame, text="Thermal Derating", padding=10)
        thermal_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(thermal_frame, text="Derating Start Temp (°C):").grid(row=0, column=0, sticky='w', pady=2)
        self.derate_temp = ttk.Entry(thermal_frame, width=15)
        self.derate_temp.insert(0, "90")
        self.derate_temp.grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(thermal_frame, text="Maximum Temp (°C):").grid(row=1, column=0, sticky='w', pady=2)
        self.max_temp = ttk.Entry(thermal_frame, width=15)
        self.max_temp.insert(0, "120")
        self.max_temp.grid(row=1, column=1, padx=5, pady=2)

        # Power consumption monitoring
        power_frame = ttk.LabelFrame(frame, text="Power Consumption Monitoring", padding=10)
        power_frame.pack(fill='both', expand=True, padx=10, pady=5)

        self.power_text = scrolledtext.ScrolledText(power_frame, height=15, width=80, font=('Courier', 9))
        self.power_text.pack(fill='both', expand=True)

    def create_economic_tab(self):
        """Create economic analysis tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Economic Analysis")

        # Cost parameters
        cost_frame = ttk.LabelFrame(frame, text="Cost Parameters", padding=10)
        cost_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(cost_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky='w', pady=2)
        self.elec_cost = ttk.Entry(cost_frame, width=15)
        self.elec_cost.insert(0, "0.12")
        self.elec_cost.grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(cost_frame, text="Maintenance Cost ($/hour):").grid(row=1, column=0, sticky='w', pady=2)
        self.maint_cost = ttk.Entry(cost_frame, width=15)
        self.maint_cost.insert(0, "5.0")
        self.maint_cost.grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(cost_frame, text="Equipment Cost ($):").grid(row=2, column=0, sticky='w', pady=2)
        self.equip_cost = ttk.Entry(cost_frame, width=15)
        self.equip_cost.insert(0, "50000")
        self.equip_cost.grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(cost_frame, text="Annual Operating Hours:").grid(row=3, column=0, sticky='w', pady=2)
        self.annual_hours = ttk.Entry(cost_frame, width=15)
        self.annual_hours.insert(0, "8760")
        self.annual_hours.grid(row=3, column=1, padx=5, pady=2)

        ttk.Label(cost_frame, text="Average Power (kW):").grid(row=4, column=0, sticky='w', pady=2)
        self.avg_power = ttk.Entry(cost_frame, width=15)
        self.avg_power.insert(0, "50")
        self.avg_power.grid(row=4, column=1, padx=5, pady=2)

        ttk.Label(cost_frame, text="Efficiency:").grid(row=5, column=0, sticky='w', pady=2)
        self.efficiency = ttk.Entry(cost_frame, width=15)
        self.efficiency.insert(0, "0.92")
        self.efficiency.grid(row=5, column=1, padx=5, pady=2)

        # Calculate button
        ttk.Button(cost_frame, text="Calculate Costs", command=self.calculate_economics).grid(
            row=6, column=0, columnspan=2, pady=10)

        # Results
        self.economic_results = scrolledtext.ScrolledText(frame, height=20, width=100, font=('Courier', 10))
        self.economic_results.pack(fill='both', expand=True, padx=10, pady=5)

    def create_multiphysics_tab(self):
        """Create multi-physics simulation tab"""
        frame = ttk.Frame(self.notebook)
        self.notebook.add(frame, text="Multi-Physics")

        # Description
        desc_frame = ttk.LabelFrame(frame, text="Multi-Physics Simulation", padding=10)
        desc_frame.pack(fill='x', padx=10, pady=5)

        desc = """This module performs coupled electromagnetic-thermal-mechanical simulation.
It solves heat transfer equations simultaneously with electrical equations for accurate
temperature prediction and evaluates shaft torque transients and bearing loads."""

        ttk.Label(desc_frame, text=desc, wraplength=800, justify='left').pack()

        # Analysis buttons
        button_frame = ttk.Frame(frame)
        button_frame.pack(pady=10)

        ttk.Button(button_frame, text="Loss Breakdown Analysis",
                  command=self.run_loss_analysis, width=25).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Mechanical Stress Analysis",
                  command=self.run_stress_analysis, width=25).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Thermal Analysis",
                  command=self.run_thermal_analysis, width=25).pack(side='left', padx=5)

        # Results
        self.multiphysics_results = scrolledtext.ScrolledText(frame, height=25, width=100,
                                                              font=('Courier', 9))
        self.multiphysics_results.pack(fill='both', expand=True, padx=10, pady=5)

    def init_plots(self):
        """Initialize plot axes"""
        self.ax1.set_title('Current (d-q axes)')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Current (A)')
        self.ax1.grid(True)

        self.ax2.set_title('Electromagnetic Torque')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Torque (N·m)')
        self.ax2.grid(True)

        self.ax3.set_title('Rotor Speed')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Speed (rad/s)')
        self.ax3.grid(True)

        self.ax4.set_title('Temperature')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Temperature (°C)')
        self.ax4.grid(True)

        self.ax5.set_title('Power Consumption')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Power (W)')
        self.ax5.grid(True)

        self.ax6.set_title('Loss Distribution')
        self.ax6.set_ylabel('Loss (W)')
        self.ax6.grid(True)

    def solve_furnace_problem(self):
        """Solve the induction furnace problem"""
        try:
            # Update solver parameters
            self.furnace_solver.V2 = float(self.furnace_voltage.get())
            self.furnace_solver.P_full = float(self.furnace_power.get()) * 1000
            self.furnace_solver.pf_full = float(self.furnace_pf.get())

            # Solve
            results = self.furnace_solver.solve()

            # Display results
            self.furnace_results.delete('1.0', tk.END)

            output = "=" * 80 + "\n"
            output += "INDUCTION FURNACE PROBLEM SOLUTION\n"
            output += "=" * 80 + "\n\n"

            output += "FULL HEARTH CONDITIONS:\n"
            output += "-" * 80 + "\n"
            output += f"Power:                {results['full_hearth']['power']/1000:.2f} kW\n"
            output += f"Power Factor:         {results['full_hearth']['pf']:.4f}\n"
            output += f"Current (RMS):        {results['full_hearth']['current']:.2f} A\n"
            output += f"Impedance:            {results['full_hearth']['impedance']:.6f} Ω\n"
            output += f"Resistance:           {results['full_hearth']['resistance']:.6f} Ω\n"
            output += f"Reactance:            {results['full_hearth']['reactance']:.6f} Ω\n"

            output += "\n" + "=" * 80 + "\n"
            output += "HALF-FULL HEARTH CONDITIONS:\n"
            output += "-" * 80 + "\n"
            output += f"Power:                {results['half_hearth']['power']/1000:.2f} kW\n"
            output += f"Power Factor:         {results['half_hearth']['pf']:.4f}\n"
            output += f"Current (RMS):        {results['half_hearth']['current']:.2f} A\n"
            output += f"Impedance:            {results['half_hearth']['impedance']:.6f} Ω\n"
            output += f"Resistance:           {results['half_hearth']['resistance']:.6f} Ω (doubled)\n"
            output += f"Reactance:            {results['half_hearth']['reactance']:.6f} Ω (unchanged)\n"

            output += "\n" + "=" * 80 + "\n"
            output += "ANALYSIS:\n"
            output += "-" * 80 + "\n"

            power_change = ((results['half_hearth']['power'] - results['full_hearth']['power']) /
                          results['full_hearth']['power'] * 100)
            output += f"Power Change:         {power_change:.2f}%\n"

            pf_change = ((results['half_hearth']['pf'] - results['full_hearth']['pf']) /
                        results['full_hearth']['pf'] * 100)
            output += f"Power Factor Change:  {pf_change:.2f}%\n"

            current_change = ((results['half_hearth']['current'] - results['full_hearth']['current']) /
                            results['full_hearth']['current'] * 100)
            output += f"Current Change:       {current_change:.2f}%\n"

            output += "\n" + "=" * 80 + "\n"
            output += "CONCLUSION:\n"
            output += "-" * 80 + "\n"
            output += f"When the hearth is half-full:\n"
            output += f"  • Power absorbed: {results['half_hearth']['power']/1000:.2f} kW\n"
            output += f"  • Power factor:   {results['half_hearth']['pf']:.4f}\n"
            output += "=" * 80 + "\n"

            self.furnace_results.insert('1.0', output)

        except Exception as e:
            messagebox.showerror("Error", f"Error solving problem: {str(e)}")

    def start_simulation(self):
        """Start the simulation"""
        if self.simulation_running:
            messagebox.showwarning("Warning", "Simulation already running!")
            return

        self.simulation_running = True
        self.status_label.config(text="Status: Running...")
        self.progress.start()

        # Clear previous data
        self.time_data = []
        self.current_data = []
        self.torque_data = []
        self.speed_data = []
        self.temp_data = []
        self.power_data = []

        # Start simulation in separate thread
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()

    def stop_simulation(self):
        """Stop the simulation"""
        self.simulation_running = False
        self.status_label.config(text="Status: Stopped")
        self.progress.stop()

    def reset_simulation(self):
        """Reset the simulation"""
        self.stop_simulation()
        self.physics_engine.reset_state()

        # Clear plots
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]:
            ax.clear()
        self.init_plots()
        self.canvas.draw()

        self.status_label.config(text="Status: Reset")

    def run_simulation(self):
        """Run the simulation"""
        try:
            # Get parameters
            self.physics_engine.V_supply = float(self.param_entries['voltage'].get())
            self.physics_engine.frequency = float(self.param_entries['frequency'].get())
            self.physics_engine.R_stator = float(self.param_entries['r_stator'].get())
            self.physics_engine.R_rotor = float(self.param_entries['r_rotor'].get())
            self.physics_engine.L_stator = float(self.param_entries['l_stator'].get())
            self.physics_engine.J = float(self.param_entries['inertia'].get())
            self.physics_engine.B = float(self.param_entries['friction'].get())
            self.physics_engine.load_torque = float(self.param_entries['load_torque'].get())

            # Update control parameters
            self.control_system.Kp = self.kp_slider.get()
            self.control_system.Ki = self.ki_slider.get()
            self.control_system.Kd = self.kd_slider.get()

            # Simulation parameters
            t_end = float(self.time_span.get())
            n_steps = int(self.time_steps.get())
            t_eval = np.linspace(0, t_end, n_steps)

            # Initial conditions
            y0 = [0, 0, 0, 0, self.physics_engine.T_ambient]

            # Voltage inputs (RMS values)
            V_d = self.physics_engine.V_supply / np.sqrt(2)
            V_q = 0
            T_load = self.physics_engine.load_torque

            # Select solver
            solver_name = self.solver_method.get()

            if solver_name == "RK45":
                sol = self.ode_solver.rk45(
                    self.physics_engine.electromagnetic_model,
                    y0, [0, t_end], t_eval, args=(V_d, V_q, T_load)
                )
            else:
                sol = self.ode_solver.euler(
                    self.physics_engine.electromagnetic_model,
                    y0, [0, t_end], t_eval, args=(V_d, V_q, T_load)
                )

            if sol.success:
                # Extract results
                self.time_data = sol.t
                i_d = sol.y[0]
                i_q = sol.y[1]
                omega = sol.y[2]
                theta = sol.y[3]
                temperature = sol.y[4]

                # Calculate torque and power
                torque = 1.5 * self.physics_engine.pole_pairs * self.physics_engine.L_mutual * i_d * i_q
                power = V_d * i_d + V_q * i_q

                self.current_data = [i_d, i_q]
                self.torque_data = torque
                self.speed_data = omega
                self.temp_data = temperature
                self.power_data = power

                # Update plots
                self.root.after(0, self.update_plots)

                # Update power consumption text
                self.root.after(0, self.update_power_consumption)

                self.status_label.config(text="Status: Completed")
            else:
                self.status_label.config(text="Status: Failed")
                messagebox.showerror("Error", "Simulation failed!")

        except Exception as e:
            self.status_label.config(text="Status: Error")
            messagebox.showerror("Error", f"Simulation error: {str(e)}")

        finally:
            self.simulation_running = False
            self.progress.stop()

    def update_plots(self):
        """Update all plots"""
        # Clear axes
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
            ax.clear()

        # Plot 1: Currents
        self.ax1.plot(self.time_data, self.current_data[0], 'b-', label='i_d', linewidth=1.5)
        self.ax1.plot(self.time_data, self.current_data[1], 'r-', label='i_q', linewidth=1.5)
        self.ax1.set_title('Current (d-q axes)')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Current (A)')
        self.ax1.legend()
        self.ax1.grid(True, alpha=0.3)

        # Plot 2: Torque
        self.ax2.plot(self.time_data, self.torque_data, 'g-', linewidth=1.5)
        self.ax2.set_title('Electromagnetic Torque')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Torque (N·m)')
        self.ax2.grid(True, alpha=0.3)

        # Plot 3: Speed
        self.ax3.plot(self.time_data, self.speed_data, 'm-', linewidth=1.5)
        self.ax3.set_title('Rotor Speed')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Speed (rad/s)')
        self.ax3.grid(True, alpha=0.3)

        # Plot 4: Temperature
        self.ax4.plot(self.time_data, self.temp_data, 'orange', linewidth=1.5)
        self.ax4.axhline(y=float(self.derate_temp.get()), color='y', linestyle='--',
                        label='Derating Start', linewidth=1)
        self.ax4.axhline(y=float(self.max_temp.get()), color='r', linestyle='--',
                        label='Max Temp', linewidth=1)
        self.ax4.set_title('Temperature')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Temperature (°C)')
        self.ax4.legend()
        self.ax4.grid(True, alpha=0.3)

        # Plot 5: Power
        self.ax5.plot(self.time_data, self.power_data, 'c-', linewidth=1.5)
        self.ax5.set_title('Power Consumption')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Power (W)')
        self.ax5.grid(True, alpha=0.3)

        # Plot 6: Loss breakdown (bar chart)
        if len(self.current_data) > 0:
            # Calculate losses at final time
            losses = self.physics_engine.calculate_losses(
                self.current_data[0][-1], self.current_data[1][-1],
                self.speed_data[-1], self.power_data[-1], 0
            )

            loss_names = ['Copper\nStator', 'Copper\nRotor', 'Hysteresis',
                         'Eddy\nCurrent', 'Friction', 'Windage', 'Stray']
            loss_values = [
                losses['copper_stator'], losses['copper_rotor'],
                losses['hysteresis'], losses['eddy_current'],
                losses['friction'], losses['windage'], losses['stray']
            ]

            colors = ['#FF6B6B', '#FFA07A', '#FFD93D', '#6BCB77', '#4D96FF', '#9D84B7', '#C0C0C0']
            self.ax6.bar(loss_names, loss_values, color=colors, alpha=0.7)
            self.ax6.set_title('Loss Distribution')
            self.ax6.set_ylabel('Loss (W)')
            self.ax6.grid(True, alpha=0.3, axis='y')

            # Rotate x labels
            for tick in self.ax6.get_xticklabels():
                tick.set_rotation(45)
                tick.set_ha('right')

        self.fig.tight_layout()
        self.canvas.draw()

    def update_power_consumption(self):
        """Update power consumption monitoring"""
        if len(self.time_data) == 0:
            return

        self.power_text.delete('1.0', tk.END)

        output = "=" * 70 + "\n"
        output += "POWER CONSUMPTION MONITORING\n"
        output += "=" * 70 + "\n\n"

        # Calculate statistics
        avg_power = np.mean(self.power_data)
        max_power = np.max(self.power_data)
        min_power = np.min(self.power_data)

        output += f"Average Power:        {avg_power:.2f} W ({avg_power/1000:.2f} kW)\n"
        output += f"Maximum Power:        {max_power:.2f} W ({max_power/1000:.2f} kW)\n"
        output += f"Minimum Power:        {min_power:.2f} W ({min_power/1000:.2f} kW)\n\n"

        # Energy consumption
        time_duration = self.time_data[-1] - self.time_data[0]
        energy = np.trapz(self.power_data, self.time_data) / 3600  # Wh

        output += f"Simulation Duration:  {time_duration:.2f} s\n"
        output += f"Energy Consumed:      {energy:.2f} Wh ({energy/1000:.4f} kWh)\n\n"

        # Loss breakdown
        if len(self.current_data) > 0:
            losses = self.physics_engine.calculate_losses(
                self.current_data[0][-1], self.current_data[1][-1],
                self.speed_data[-1], self.power_data[-1], 0
            )

            output += "=" * 70 + "\n"
            output += "DETAILED LOSS BREAKDOWN\n"
            output += "=" * 70 + "\n\n"

            output += f"Copper Loss (Stator):  {losses['copper_stator']:.2f} W\n"
            output += f"Copper Loss (Rotor):   {losses['copper_rotor']:.2f} W\n"
            output += f"Hysteresis Loss:       {losses['hysteresis']:.2f} W\n"
            output += f"Eddy Current Loss:     {losses['eddy_current']:.2f} W\n"
            output += f"Friction Loss:         {losses['friction']:.2f} W\n"
            output += f"Windage Loss:          {losses['windage']:.2f} W\n"
            output += f"Stray Load Loss:       {losses['stray']:.2f} W\n"
            output += "-" * 70 + "\n"
            output += f"Total Losses:          {losses['total']:.2f} W\n\n"

            if avg_power > 0:
                efficiency = ((avg_power - losses['total']) / avg_power) * 100
                output += f"Efficiency:            {efficiency:.2f}%\n"

        # Thermal derating
        output += "\n" + "=" * 70 + "\n"
        output += "THERMAL DERATING STATUS\n"
        output += "=" * 70 + "\n\n"

        current_temp = self.temp_data[-1]
        derate_start = float(self.derate_temp.get())
        max_temp = float(self.max_temp.get())

        output += f"Current Temperature:   {current_temp:.2f} °C\n"
        output += f"Derating Start:        {derate_start:.2f} °C\n"
        output += f"Maximum Temperature:   {max_temp:.2f} °C\n\n"

        if current_temp < derate_start:
            output += "Status: NORMAL OPERATION (No derating)\n"
        elif current_temp < max_temp:
            derate_factor = 1 - (current_temp - derate_start) / (max_temp - derate_start)
            output += f"Status: DERATING ACTIVE ({derate_factor*100:.1f}% capacity)\n"
        else:
            output += "Status: OVERTEMPERATURE SHUTDOWN\n"

        self.power_text.insert('1.0', output)

    def calculate_economics(self):
        """Calculate economic analysis"""
        try:
            # Update analyzer parameters
            self.economic_analyzer.electricity_cost = float(self.elec_cost.get())
            self.economic_analyzer.maintenance_cost_per_hour = float(self.maint_cost.get())
            self.economic_analyzer.equipment_cost = float(self.equip_cost.get())

            hours = float(self.annual_hours.get())
            power_kw = float(self.avg_power.get())
            eff = float(self.efficiency.get())

            # Calculate operating cost
            op_cost = self.economic_analyzer.calculate_operating_cost(power_kw, hours, eff)

            # Calculate lifecycle cost
            lc_cost = self.economic_analyzer.calculate_lifecycle_cost(hours, power_kw, eff)

            # Display results
            self.economic_results.delete('1.0', tk.END)

            output = "=" * 80 + "\n"
            output += "ECONOMIC ANALYSIS RESULTS\n"
            output += "=" * 80 + "\n\n"

            output += "ANNUAL OPERATING COSTS:\n"
            output += "-" * 80 + "\n"
            output += f"Energy Cost:           ${op_cost['energy_cost']:,.2f}\n"
            output += f"Maintenance Cost:      ${op_cost['maintenance_cost']:,.2f}\n"
            output += f"Energy Loss Cost:      ${op_cost['loss_cost']:,.2f}\n"
            output += f"Total Annual Cost:     ${op_cost['total_cost']:,.2f}\n\n"
            output += f"Energy Consumed:       {op_cost['energy_consumed']:,.2f} kWh\n"

            output += "\n" + "=" * 80 + "\n"
            output += f"LIFECYCLE COST ANALYSIS ({self.economic_analyzer.lifetime_years} years)\n"
            output += "=" * 80 + "\n\n"
            output += f"Equipment Cost:        ${lc_cost['equipment_cost']:,.2f}\n"
            output += f"Annual Operating Cost: ${lc_cost['annual_operating_cost']:,.2f}\n"
            output += f"PV Operating Cost:     ${lc_cost['pv_operating_cost']:,.2f}\n"
            output += "-" * 80 + "\n"
            output += f"Total Lifecycle Cost:  ${lc_cost['total_lifecycle_cost']:,.2f}\n"
            output += f"Cost per Year:         ${lc_cost['cost_per_year']:,.2f}\n"

            # ROI analysis
            output += "\n" + "=" * 80 + "\n"
            output += "RETURN ON INVESTMENT ANALYSIS\n"
            output += "=" * 80 + "\n\n"

            # Calculate payback period
            annual_savings = 0  # Would need baseline for comparison
            if annual_savings > 0:
                payback = lc_cost['equipment_cost'] / annual_savings
                output += f"Simple Payback Period: {payback:.2f} years\n"

            output += f"Discount Rate:         {self.economic_analyzer.discount_rate*100:.1f}%\n"

            self.economic_results.insert('1.0', output)

        except Exception as e:
            messagebox.showerror("Error", f"Error calculating economics: {str(e)}")

    def run_loss_analysis(self):
        """Run detailed loss analysis"""
        if len(self.current_data) == 0:
            messagebox.showwarning("Warning", "Please run simulation first!")
            return

        self.multiphysics_results.delete('1.0', tk.END)

        output = "=" * 80 + "\n"
        output += "DETAILED LOSS BREAKDOWN ANALYSIS\n"
        output += "=" * 80 + "\n\n"

        # Calculate losses at different time points
        n_points = min(10, len(self.time_data))
        indices = np.linspace(0, len(self.time_data)-1, n_points, dtype=int)

        for i, idx in enumerate(indices):
            losses = self.physics_engine.calculate_losses(
                self.current_data[0][idx], self.current_data[1][idx],
                self.speed_data[idx],
                self.power_data[idx] if len(self.power_data) > idx else 0,
                0
            )

            output += f"Time: {self.time_data[idx]:.2f} s\n"
            output += "-" * 80 + "\n"

            # Calculate percentages
            total = losses['total']
            if total > 0:
                output += f"Copper Loss (Stator):  {losses['copper_stator']:8.2f} W "
                output += f"({losses['copper_stator']/total*100:5.1f}%)\n"

                output += f"Copper Loss (Rotor):   {losses['copper_rotor']:8.2f} W "
                output += f"({losses['copper_rotor']/total*100:5.1f}%)\n"

                output += f"Hysteresis Loss:       {losses['hysteresis']:8.2f} W "
                output += f"({losses['hysteresis']/total*100:5.1f}%)\n"

                output += f"Eddy Current Loss:     {losses['eddy_current']:8.2f} W "
                output += f"({losses['eddy_current']/total*100:5.1f}%)\n"

                output += f"Friction Loss:         {losses['friction']:8.2f} W "
                output += f"({losses['friction']/total*100:5.1f}%)\n"

                output += f"Windage Loss:          {losses['windage']:8.2f} W "
                output += f"({losses['windage']/total*100:5.1f}%)\n"

                output += f"Stray Load Loss:       {losses['stray']:8.2f} W "
                output += f"({losses['stray']/total*100:5.1f}%)\n"

                output += "-" * 80 + "\n"
                output += f"Total Losses:          {total:8.2f} W\n"

            output += "\n"

        self.multiphysics_results.insert('1.0', output)

    def run_stress_analysis(self):
        """Run mechanical stress analysis"""
        if len(self.torque_data) == 0:
            messagebox.showwarning("Warning", "Please run simulation first!")
            return

        self.multiphysics_results.delete('1.0', tk.END)

        output = "=" * 80 + "\n"
        output += "MECHANICAL STRESS ANALYSIS\n"
        output += "=" * 80 + "\n\n"

        # Analyze peak, average, and transient conditions
        max_torque = np.max(np.abs(self.torque_data))
        avg_torque = np.mean(np.abs(self.torque_data))
        max_speed = np.max(np.abs(self.speed_data))

        output += "OPERATING CONDITIONS:\n"
        output += "-" * 80 + "\n"
        output += f"Maximum Torque:        {max_torque:.2f} N·m\n"
        output += f"Average Torque:        {avg_torque:.2f} N·m\n"
        output += f"Maximum Speed:         {max_speed:.2f} rad/s ({max_speed*60/(2*np.pi):.1f} RPM)\n\n"

        # Calculate stress at peak torque
        idx_max = np.argmax(np.abs(self.torque_data))
        stress_peak = self.physics_engine.calculate_mechanical_stress(
            self.torque_data[idx_max], self.speed_data[idx_max]
        )

        output += "PEAK LOAD STRESS ANALYSIS:\n"
        output += "-" * 80 + "\n"
        output += f"Shaft Shear Stress:    {stress_peak['shaft_shear_stress']/1e6:.2f} MPa\n"
        output += f"Bearing Radial Load:   {stress_peak['bearing_radial_load']:.2f} N\n"
        output += f"Bearing Axial Load:    {stress_peak['bearing_axial_load']:.2f} N\n"
        output += f"Safety Factor:         {stress_peak['safety_factor']:.2f}\n\n"

        # Calculate stress at average conditions
        idx_mid = len(self.torque_data) // 2
        stress_avg = self.physics_engine.calculate_mechanical_stress(
            avg_torque, self.speed_data[idx_mid]
        )

        output += "AVERAGE LOAD STRESS ANALYSIS:\n"
        output += "-" * 80 + "\n"
        output += f"Shaft Shear Stress:    {stress_avg['shaft_shear_stress']/1e6:.2f} MPa\n"
        output += f"Bearing Radial Load:   {stress_avg['bearing_radial_load']:.2f} N\n"
        output += f"Bearing Axial Load:    {stress_avg['bearing_axial_load']:.2f} N\n"
        output += f"Safety Factor:         {stress_avg['safety_factor']:.2f}\n\n"

        # Torque transients
        output += "TORQUE TRANSIENTS:\n"
        output += "-" * 80 + "\n"

        torque_rate = np.gradient(self.torque_data, self.time_data)
        max_transient = np.max(np.abs(torque_rate))

        output += f"Max Torque Rate:       {max_transient:.2f} N·m/s\n"

        # Fatigue analysis (simplified)
        torque_range = np.max(self.torque_data) - np.min(self.torque_data)
        output += f"Torque Range:          {torque_range:.2f} N·m\n\n"

        output += "DESIGN RECOMMENDATIONS:\n"
        output += "-" * 80 + "\n"

        if stress_peak['safety_factor'] < 2:
            output += "⚠ WARNING: Low safety factor - consider shaft reinforcement\n"
        elif stress_peak['safety_factor'] < 3:
            output += "⚡ CAUTION: Adequate but monitor for fatigue\n"
        else:
            output += "✓ SAFE: Safety factor within acceptable range\n"

        if stress_peak['bearing_radial_load'] > 5000:
            output += "⚠ WARNING: High bearing loads - verify bearing selection\n"
        else:
            output += "✓ SAFE: Bearing loads within typical range\n"

        self.multiphysics_results.insert('1.0', output)

    def run_thermal_analysis(self):
        """Run thermal analysis"""
        if len(self.temp_data) == 0:
            messagebox.showwarning("Warning", "Please run simulation first!")
            return

        self.multiphysics_results.delete('1.0', tk.END)

        output = "=" * 80 + "\n"
        output += "COUPLED ELECTROMAGNETIC-THERMAL ANALYSIS\n"
        output += "=" * 80 + "\n\n"

        # Temperature statistics
        max_temp = np.max(self.temp_data)
        avg_temp = np.mean(self.temp_data)
        final_temp = self.temp_data[-1]
        temp_rise = final_temp - self.physics_engine.T_ambient

        output += "TEMPERATURE PROFILE:\n"
        output += "-" * 80 + "\n"
        output += f"Ambient Temperature:   {self.physics_engine.T_ambient:.2f} °C\n"
        output += f"Maximum Temperature:   {max_temp:.2f} °C\n"
        output += f"Average Temperature:   {avg_temp:.2f} °C\n"
        output += f"Final Temperature:     {final_temp:.2f} °C\n"
        output += f"Temperature Rise:      {temp_rise:.2f} °C\n\n"

        # Thermal time constant
        # Find time to reach 63.2% of final value
        target_temp = self.physics_engine.T_ambient + 0.632 * temp_rise
        idx_tau = np.argmin(np.abs(self.temp_data - target_temp))
        tau = self.time_data[idx_tau] if idx_tau > 0 else 0

        output += "THERMAL CHARACTERISTICS:\n"
        output += "-" * 80 + "\n"
        output += f"Thermal Time Constant: {tau:.2f} s\n"
        output += f"Thermal Resistance:    {self.physics_engine.R_thermal:.2f} K/W\n"
        output += f"Thermal Capacitance:   {self.physics_engine.C_thermal:.2f} J/K\n\n"

        # Heat generation
        if len(self.current_data) > 0:
            final_losses = self.physics_engine.calculate_losses(
                self.current_data[0][-1], self.current_data[1][-1],
                self.speed_data[-1], self.power_data[-1], 0
            )

            output += "HEAT GENERATION:\n"
            output += "-" * 80 + "\n"
            output += f"Total Heat Generated:  {final_losses['total']:.2f} W\n"
            output += f"Heat Dissipated:       {temp_rise/self.physics_engine.R_thermal:.2f} W\n\n"

        # Thermal limits
        derate_temp = float(self.derate_temp.get())
        max_allowed = float(self.max_temp.get())

        output += "THERMAL LIMITS:\n"
        output += "-" * 80 + "\n"
        output += f"Derating Temperature:  {derate_temp:.2f} °C\n"
        output += f"Maximum Temperature:   {max_allowed:.2f} °C\n"
        output += f"Thermal Margin:        {max_allowed - max_temp:.2f} °C\n\n"

        if max_temp >= max_allowed:
            output += "⚠ OVERTEMPERATURE: Machine exceeds thermal limits!\n"
        elif max_temp >= derate_temp:
            output += "⚡ DERATING REQUIRED: Operating in derated region\n"
        else:
            margin_percent = (max_allowed - max_temp) / max_allowed * 100
            output += f"✓ NORMAL: {margin_percent:.1f}% thermal margin available\n"

        # Cooling recommendations
        output += "\n" + "=" * 80 + "\n"
        output += "COOLING RECOMMENDATIONS:\n"
        output += "=" * 80 + "\n\n"

        if temp_rise > 60:
            output += "• Consider enhanced cooling system (forced air/liquid cooling)\n"
            output += "• Verify ambient temperature and ventilation\n"
            output += "• Check for blocked cooling paths\n"
        elif temp_rise > 40:
            output += "• Monitor cooling system performance\n"
            output += "• Ensure adequate ventilation\n"
        else:
            output += "• Current cooling is adequate\n"
            output += "• Maintain regular cleaning of cooling surfaces\n"

        self.multiphysics_results.insert('1.0', output)

    def show_furnace_solver(self):
        """Show furnace problem solver"""
        self.notebook.select(0)

    def show_loss_analysis(self):
        """Show loss analysis"""
        self.notebook.select(5)
        self.run_loss_analysis()

    def show_stress_analysis(self):
        """Show stress analysis"""
        self.notebook.select(5)
        self.run_stress_analysis()

    def show_about(self):
        """Show about dialog"""
        about_text = """Advanced Induction Furnace & Multi-Physics Simulation
Version 1.0

Features:
• Induction furnace problem solver
• Multi-physics electromagnetic-thermal-mechanical simulation
• Advanced ODE solvers (RK45, Euler)
• Real-time dynamic visualization
• Economic lifecycle analysis
• Thermal derating and advanced control
• Detailed loss breakdown
• Mechanical stress analysis

Developed for electrical engineering education and research.
"""
        messagebox.showinfo("About", about_text)

    def save_results(self):
        """Save simulation results"""
        if len(self.time_data) == 0:
            messagebox.showwarning("Warning", "No simulation data to save!")
            return

        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"simulation_results_{timestamp}.txt"

            with open(filename, 'w') as f:
                f.write("SIMULATION RESULTS\n")
                f.write("=" * 80 + "\n\n")
                f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

                f.write("PARAMETERS:\n")
                f.write(f"Supply Voltage: {self.param_entries['voltage'].get()} V\n")
                f.write(f"Frequency: {self.param_entries['frequency'].get()} Hz\n")
                f.write(f"Load Torque: {self.param_entries['load_torque'].get()} N·m\n\n")

                f.write("RESULTS:\n")
                f.write(f"Final Speed: {self.speed_data[-1]:.2f} rad/s\n")
                f.write(f"Final Temperature: {self.temp_data[-1]:.2f} °C\n")
                f.write(f"Average Power: {np.mean(self.power_data):.2f} W\n")

            messagebox.showinfo("Success", f"Results saved to {filename}")

        except Exception as e:
            messagebox.showerror("Error", f"Error saving results: {str(e)}")

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        # Only handle resize for the main window
        if event.widget == self.root:
            # Update canvas size if needed
            try:
                if hasattr(self, 'canvas'):
                    self.canvas.get_tk_widget().update()
                    self.fig.tight_layout()
                    self.canvas.draw_idle()
            except:
                pass


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = MultiPhysicsGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
