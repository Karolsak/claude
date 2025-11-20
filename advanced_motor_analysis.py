"""
Advanced Motor Analysis and Simulation System
Comprehensive electrical machine analysis with multi-physics simulation
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from scipy.integrate import solve_ivp, odeint
from scipy.interpolate import interp1d
import math
from datetime import datetime

# ===============================================================================
# PART 1: FLYWHEEL AND MOTOR RATING PROBLEM SOLUTIONS
# ===============================================================================

def calculate_flywheel_inertia():
    """
    Solve the flywheel inertia problem:
    - Motor with flywheel supplies load torque of 150 kg-m for 15 sec
    - Motor torque limited to 85 kg-m
    - No load speed: 500 rpm (slip = 0%)
    - Full load slip: 10%
    - Find: Moment of inertia of flywheel

    Solution Approach:
    During load period, torque deficit = Load torque - Motor torque
    This deficit is supplied by flywheel kinetic energy
    Energy from flywheel = ½J(ω₁² - ω₂²)
    """
    # Given parameters
    T_load_kgm = 150  # Load torque in kg-m
    T_motor_kgm = 85  # Motor torque in kg-m
    t_load = 15  # Load period in seconds
    N_no_load = 500  # No load speed in rpm
    slip_full_load = 0.10  # 10% slip at full load

    # Convert kg-m to N·m (1 kg-m = 9.81 N·m)
    g = 9.81
    T_load = T_load_kgm * g  # 1471.5 N·m
    T_motor = T_motor_kgm * g  # 833.85 N·m

    # Calculate speeds in rad/s
    omega_no_load = N_no_load * 2 * math.pi / 60  # 52.36 rad/s
    N_full_load = N_no_load * (1 - slip_full_load)  # 450 rpm
    omega_full_load = N_full_load * 2 * math.pi / 60  # 47.12 rad/s

    # Torque deficit (supplied by flywheel)
    T_deficit = T_load - T_motor  # 637.65 N·m

    # Energy method:
    # During load period, flywheel decelerates from omega_no_load to omega_full_load
    # Energy released = ½J(ω₁² - ω₂²) = T_deficit × θ
    # where θ = average_speed × time = [(ω₁ + ω₂)/2] × t
    # Simplifying: J = T_deficit × t / (ω₁ - ω₂)

    J = T_deficit * t_load / (omega_no_load - omega_full_load)

    # Alternative calculation using energy balance
    energy_deficit = T_deficit * ((omega_no_load + omega_full_load) / 2) * t_load
    delta_omega_squared = omega_no_load**2 - omega_full_load**2
    J_alternative = 2 * energy_deficit / delta_omega_squared

    return {
        'J': J,
        'J_alternative': J_alternative,
        'T_load': T_load,
        'T_motor': T_motor,
        'T_deficit': T_deficit,
        'omega_no_load': omega_no_load,
        'omega_full_load': omega_full_load,
        'N_no_load': N_no_load,
        'N_full_load': N_full_load,
        'energy_stored': 0.5 * J * delta_omega_squared,
        't_load': t_load
    }

def calculate_motor_rating():
    """
    Solve the motor rating problem:
    - Acceleration: 0 to 2000 hp linearly over 20 sec
    - Full speed: 1000 hp for 40 sec
    - Deceleration: 330 to 0 hp over 10 sec (regenerative)
    - Rest: 0 hp for 20 sec
    Total cycle: 90 seconds

    RMS HP = sqrt(∫P²dt / T)
    """
    # Period 1: Acceleration (0-20s) - Linear rise from 0 to 2000 hp
    # P(t) = 100t
    # ∫P²dt = ∫(100t)²dt = 10000∫t²dt = 10000[t³/3]₀²⁰
    t1 = 20
    energy1 = 10000 * (t1**3 / 3)

    # Period 2: Full speed (20-60s) - Constant 1000 hp
    t2 = 40
    p2 = 1000
    energy2 = p2**2 * t2

    # Period 3: Deceleration (60-70s) - Linear fall from 330 to 0
    # P(t) = 330 - 33(t-60) for t in [60,70]
    # Let τ = t-60, then P(τ) = 330 - 33τ for τ in [0,10]
    # ∫P²dτ = ∫(330-33τ)²dτ = ∫(108900 - 21780τ + 1089τ²)dτ
    # = [108900τ - 10890τ² + 363τ³]₀¹⁰
    t3 = 10
    energy3 = 108900*t3 - 10890*t3**2 + 363*t3**3

    # Period 4: Rest (70-90s) - 0 hp
    energy4 = 0

    # Total cycle time and energy
    total_time = 20 + 40 + 10 + 20  # 90 seconds
    total_energy = energy1 + energy2 + energy3 + energy4

    # RMS horsepower rating
    rms_hp = math.sqrt(total_energy / total_time)

    return {
        'rms_hp': rms_hp,
        'energy1': energy1,
        'energy2': energy2,
        'energy3': energy3,
        'total_time': total_time,
        'total_energy': total_energy
    }

# ===============================================================================
# PART 2: MOTOR MODELS AND DIFFERENTIAL EQUATIONS
# ===============================================================================

class DCMotorModel:
    """DC Motor mathematical model with electrical, mechanical, thermal, and stress dynamics"""

    def __init__(self, Ra=0.5, La=0.01, Rf=100, Lf=10, J=0.5, B=0.1, Kt=1.0, Kb=1.0):
        """
        Parameters:
        Ra: Armature resistance (Ω)
        La: Armature inductance (H)
        Rf: Field resistance (Ω)
        Lf: Field inductance (H)
        J: Moment of inertia (kg·m²)
        B: Viscous friction coefficient (N·m·s/rad)
        Kt: Torque constant (N·m/A)
        Kb: Back-EMF constant (V·s/rad)
        """
        self.Ra = Ra
        self.La = La
        self.Rf = Rf
        self.Lf = Lf
        self.J = J
        self.B = B
        self.Kt = Kt
        self.Kb = Kb

        # Thermal parameters
        self.thermal_resistance = 2.0  # °C/W
        self.thermal_capacitance = 500  # J/°C
        self.ambient_temp = 25  # °C
        self.max_temp = 120  # °C
        self.derating_start_temp = 100  # °C

        # Loss coefficients
        self.iron_loss_coeff = 0.01
        self.mech_loss_coeff = 0.005
        self.stray_loss_coeff = 0.002

        # Mechanical parameters for stress analysis
        self.shaft_diameter = 0.05  # meters (50mm)
        self.shaft_length = 0.3  # meters
        self.bearing_distance = 0.25  # meters between bearings
        self.rotor_mass = 15.0  # kg
        self.shaft_material_modulus = 200e9  # Pa (steel)
        self.shaft_shear_modulus = 80e9  # Pa
        self.shaft_yield_strength = 250e6  # Pa

        # Vibration parameters
        self.natural_frequency = 100  # Hz
        self.damping_ratio = 0.05

        # Bearing parameters
        self.bearing_life_rating = 10000  # hours
        self.bearing_dynamic_load = 5000  # N

    def get_differential_equations(self, t, state, Va, Vf, Tload):
        """
        State vector: [ia, if, omega, theta, temp]
        ia: Armature current (A)
        if: Field current (A)
        omega: Angular velocity (rad/s)
        theta: Angular position (rad)
        temp: Motor temperature (°C)
        """
        ia, if_current, omega, theta, temp = state

        # Back-EMF
        eb = self.Kb * if_current * omega

        # Electrical equations
        dia_dt = (Va - ia * self.Ra - eb) / self.La
        dif_dt = (Vf - if_current * self.Rf) / self.Lf

        # Electromagnetic torque
        Te = self.Kt * if_current * ia

        # Mechanical equation
        domega_dt = (Te - Tload - self.B * omega) / self.J
        dtheta_dt = omega

        # Calculate losses
        copper_loss_armature = ia**2 * self.Ra
        copper_loss_field = if_current**2 * self.Rf
        iron_loss = self.iron_loss_coeff * omega**2
        mech_loss = self.mech_loss_coeff * omega**2
        stray_loss = self.stray_loss_coeff * (ia**2 + if_current**2)

        total_loss = (copper_loss_armature + copper_loss_field +
                     iron_loss + mech_loss + stray_loss)

        # Thermal equation
        dtemp_dt = (total_loss - (temp - self.ambient_temp) / self.thermal_resistance) / self.thermal_capacitance

        return [dia_dt, dif_dt, domega_dt, dtheta_dt, dtemp_dt]

    def get_losses(self, ia, if_current, omega):
        """Calculate detailed loss breakdown"""
        copper_loss_armature = ia**2 * self.Ra
        copper_loss_field = if_current**2 * self.Rf
        iron_loss = self.iron_loss_coeff * omega**2
        mech_loss = self.mech_loss_coeff * omega**2
        stray_loss = self.stray_loss_coeff * (ia**2 + if_current**2)

        return {
            'copper_armature': copper_loss_armature,
            'copper_field': copper_loss_field,
            'iron': iron_loss,
            'mechanical': mech_loss,
            'stray': stray_loss,
            'total': copper_loss_armature + copper_loss_field + iron_loss + mech_loss + stray_loss
        }

    def calculate_shaft_stress(self, torque, omega, acceleration):
        """
        Calculate shaft stress components
        Returns: Dict with torsional stress, bending stress, and combined stress
        """
        # Torsional shear stress: τ = 16T/(πd³)
        d = self.shaft_diameter
        torsional_stress = (16 * abs(torque)) / (math.pi * d**3)

        # Angular acceleration creates dynamic torque
        dynamic_torque = self.J * acceleration

        # Polar moment of inertia
        J_polar = math.pi * d**4 / 32

        # Shaft twist angle (radians)
        shaft_twist = (torque * self.shaft_length) / (self.shaft_shear_modulus * J_polar)

        # Critical speed (first mode)
        # ω_critical = (π²/L²) * sqrt(EI/m)
        I_area = math.pi * d**4 / 64
        mass_per_length = self.rotor_mass / self.shaft_length
        critical_speed = (math.pi**2 / self.shaft_length**2) * math.sqrt(
            (self.shaft_material_modulus * I_area) / mass_per_length
        )

        # Safety factor
        safety_factor = self.shaft_yield_strength / torsional_stress if torsional_stress > 0 else float('inf')

        return {
            'torsional_stress': torsional_stress,
            'dynamic_torque': dynamic_torque,
            'shaft_twist': shaft_twist,
            'critical_speed': critical_speed,
            'safety_factor': safety_factor,
            'max_stress': torsional_stress  # Can be extended with combined stress
        }

    def calculate_bearing_loads(self, torque, omega, acceleration):
        """
        Calculate bearing loads and life expectancy
        """
        # Radial load due to rotor weight (static)
        radial_load_static = (self.rotor_mass * 9.81) / 2  # Divided between two bearings

        # Dynamic load due to torque pulsations and unbalance
        # Assume 5% unbalance
        unbalance_factor = 0.05
        centrifugal_force = self.rotor_mass * unbalance_factor * self.shaft_diameter/2 * omega**2

        # Total radial load (vector sum, simplified to arithmetic sum)
        radial_load_total = radial_load_static + centrifugal_force/2

        # Axial load (typically magnetic pull, assume 10% of radial)
        axial_load = 0.1 * radial_load_total

        # Equivalent dynamic load: P = X*Fr + Y*Fa
        # For ball bearings, simplified: X=1, Y=0 for small axial loads
        equivalent_load = radial_load_total + 0.5 * axial_load

        # Bearing life calculation (L10 life in hours)
        # L10 = (C/P)^p * 10^6 / (60 * n)
        # where p=3 for ball bearings, C is dynamic load rating
        if omega > 0:
            rpm = omega * 60 / (2 * math.pi)
            p_exponent = 3  # For ball bearings
            bearing_life = ((self.bearing_dynamic_load / equivalent_load) ** p_exponent *
                          10**6 / (60 * rpm))
        else:
            bearing_life = float('inf')

        return {
            'radial_load': radial_load_total,
            'axial_load': axial_load,
            'equivalent_load': equivalent_load,
            'bearing_life_hours': bearing_life,
            'centrifugal_force': centrifugal_force
        }

    def calculate_thermal_derating(self, temperature):
        """Calculate power derating factor based on temperature"""
        if temperature < self.derating_start_temp:
            return 1.0
        elif temperature < self.max_temp:
            return 1.0 - (temperature - self.derating_start_temp) / \
                   (self.max_temp - self.derating_start_temp)
        else:
            return 0.0

    def calculate_vibration(self, omega, torque_ripple):
        """
        Calculate vibration amplitude based on torque ripple
        """
        # Natural frequency in rad/s
        omega_n = self.natural_frequency * 2 * math.pi

        # Forcing frequency
        omega_f = omega

        # Frequency ratio
        r = omega_f / omega_n if omega_n > 0 else 0

        # Magnification factor
        denominator = math.sqrt((1 - r**2)**2 + (2 * self.damping_ratio * r)**2)
        if denominator > 0:
            magnification = 1 / denominator
        else:
            magnification = 1

        # Vibration amplitude (simplified)
        base_amplitude = torque_ripple * 0.001  # Convert to mm
        vibration_amplitude = base_amplitude * magnification

        return {
            'amplitude': vibration_amplitude,
            'magnification_factor': magnification,
            'frequency_ratio': r,
            'natural_frequency': self.natural_frequency
        }

class InductionMotorModel:
    """Three-phase induction motor model"""

    def __init__(self, Rs=0.5, Rr=0.3, Ls=0.1, Lr=0.1, Lm=0.09, J=0.5, B=0.1, poles=4):
        self.Rs = Rs  # Stator resistance
        self.Rr = Rr  # Rotor resistance
        self.Ls = Ls  # Stator inductance
        self.Lr = Lr  # Rotor inductance
        self.Lm = Lm  # Magnetizing inductance
        self.J = J    # Inertia
        self.B = B    # Friction
        self.poles = poles

        # Thermal parameters
        self.thermal_resistance = 1.8
        self.thermal_capacitance = 600
        self.ambient_temp = 25

# ===============================================================================
# PART 3: ADVANCED MOTOR ANALYSIS GUI
# ===============================================================================

class AdvancedMotorAnalysisApp:
    """Main application class for comprehensive motor analysis"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Motor Analysis & Simulation System")
        self.root.geometry("1400x900")

        # Simulation state
        self.is_running = False
        self.simulation_time = 0
        self.time_history = []
        self.data_history = {
            'current': [], 'voltage': [], 'speed': [], 'torque': [],
            'power': [], 'efficiency': [], 'temperature': [],
            'losses': [], 'shaft_stress': [], 'bearing_load': [],
            'vibration': [], 'acceleration': [], 'safety_factor': []
        }
        self.previous_omega = 0

        # Motor model
        self.motor = DCMotorModel()

        # Initial state [ia, if, omega, theta, temp]
        self.state = [0, 0, 0, 0, 25]

        # Control parameters
        self.Va = tk.DoubleVar(value=220)
        self.Vf = tk.DoubleVar(value=220)
        self.Tload = tk.DoubleVar(value=10)
        self.simulation_method = tk.StringVar(value="RK45")

        # PID control variables
        self.integral_error = 0
        self.previous_error = 0
        self.control_output_history = []
        self.error_history = []

        # Economic parameters
        self.energy_cost = tk.DoubleVar(value=0.12)  # $/kWh
        self.maintenance_cost = tk.DoubleVar(value=100)  # $/year

        # Setup UI
        self.setup_ui()

        # Bind resize event for auto-scaling
        self.root.bind('<Configure>', self.on_window_resize)

    def setup_ui(self):
        """Create the main user interface"""

        # Create main notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create tabs
        self.tab_main = ttk.Frame(self.notebook)
        self.tab_dynamics = ttk.Frame(self.notebook)
        self.tab_thermal = ttk.Frame(self.notebook)
        self.tab_losses = ttk.Frame(self.notebook)
        self.tab_mechanical = ttk.Frame(self.notebook)
        self.tab_economic = ttk.Frame(self.notebook)
        self.tab_controls = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_main, text="Main Control")
        self.notebook.add(self.tab_dynamics, text="Dynamic Simulation")
        self.notebook.add(self.tab_thermal, text="Thermal Analysis")
        self.notebook.add(self.tab_losses, text="Loss Analysis")
        self.notebook.add(self.tab_mechanical, text="Mechanical Stress")
        self.notebook.add(self.tab_economic, text="Economic Analysis")
        self.notebook.add(self.tab_controls, text="Advanced Controls")

        # Setup each tab
        self.setup_main_tab()
        self.setup_dynamics_tab()
        self.setup_thermal_tab()
        self.setup_losses_tab()
        self.setup_mechanical_tab()
        self.setup_economic_tab()
        self.setup_controls_tab()

    def setup_main_tab(self):
        """Main control panel"""

        # Left panel - Controls
        left_frame = ttk.Frame(self.tab_main)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=10, pady=10)

        # Title
        title_label = ttk.Label(left_frame, text="Motor Control Panel",
                               font=('Arial', 14, 'bold'))
        title_label.pack(pady=10)

        # Flywheel Problem Solution Display
        flywheel_frame = ttk.LabelFrame(left_frame, text="Flywheel Inertia Problem Solution", padding=10)
        flywheel_frame.pack(fill=tk.X, pady=10)

        flywheel_result = calculate_flywheel_inertia()

        ttk.Label(flywheel_frame, text=f"Flywheel Inertia: {flywheel_result['J']:.2f} kg·m²",
                 font=('Arial', 11, 'bold'), foreground='blue').pack(anchor=tk.W)
        ttk.Label(flywheel_frame, text=f"Torque Deficit: {flywheel_result['T_deficit']:.2f} N·m").pack(anchor=tk.W)
        ttk.Label(flywheel_frame, text=f"Speed Range: {flywheel_result['N_full_load']:.0f} - {flywheel_result['N_no_load']:.0f} rpm").pack(anchor=tk.W)
        ttk.Label(flywheel_frame, text=f"Energy Stored: {flywheel_result['energy_stored']:.2f} J").pack(anchor=tk.W)

        # Motor Rating Solution Display
        rating_frame = ttk.LabelFrame(left_frame, text="Motor Rating Problem Solution", padding=10)
        rating_frame.pack(fill=tk.X, pady=10)

        rating_result = calculate_motor_rating()

        ttk.Label(rating_frame, text=f"RMS HP Rating: {rating_result['rms_hp']:.2f} hp",
                 font=('Arial', 11, 'bold'), foreground='blue').pack(anchor=tk.W)
        ttk.Label(rating_frame, text=f"Total Cycle Time: {rating_result['total_time']} seconds").pack(anchor=tk.W)
        ttk.Label(rating_frame, text=f"Total Energy: {rating_result['total_energy']:.2f} hp²·s").pack(anchor=tk.W)

        # Motor Parameters Frame
        param_frame = ttk.LabelFrame(left_frame, text="Motor Parameters", padding=10)
        param_frame.pack(fill=tk.X, pady=10)

        # Armature Voltage
        ttk.Label(param_frame, text="Armature Voltage (V):").grid(row=0, column=0, sticky=tk.W, pady=5)
        va_slider = ttk.Scale(param_frame, from_=0, to=440, orient=tk.HORIZONTAL,
                             variable=self.Va, length=200)
        va_slider.grid(row=0, column=1, pady=5)
        self.va_label = ttk.Label(param_frame, text=f"{self.Va.get():.1f} V")
        self.va_label.grid(row=0, column=2, padx=5)
        va_slider.configure(command=lambda v: self.va_label.config(text=f"{float(v):.1f} V"))

        # Field Voltage
        ttk.Label(param_frame, text="Field Voltage (V):").grid(row=1, column=0, sticky=tk.W, pady=5)
        vf_slider = ttk.Scale(param_frame, from_=0, to=440, orient=tk.HORIZONTAL,
                             variable=self.Vf, length=200)
        vf_slider.grid(row=1, column=1, pady=5)
        self.vf_label = ttk.Label(param_frame, text=f"{self.Vf.get():.1f} V")
        self.vf_label.grid(row=1, column=2, padx=5)
        vf_slider.configure(command=lambda v: self.vf_label.config(text=f"{float(v):.1f} V"))

        # Load Torque
        ttk.Label(param_frame, text="Load Torque (N·m):").grid(row=2, column=0, sticky=tk.W, pady=5)
        tl_slider = ttk.Scale(param_frame, from_=0, to=100, orient=tk.HORIZONTAL,
                             variable=self.Tload, length=200)
        tl_slider.grid(row=2, column=1, pady=5)
        self.tl_label = ttk.Label(param_frame, text=f"{self.Tload.get():.1f} N·m")
        self.tl_label.grid(row=2, column=2, padx=5)
        tl_slider.configure(command=lambda v: self.tl_label.config(text=f"{float(v):.1f} N·m"))

        # Simulation Method
        ttk.Label(param_frame, text="Simulation Method:").grid(row=3, column=0, sticky=tk.W, pady=5)
        method_combo = ttk.Combobox(param_frame, textvariable=self.simulation_method,
                                   values=["RK45", "Euler", "RK4"], state="readonly", width=15)
        method_combo.grid(row=3, column=1, pady=5, sticky=tk.W)

        # Control Buttons
        button_frame = ttk.Frame(left_frame)
        button_frame.pack(pady=20)

        self.start_button = ttk.Button(button_frame, text="Start", command=self.start_simulation,
                                       width=12, style='Success.TButton')
        self.start_button.grid(row=0, column=0, padx=5)

        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_simulation,
                                      width=12, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=1, padx=5)

        self.reset_button = ttk.Button(button_frame, text="Reset", command=self.reset_simulation,
                                       width=12)
        self.reset_button.grid(row=0, column=2, padx=5)

        # Status Display
        status_frame = ttk.LabelFrame(left_frame, text="Real-time Status", padding=10)
        status_frame.pack(fill=tk.X, pady=10)

        self.status_labels = {}
        status_items = [
            ('Time', 's'), ('Current', 'A'), ('Speed', 'rad/s'),
            ('Torque', 'N·m'), ('Power', 'W'), ('Temp', '°C')
        ]

        for i, (name, unit) in enumerate(status_items):
            ttk.Label(status_frame, text=f"{name}:").grid(row=i, column=0, sticky=tk.W)
            label = ttk.Label(status_frame, text=f"0.00 {unit}", font=('Courier', 10))
            label.grid(row=i, column=1, sticky=tk.E, padx=10)
            self.status_labels[name] = label

        # Right panel - Visualization
        right_frame = ttk.Frame(self.tab_main)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Create matplotlib figure
        self.main_fig = Figure(figsize=(8, 6), dpi=100)
        self.main_canvas = FigureCanvasTkAgg(self.main_fig, right_frame)
        self.main_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Create subplots
        self.ax_speed = self.main_fig.add_subplot(221)
        self.ax_current = self.main_fig.add_subplot(222)
        self.ax_torque = self.main_fig.add_subplot(223)
        self.ax_power = self.main_fig.add_subplot(224)

        self.ax_speed.set_title('Speed vs Time')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rad/s)')
        self.ax_speed.grid(True)

        self.ax_current.set_title('Current vs Time')
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.grid(True)

        self.ax_torque.set_title('Torque vs Time')
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.grid(True)

        self.ax_power.set_title('Power vs Time')
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (W)')
        self.ax_power.grid(True)

        self.main_fig.tight_layout()

    def setup_dynamics_tab(self):
        """Dynamic simulation with ODE solvers"""

        # Create matplotlib figure for dynamics
        self.dynamics_fig = Figure(figsize=(10, 8), dpi=100)
        self.dynamics_canvas = FigureCanvasTkAgg(self.dynamics_fig, self.tab_dynamics)
        self.dynamics_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Create subplots for detailed dynamics
        self.ax_dyn1 = self.dynamics_fig.add_subplot(321)
        self.ax_dyn2 = self.dynamics_fig.add_subplot(322)
        self.ax_dyn3 = self.dynamics_fig.add_subplot(323)
        self.ax_dyn4 = self.dynamics_fig.add_subplot(324)
        self.ax_dyn5 = self.dynamics_fig.add_subplot(325)
        self.ax_dyn6 = self.dynamics_fig.add_subplot(326)

        self.ax_dyn1.set_title('Armature Current')
        self.ax_dyn1.set_ylabel('Ia (A)')
        self.ax_dyn1.grid(True)

        self.ax_dyn2.set_title('Field Current')
        self.ax_dyn2.set_ylabel('If (A)')
        self.ax_dyn2.grid(True)

        self.ax_dyn3.set_title('Angular Velocity')
        self.ax_dyn3.set_ylabel('ω (rad/s)')
        self.ax_dyn3.grid(True)

        self.ax_dyn4.set_title('Position')
        self.ax_dyn4.set_ylabel('θ (rad)')
        self.ax_dyn4.grid(True)

        self.ax_dyn5.set_title('Temperature')
        self.ax_dyn5.set_ylabel('T (°C)')
        self.ax_dyn5.set_xlabel('Time (s)')
        self.ax_dyn5.grid(True)

        self.ax_dyn6.set_title('Phase Portrait (Speed vs Current)')
        self.ax_dyn6.set_xlabel('Speed (rad/s)')
        self.ax_dyn6.set_ylabel('Current (A)')
        self.ax_dyn6.grid(True)

        self.dynamics_fig.tight_layout()

    def setup_thermal_tab(self):
        """Thermal analysis and derating"""

        frame = ttk.Frame(self.tab_thermal)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left panel - Thermal parameters
        left_panel = ttk.LabelFrame(frame, text="Thermal Parameters", padding=10)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=5)

        ttk.Label(left_panel, text="Thermal Model Settings",
                 font=('Arial', 11, 'bold')).pack(pady=10)

        thermal_params = [
            ("Thermal Resistance (°C/W):", self.motor.thermal_resistance),
            ("Thermal Capacitance (J/°C):", self.motor.thermal_capacitance),
            ("Ambient Temperature (°C):", self.motor.ambient_temp),
            ("Max Operating Temp (°C):", 120),
            ("Derating Start Temp (°C):", 100)
        ]

        for label, value in thermal_params:
            frame_row = ttk.Frame(left_panel)
            frame_row.pack(fill=tk.X, pady=5)
            ttk.Label(frame_row, text=label, width=25).pack(side=tk.LEFT)
            ttk.Label(frame_row, text=f"{value:.2f}", font=('Courier', 10)).pack(side=tk.RIGHT)

        # Derating curve info
        info_frame = ttk.LabelFrame(left_panel, text="Derating Information", padding=10)
        info_frame.pack(fill=tk.X, pady=20)

        info_text = """
Motor derating curve:
- Full power: T < 100°C
- Linear derating: 100-120°C
- Shutdown: T > 120°C

Thermal time constant:
τ = R_th × C_th = {:.2f} s
        """.format(self.motor.thermal_resistance * self.motor.thermal_capacitance)

        ttk.Label(info_frame, text=info_text, justify=tk.LEFT).pack()

        # Right panel - Thermal visualization
        right_panel = ttk.Frame(frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        self.thermal_fig = Figure(figsize=(8, 6), dpi=100)
        self.thermal_canvas = FigureCanvasTkAgg(self.thermal_fig, right_panel)
        self.thermal_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ax_thermal1 = self.thermal_fig.add_subplot(211)
        self.ax_thermal2 = self.thermal_fig.add_subplot(212)

        self.ax_thermal1.set_title('Temperature Rise')
        self.ax_thermal1.set_ylabel('Temperature (°C)')
        self.ax_thermal1.grid(True)
        self.ax_thermal1.axhline(y=100, color='orange', linestyle='--', label='Derating Start')
        self.ax_thermal1.axhline(y=120, color='red', linestyle='--', label='Max Temp')
        self.ax_thermal1.legend()

        self.ax_thermal2.set_title('Power Derating Factor')
        self.ax_thermal2.set_xlabel('Time (s)')
        self.ax_thermal2.set_ylabel('Derating Factor')
        self.ax_thermal2.grid(True)
        self.ax_thermal2.set_ylim([0, 1.1])

        self.thermal_fig.tight_layout()

    def setup_losses_tab(self):
        """Detailed loss analysis"""

        frame = ttk.Frame(self.tab_losses)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Create matplotlib figure for losses
        self.losses_fig = Figure(figsize=(10, 8), dpi=100)
        self.losses_canvas = FigureCanvasTkAgg(self.losses_fig, frame)
        self.losses_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Create subplots
        self.ax_loss1 = self.losses_fig.add_subplot(221)
        self.ax_loss2 = self.losses_fig.add_subplot(222)
        self.ax_loss3 = self.losses_fig.add_subplot(223)
        self.ax_loss4 = self.losses_fig.add_subplot(224)

        self.ax_loss1.set_title('Loss Breakdown (Current State)')
        self.ax_loss1.set_ylabel('Power Loss (W)')

        self.ax_loss2.set_title('Efficiency vs Time')
        self.ax_loss2.set_ylabel('Efficiency (%)')
        self.ax_loss2.grid(True)

        self.ax_loss3.set_title('Total Losses vs Time')
        self.ax_loss3.set_xlabel('Time (s)')
        self.ax_loss3.set_ylabel('Total Loss (W)')
        self.ax_loss3.grid(True)

        self.ax_loss4.set_title('Loss Distribution (Pie Chart)')

        self.losses_fig.tight_layout()

    def setup_mechanical_tab(self):
        """Mechanical stress and vibration analysis"""

        frame = ttk.Frame(self.tab_mechanical)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left panel - Mechanical parameters
        left_panel = ttk.LabelFrame(frame, text="Mechanical Parameters", padding=10)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=5)

        ttk.Label(left_panel, text="Shaft & Bearing Analysis",
                 font=('Arial', 11, 'bold')).pack(pady=10)

        mech_params = [
            ("Shaft Diameter (mm):", self.motor.shaft_diameter * 1000),
            ("Shaft Length (mm):", self.motor.shaft_length * 1000),
            ("Rotor Mass (kg):", self.motor.rotor_mass),
            ("Material: Steel", ""),
            ("Yield Strength (MPa):", self.motor.shaft_yield_strength / 1e6),
            ("", ""),
            ("Bearing Type: Ball Bearing", ""),
            ("Dynamic Load Rating (N):", self.motor.bearing_dynamic_load),
            ("Rated Life (hours):", self.motor.bearing_life_rating)
        ]

        for label, value in mech_params:
            frame_row = ttk.Frame(left_panel)
            frame_row.pack(fill=tk.X, pady=3)
            ttk.Label(frame_row, text=label, width=25).pack(side=tk.LEFT)
            if value:
                if isinstance(value, str):
                    ttk.Label(frame_row, text=value, font=('Courier', 9)).pack(side=tk.RIGHT)
                else:
                    ttk.Label(frame_row, text=f"{value:.2f}", font=('Courier', 9)).pack(side=tk.RIGHT)

        # Real-time mechanical status
        status_frame = ttk.LabelFrame(left_panel, text="Current Mechanical Status", padding=10)
        status_frame.pack(fill=tk.X, pady=20)

        self.mech_status_labels = {}
        mech_items = [
            ('Shaft Stress', 'MPa'),
            ('Safety Factor', ''),
            ('Bearing Load', 'N'),
            ('Bearing Life', 'hrs'),
            ('Vibration', 'μm'),
            ('Twist Angle', 'deg')
        ]

        for i, (name, unit) in enumerate(mech_items):
            ttk.Label(status_frame, text=f"{name}:").grid(row=i, column=0, sticky=tk.W, pady=2)
            label = ttk.Label(status_frame, text=f"0.00 {unit}", font=('Courier', 9))
            label.grid(row=i, column=1, sticky=tk.E, padx=10, pady=2)
            self.mech_status_labels[name] = label

        # Right panel - Mechanical visualization
        right_panel = ttk.Frame(frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        self.mechanical_fig = Figure(figsize=(10, 8), dpi=100)
        self.mechanical_canvas = FigureCanvasTkAgg(self.mechanical_fig, right_panel)
        self.mechanical_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Create subplots for mechanical analysis
        self.ax_mech1 = self.mechanical_fig.add_subplot(321)
        self.ax_mech2 = self.mechanical_fig.add_subplot(322)
        self.ax_mech3 = self.mechanical_fig.add_subplot(323)
        self.ax_mech4 = self.mechanical_fig.add_subplot(324)
        self.ax_mech5 = self.mechanical_fig.add_subplot(325)
        self.ax_mech6 = self.mechanical_fig.add_subplot(326)

        self.ax_mech1.set_title('Shaft Torsional Stress')
        self.ax_mech1.set_ylabel('Stress (MPa)')
        self.ax_mech1.grid(True)
        # Add safety limit line
        self.ax_mech1.axhline(y=self.motor.shaft_yield_strength/1e6, color='r',
                             linestyle='--', label='Yield Strength')
        self.ax_mech1.legend()

        self.ax_mech2.set_title('Safety Factor')
        self.ax_mech2.set_ylabel('Safety Factor')
        self.ax_mech2.grid(True)
        self.ax_mech2.axhline(y=2.0, color='orange', linestyle='--', label='Min Recommended')
        self.ax_mech2.legend()

        self.ax_mech3.set_title('Bearing Radial Load')
        self.ax_mech3.set_ylabel('Load (N)')
        self.ax_mech3.grid(True)

        self.ax_mech4.set_title('Bearing Life Expectancy')
        self.ax_mech4.set_ylabel('Life (hours)')
        self.ax_mech4.grid(True)
        self.ax_mech4.set_yscale('log')

        self.ax_mech5.set_title('Vibration Amplitude')
        self.ax_mech5.set_xlabel('Time (s)')
        self.ax_mech5.set_ylabel('Amplitude (μm)')
        self.ax_mech5.grid(True)

        self.ax_mech6.set_title('Angular Acceleration')
        self.ax_mech6.set_xlabel('Time (s)')
        self.ax_mech6.set_ylabel('Accel (rad/s²)')
        self.ax_mech6.grid(True)

        self.mechanical_fig.tight_layout()

    def setup_economic_tab(self):
        """Economic analysis tab"""

        frame = ttk.Frame(self.tab_economic)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left panel - Economic parameters
        left_panel = ttk.LabelFrame(frame, text="Economic Parameters", padding=10)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=5)

        ttk.Label(left_panel, text="Cost Analysis Settings",
                 font=('Arial', 11, 'bold')).pack(pady=10)

        # Energy cost
        ttk.Label(left_panel, text="Energy Cost ($/kWh):").pack(anchor=tk.W, pady=5)
        energy_entry = ttk.Entry(left_panel, textvariable=self.energy_cost, width=20)
        energy_entry.pack(pady=5)

        # Maintenance cost
        ttk.Label(left_panel, text="Maintenance Cost ($/year):").pack(anchor=tk.W, pady=5)
        maint_entry = ttk.Entry(left_panel, textvariable=self.maintenance_cost, width=20)
        maint_entry.pack(pady=5)

        # Operating hours
        self.operating_hours = tk.DoubleVar(value=4000)
        ttk.Label(left_panel, text="Operating Hours (h/year):").pack(anchor=tk.W, pady=5)
        hours_entry = ttk.Entry(left_panel, textvariable=self.operating_hours, width=20)
        hours_entry.pack(pady=5)

        # Motor cost
        self.motor_cost = tk.DoubleVar(value=5000)
        ttk.Label(left_panel, text="Motor Initial Cost ($):").pack(anchor=tk.W, pady=5)
        cost_entry = ttk.Entry(left_panel, textvariable=self.motor_cost, width=20)
        cost_entry.pack(pady=5)

        # Calculate button
        ttk.Button(left_panel, text="Calculate Economics",
                  command=self.calculate_economics).pack(pady=20)

        # Results display
        self.economic_results = tk.Text(left_panel, height=15, width=35)
        self.economic_results.pack(pady=10)

        # Right panel - Economic visualization
        right_panel = ttk.Frame(frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)

        self.economic_fig = Figure(figsize=(8, 6), dpi=100)
        self.economic_canvas = FigureCanvasTkAgg(self.economic_fig, right_panel)
        self.economic_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ax_econ1 = self.economic_fig.add_subplot(211)
        self.ax_econ2 = self.economic_fig.add_subplot(212)

        self.ax_econ1.set_title('Cost Breakdown')
        self.ax_econ2.set_title('Cumulative Cost Over Time')
        self.ax_econ2.set_xlabel('Years')
        self.ax_econ2.set_ylabel('Cumulative Cost ($)')
        self.ax_econ2.grid(True)

        self.economic_fig.tight_layout()

    def setup_controls_tab(self):
        """Advanced control methods"""

        frame = ttk.Frame(self.tab_controls)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Control method selection
        control_frame = ttk.LabelFrame(frame, text="Control Methods", padding=10)
        control_frame.pack(fill=tk.X, padx=5, pady=5)

        self.control_method = tk.StringVar(value="Open Loop")

        control_methods = [
            "Open Loop",
            "Armature Voltage Control",
            "Field Flux Control",
            "Armature Resistance Control",
            "PID Speed Control",
            "Vector Control",
            "Field-Oriented Control (FOC)"
        ]

        ttk.Label(control_frame, text="Select Control Method:",
                 font=('Arial', 10, 'bold')).grid(row=0, column=0, pady=10)

        for i, method in enumerate(control_methods):
            ttk.Radiobutton(control_frame, text=method, variable=self.control_method,
                           value=method).grid(row=i+1, column=0, sticky=tk.W, padx=20)

        # PID parameters
        pid_frame = ttk.LabelFrame(frame, text="PID Controller Parameters", padding=10)
        pid_frame.pack(fill=tk.X, padx=5, pady=5)

        self.Kp = tk.DoubleVar(value=10.0)
        self.Ki = tk.DoubleVar(value=1.0)
        self.Kd = tk.DoubleVar(value=0.5)
        self.setpoint = tk.DoubleVar(value=100.0)

        ttk.Label(pid_frame, text="Kp (Proportional):").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(pid_frame, textvariable=self.Kp, width=15).grid(row=0, column=1, padx=5)

        ttk.Label(pid_frame, text="Ki (Integral):").grid(row=1, column=0, sticky=tk.W)
        ttk.Entry(pid_frame, textvariable=self.Ki, width=15).grid(row=1, column=1, padx=5)

        ttk.Label(pid_frame, text="Kd (Derivative):").grid(row=2, column=0, sticky=tk.W)
        ttk.Entry(pid_frame, textvariable=self.Kd, width=15).grid(row=2, column=1, padx=5)

        ttk.Label(pid_frame, text="Speed Setpoint (rad/s):").grid(row=3, column=0, sticky=tk.W)
        ttk.Entry(pid_frame, textvariable=self.setpoint, width=15).grid(row=3, column=1, padx=5)

        # Control visualization
        viz_frame = ttk.Frame(frame)
        viz_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.control_fig = Figure(figsize=(10, 6), dpi=100)
        self.control_canvas = FigureCanvasTkAgg(self.control_fig, viz_frame)
        self.control_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.ax_ctrl1 = self.control_fig.add_subplot(211)
        self.ax_ctrl2 = self.control_fig.add_subplot(212)

        self.ax_ctrl1.set_title('Control Signal vs Time')
        self.ax_ctrl1.set_ylabel('Control Signal')
        self.ax_ctrl1.grid(True)

        self.ax_ctrl2.set_title('Tracking Error vs Time')
        self.ax_ctrl2.set_xlabel('Time (s)')
        self.ax_ctrl2.set_ylabel('Error (rad/s)')
        self.ax_ctrl2.grid(True)

        self.control_fig.tight_layout()

    def start_simulation(self):
        """Start the simulation"""
        self.is_running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.run_simulation_step()

    def stop_simulation(self):
        """Stop the simulation"""
        self.is_running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

    def reset_simulation(self):
        """Reset the simulation"""
        self.stop_simulation()
        self.simulation_time = 0
        self.state = [0, 0, 0, 0, 25]
        self.time_history = []
        for key in self.data_history:
            self.data_history[key] = []
        self.update_plots()

    def calculate_pid_control(self, setpoint, current_value, dt):
        """Calculate PID control output"""
        error = setpoint - current_value

        # Proportional term
        P = self.Kp.get() * error

        # Integral term with anti-windup
        self.integral_error += error * dt
        # Anti-windup: limit integral term
        max_integral = 100
        self.integral_error = max(min(self.integral_error, max_integral), -max_integral)
        I = self.Ki.get() * self.integral_error

        # Derivative term
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        D = self.Kd.get() * derivative

        # Update previous error
        self.previous_error = error

        # Total control output
        control_output = P + I + D

        # Limit control output to valid voltage range
        control_output = max(min(control_output, 440), 0)

        return control_output, error

    def run_simulation_step(self):
        """Execute one simulation step"""
        if not self.is_running:
            return

        dt = 0.01  # Time step (10ms)

        # Get control inputs
        Va = self.Va.get()
        Vf = self.Vf.get()
        Tload = self.Tload.get()

        # Apply control method if PID is selected
        if self.control_method.get() == "PID Speed Control":
            setpoint = self.setpoint.get()
            current_speed = self.state[2]  # omega
            Va, error = self.calculate_pid_control(setpoint, current_speed, dt)
            self.control_output_history.append(Va)
            self.error_history.append(error)

        # Solve differential equations
        if self.simulation_method.get() == "Euler":
            # Euler method
            derivatives = self.motor.get_differential_equations(
                self.simulation_time, self.state, Va, Vf, Tload
            )
            self.state = [s + d * dt for s, d in zip(self.state, derivatives)]
        elif self.simulation_method.get() == "RK4":
            # RK4 method
            self.state = self.rk4_step(self.state, self.simulation_time, dt, Va, Vf, Tload)
        else:  # RK45
            # Use scipy's RK45
            sol = solve_ivp(
                lambda t, y: self.motor.get_differential_equations(t, y, Va, Vf, Tload),
                [self.simulation_time, self.simulation_time + dt],
                self.state,
                method='RK45',
                dense_output=True
            )
            self.state = sol.y[:, -1].tolist()

        # Extract state variables
        ia, if_current, omega, theta, temp = self.state

        # Calculate derived quantities
        eb = self.motor.Kb * if_current * omega
        Te = self.motor.Kt * if_current * ia
        Pin = Va * ia + Vf * if_current
        Pout = Te * omega
        losses = self.motor.get_losses(ia, if_current, omega)
        efficiency = (Pout / Pin * 100) if Pin > 0.1 else 0

        # Calculate angular acceleration
        acceleration = (omega - self.previous_omega) / dt if dt > 0 else 0
        self.previous_omega = omega

        # Calculate mechanical stress and bearing loads
        stress_data = self.motor.calculate_shaft_stress(Te, omega, acceleration)
        bearing_data = self.motor.calculate_bearing_loads(Te, omega, acceleration)

        # Calculate vibration (torque ripple simplified as 5% of torque)
        torque_ripple = 0.05 * abs(Te)
        vibration_data = self.motor.calculate_vibration(omega, torque_ripple)

        # Store data
        self.time_history.append(self.simulation_time)
        self.data_history['current'].append(ia)
        self.data_history['voltage'].append(Va)
        self.data_history['speed'].append(omega)
        self.data_history['torque'].append(Te)
        self.data_history['power'].append(Pout)
        self.data_history['efficiency'].append(efficiency)
        self.data_history['temperature'].append(temp)
        self.data_history['losses'].append(losses['total'])
        self.data_history['shaft_stress'].append(stress_data['torsional_stress'])
        self.data_history['bearing_load'].append(bearing_data['radial_load'])
        self.data_history['vibration'].append(vibration_data['amplitude'])
        self.data_history['acceleration'].append(acceleration)
        self.data_history['safety_factor'].append(min(stress_data['safety_factor'], 50))

        # Update status labels
        self.status_labels['Time'].config(text=f"{self.simulation_time:.2f} s")
        self.status_labels['Current'].config(text=f"{ia:.2f} A")
        self.status_labels['Speed'].config(text=f"{omega:.2f} rad/s")
        self.status_labels['Torque'].config(text=f"{Te:.2f} N·m")
        self.status_labels['Power'].config(text=f"{Pout:.2f} W")
        self.status_labels['Temp'].config(text=f"{temp:.2f} °C")

        # Update mechanical status labels
        self.mech_status_labels['Shaft Stress'].config(
            text=f"{stress_data['torsional_stress']/1e6:.2f} MPa")
        self.mech_status_labels['Safety Factor'].config(
            text=f"{min(stress_data['safety_factor'], 999):.2f}")
        self.mech_status_labels['Bearing Load'].config(
            text=f"{bearing_data['radial_load']:.2f} N")
        self.mech_status_labels['Bearing Life'].config(
            text=f"{min(bearing_data['bearing_life_hours'], 999999):.0f} hrs")
        self.mech_status_labels['Vibration'].config(
            text=f"{vibration_data['amplitude']*1000:.2f} μm")
        self.mech_status_labels['Twist Angle'].config(
            text=f"{math.degrees(stress_data['shaft_twist']):.4f} deg")

        # Update plots every 50ms
        if len(self.time_history) % 5 == 0:
            self.update_plots()

        # Increment time
        self.simulation_time += dt

        # Schedule next step
        self.root.after(10, self.run_simulation_step)

    def rk4_step(self, state, t, dt, Va, Vf, Tload):
        """Runge-Kutta 4th order integration step"""
        k1 = self.motor.get_differential_equations(t, state, Va, Vf, Tload)

        state_k2 = [s + 0.5 * dt * k for s, k in zip(state, k1)]
        k2 = self.motor.get_differential_equations(t + 0.5*dt, state_k2, Va, Vf, Tload)

        state_k3 = [s + 0.5 * dt * k for s, k in zip(state, k2)]
        k3 = self.motor.get_differential_equations(t + 0.5*dt, state_k3, Va, Vf, Tload)

        state_k4 = [s + dt * k for s, k in zip(state, k3)]
        k4 = self.motor.get_differential_equations(t + dt, state_k4, Va, Vf, Tload)

        new_state = [s + (dt/6) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
                    for i, s in enumerate(state)]

        return new_state

    def update_plots(self):
        """Update all plots with current data"""
        if not self.time_history:
            return

        t = self.time_history

        # Main tab plots
        self.ax_speed.clear()
        self.ax_speed.plot(t, self.data_history['speed'], 'b-', linewidth=2)
        self.ax_speed.set_title('Speed vs Time')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rad/s)')
        self.ax_speed.grid(True)

        self.ax_current.clear()
        self.ax_current.plot(t, self.data_history['current'], 'r-', linewidth=2)
        self.ax_current.set_title('Armature Current vs Time')
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.grid(True)

        self.ax_torque.clear()
        self.ax_torque.plot(t, self.data_history['torque'], 'g-', linewidth=2)
        self.ax_torque.set_title('Torque vs Time')
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.grid(True)

        self.ax_power.clear()
        self.ax_power.plot(t, self.data_history['power'], 'm-', linewidth=2)
        self.ax_power.set_title('Output Power vs Time')
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (W)')
        self.ax_power.grid(True)

        self.main_fig.tight_layout()
        self.main_canvas.draw()

        # Update dynamics tab if it has data
        if len(self.time_history) > 1:
            self.update_dynamics_plots()
            self.update_thermal_plots()
            self.update_losses_plots()
            self.update_mechanical_plots()
            self.update_control_plots()

    def update_dynamics_plots(self):
        """Update dynamics tab plots"""
        t = self.time_history
        ia, if_current, omega, theta, temp = zip(*[
            [self.data_history['current'][i],
             self.state[1],  # Simplified - would need full history
             self.data_history['speed'][i],
             0,  # Would need position history
             self.data_history['temperature'][i]]
            for i in range(len(t))
        ])

        self.ax_dyn1.clear()
        self.ax_dyn1.plot(t, ia, 'b-', linewidth=2)
        self.ax_dyn1.set_title('Armature Current')
        self.ax_dyn1.set_ylabel('Ia (A)')
        self.ax_dyn1.grid(True)

        self.ax_dyn3.clear()
        self.ax_dyn3.plot(t, omega, 'g-', linewidth=2)
        self.ax_dyn3.set_title('Angular Velocity')
        self.ax_dyn3.set_ylabel('ω (rad/s)')
        self.ax_dyn3.grid(True)

        self.ax_dyn5.clear()
        self.ax_dyn5.plot(t, temp, 'r-', linewidth=2)
        self.ax_dyn5.set_title('Temperature')
        self.ax_dyn5.set_ylabel('T (°C)')
        self.ax_dyn5.set_xlabel('Time (s)')
        self.ax_dyn5.grid(True)

        self.ax_dyn6.clear()
        self.ax_dyn6.plot(omega, ia, 'b-', linewidth=2)
        self.ax_dyn6.set_title('Phase Portrait')
        self.ax_dyn6.set_xlabel('Speed (rad/s)')
        self.ax_dyn6.set_ylabel('Current (A)')
        self.ax_dyn6.grid(True)

        self.dynamics_fig.tight_layout()
        self.dynamics_canvas.draw()

    def update_thermal_plots(self):
        """Update thermal analysis plots"""
        t = self.time_history
        temp = self.data_history['temperature']

        # Calculate derating factor
        derating = []
        for T in temp:
            if T < 100:
                derating.append(1.0)
            elif T < 120:
                derating.append(1.0 - (T - 100) / 20)
            else:
                derating.append(0.0)

        self.ax_thermal1.clear()
        self.ax_thermal1.plot(t, temp, 'r-', linewidth=2, label='Temperature')
        self.ax_thermal1.axhline(y=100, color='orange', linestyle='--', label='Derating Start')
        self.ax_thermal1.axhline(y=120, color='red', linestyle='--', label='Max Temp')
        self.ax_thermal1.set_title('Temperature Rise')
        self.ax_thermal1.set_ylabel('Temperature (°C)')
        self.ax_thermal1.grid(True)
        self.ax_thermal1.legend()

        self.ax_thermal2.clear()
        self.ax_thermal2.plot(t, derating, 'b-', linewidth=2)
        self.ax_thermal2.set_title('Power Derating Factor')
        self.ax_thermal2.set_xlabel('Time (s)')
        self.ax_thermal2.set_ylabel('Derating Factor')
        self.ax_thermal2.grid(True)
        self.ax_thermal2.set_ylim([0, 1.1])

        self.thermal_fig.tight_layout()
        self.thermal_canvas.draw()

    def update_losses_plots(self):
        """Update loss analysis plots"""
        t = self.time_history

        # Get current losses
        if len(self.time_history) > 0:
            ia = self.data_history['current'][-1]
            omega = self.data_history['speed'][-1]
            if_current = 2.2  # Simplified
            current_losses = self.motor.get_losses(ia, if_current, omega)

            # Bar chart of loss breakdown
            self.ax_loss1.clear()
            loss_types = ['Copper\nArmature', 'Copper\nField', 'Iron', 'Mechanical', 'Stray']
            loss_values = [
                current_losses['copper_armature'],
                current_losses['copper_field'],
                current_losses['iron'],
                current_losses['mechanical'],
                current_losses['stray']
            ]
            colors = ['#ff6b6b', '#ff8787', '#ffa94d', '#ffd43b', '#74c0fc']
            self.ax_loss1.bar(loss_types, loss_values, color=colors)
            self.ax_loss1.set_title('Loss Breakdown (Current State)')
            self.ax_loss1.set_ylabel('Power Loss (W)')
            self.ax_loss1.tick_params(axis='x', labelsize=8)

            # Efficiency vs time
            self.ax_loss2.clear()
            self.ax_loss2.plot(t, self.data_history['efficiency'], 'g-', linewidth=2)
            self.ax_loss2.set_title('Efficiency vs Time')
            self.ax_loss2.set_ylabel('Efficiency (%)')
            self.ax_loss2.grid(True)
            self.ax_loss2.set_ylim([0, 105])

            # Total losses vs time
            self.ax_loss3.clear()
            self.ax_loss3.plot(t, self.data_history['losses'], 'r-', linewidth=2)
            self.ax_loss3.set_title('Total Losses vs Time')
            self.ax_loss3.set_xlabel('Time (s)')
            self.ax_loss3.set_ylabel('Total Loss (W)')
            self.ax_loss3.grid(True)

            # Pie chart of loss distribution
            self.ax_loss4.clear()
            if sum(loss_values) > 0:
                self.ax_loss4.pie(loss_values, labels=loss_types, autopct='%1.1f%%',
                                 colors=colors, startangle=90)
                self.ax_loss4.set_title('Loss Distribution')

            self.losses_fig.tight_layout()
            self.losses_canvas.draw()

    def update_mechanical_plots(self):
        """Update mechanical stress and vibration plots"""
        t = self.time_history

        if not t:
            return

        # Shaft stress
        self.ax_mech1.clear()
        stress_mpa = [s/1e6 for s in self.data_history['shaft_stress']]
        self.ax_mech1.plot(t, stress_mpa, 'b-', linewidth=2)
        self.ax_mech1.axhline(y=self.motor.shaft_yield_strength/1e6, color='r',
                             linestyle='--', label='Yield Strength')
        self.ax_mech1.set_title('Shaft Torsional Stress')
        self.ax_mech1.set_ylabel('Stress (MPa)')
        self.ax_mech1.grid(True)
        self.ax_mech1.legend()

        # Safety factor
        self.ax_mech2.clear()
        self.ax_mech2.plot(t, self.data_history['safety_factor'], 'g-', linewidth=2)
        self.ax_mech2.axhline(y=2.0, color='orange', linestyle='--', label='Min Recommended')
        self.ax_mech2.set_title('Safety Factor')
        self.ax_mech2.set_ylabel('Safety Factor')
        self.ax_mech2.grid(True)
        self.ax_mech2.legend()
        self.ax_mech2.set_ylim([0, 20])

        # Bearing load
        self.ax_mech3.clear()
        self.ax_mech3.plot(t, self.data_history['bearing_load'], 'r-', linewidth=2)
        self.ax_mech3.set_title('Bearing Radial Load')
        self.ax_mech3.set_ylabel('Load (N)')
        self.ax_mech3.grid(True)

        # Bearing life - calculate from current data
        bearing_life_data = []
        for i in range(len(t)):
            omega = self.data_history['speed'][i]
            bearing_load = self.data_history['bearing_load'][i]
            if omega > 0.1 and bearing_load > 0:
                rpm = omega * 60 / (2 * math.pi)
                life = ((self.motor.bearing_dynamic_load / bearing_load) ** 3 *
                       10**6 / (60 * rpm))
                bearing_life_data.append(min(life, 1e6))
            else:
                bearing_life_data.append(1e6)

        self.ax_mech4.clear()
        self.ax_mech4.plot(t, bearing_life_data, 'm-', linewidth=2)
        self.ax_mech4.set_title('Bearing Life Expectancy')
        self.ax_mech4.set_ylabel('Life (hours)')
        self.ax_mech4.grid(True)
        self.ax_mech4.set_yscale('log')

        # Vibration
        self.ax_mech5.clear()
        vibration_um = [v*1000 for v in self.data_history['vibration']]
        self.ax_mech5.plot(t, vibration_um, 'c-', linewidth=2)
        self.ax_mech5.set_title('Vibration Amplitude')
        self.ax_mech5.set_xlabel('Time (s)')
        self.ax_mech5.set_ylabel('Amplitude (μm)')
        self.ax_mech5.grid(True)

        # Angular acceleration
        self.ax_mech6.clear()
        self.ax_mech6.plot(t, self.data_history['acceleration'], 'k-', linewidth=2)
        self.ax_mech6.set_title('Angular Acceleration')
        self.ax_mech6.set_xlabel('Time (s)')
        self.ax_mech6.set_ylabel('Accel (rad/s²)')
        self.ax_mech6.grid(True)

        self.mechanical_fig.tight_layout()
        self.mechanical_canvas.draw()

    def calculate_economics(self):
        """Calculate economic analysis"""
        # Get parameters
        energy_cost = self.energy_cost.get()
        maint_cost = self.maintenance_cost.get()
        op_hours = self.operating_hours.get()
        motor_cost = self.motor_cost.get()

        # Calculate average power from simulation
        if len(self.data_history['power']) > 0:
            avg_power = np.mean(self.data_history['power'])
        else:
            avg_power = 5000  # Default 5kW

        # Annual energy consumption (kWh)
        annual_energy = avg_power / 1000 * op_hours

        # Annual energy cost
        annual_energy_cost = annual_energy * energy_cost

        # Total annual cost
        total_annual_cost = annual_energy_cost + maint_cost

        # Calculate payback and lifecycle costs (10 years)
        years = 10
        lifecycle_cost = motor_cost + total_annual_cost * years

        # Display results
        results_text = f"""
ECONOMIC ANALYSIS RESULTS
{'='*40}

Initial Investment:      ${motor_cost:,.2f}

Annual Operating Costs:
  Energy Cost:           ${annual_energy_cost:,.2f}
  Maintenance Cost:      ${maint_cost:,.2f}
  Total Annual Cost:     ${total_annual_cost:,.2f}

Energy Consumption:
  Average Power:         {avg_power:.2f} W
  Annual Energy:         {annual_energy:,.2f} kWh

{years}-Year Lifecycle:
  Total Energy Cost:     ${annual_energy_cost*years:,.2f}
  Total Maint. Cost:     ${maint_cost*years:,.2f}
  Total Lifecycle Cost:  ${lifecycle_cost:,.2f}

Cost per Operating Hour: ${total_annual_cost/op_hours:.4f}
        """

        self.economic_results.delete(1.0, tk.END)
        self.economic_results.insert(1.0, results_text)

        # Update plots
        self.ax_econ1.clear()
        costs = ['Initial\nCost', 'Annual\nEnergy', 'Annual\nMaintenance']
        values = [motor_cost, annual_energy_cost, maint_cost]
        colors = ['#4ecdc4', '#ff6b6b', '#ffe66d']
        self.ax_econ1.bar(costs, values, color=colors)
        self.ax_econ1.set_title('Cost Breakdown')
        self.ax_econ1.set_ylabel('Cost ($)')

        self.ax_econ2.clear()
        years_range = np.arange(0, years + 1)
        cumulative_cost = motor_cost + total_annual_cost * years_range
        self.ax_econ2.plot(years_range, cumulative_cost, 'b-o', linewidth=2)
        self.ax_econ2.set_title('Cumulative Cost Over Time')
        self.ax_econ2.set_xlabel('Years')
        self.ax_econ2.set_ylabel('Cumulative Cost ($)')
        self.ax_econ2.grid(True)

        self.economic_fig.tight_layout()
        self.economic_canvas.draw()

    def update_control_plots(self):
        """Update control system plots"""
        if not self.control_output_history or not self.error_history:
            return

        t = self.time_history[-len(self.control_output_history):]

        # Control signal
        self.ax_ctrl1.clear()
        self.ax_ctrl1.plot(t, self.control_output_history, 'b-', linewidth=2)
        self.ax_ctrl1.set_title('Control Signal (Armature Voltage)')
        self.ax_ctrl1.set_ylabel('Voltage (V)')
        self.ax_ctrl1.grid(True)

        # Tracking error
        self.ax_ctrl2.clear()
        self.ax_ctrl2.plot(t, self.error_history, 'r-', linewidth=2)
        self.ax_ctrl2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.ax_ctrl2.set_title('Speed Tracking Error')
        self.ax_ctrl2.set_xlabel('Time (s)')
        self.ax_ctrl2.set_ylabel('Error (rad/s)')
        self.ax_ctrl2.grid(True)

        self.control_fig.tight_layout()
        self.control_canvas.draw()

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        # This is called on any window event, only handle actual size changes
        if event.widget == self.root:
            # The canvas will automatically resize due to pack with fill and expand
            pass

# ===============================================================================
# MAIN APPLICATION ENTRY POINT
# ===============================================================================

def main():
    """Main entry point for the application"""

    # Print flywheel problem solution to console
    print("=" * 70)
    print("FLYWHEEL INERTIA PROBLEM SOLUTION")
    print("=" * 70)
    flywheel_result = calculate_flywheel_inertia()
    print(f"\nProblem Statement:")
    print(f"  - Load Torque: 150 kg-m for 15 seconds")
    print(f"  - Motor Torque Limited: 85 kg-m")
    print(f"  - No Load Speed: 500 rpm")
    print(f"  - Full Load Slip: 10%")
    print(f"\nSolution:")
    print(f"  - No Load Speed: {flywheel_result['N_no_load']:.0f} rpm ({flywheel_result['omega_no_load']:.2f} rad/s)")
    print(f"  - Full Load Speed: {flywheel_result['N_full_load']:.0f} rpm ({flywheel_result['omega_full_load']:.2f} rad/s)")
    print(f"  - Load Torque: {flywheel_result['T_load']:.2f} N·m")
    print(f"  - Motor Torque: {flywheel_result['T_motor']:.2f} N·m")
    print(f"  - Torque Deficit: {flywheel_result['T_deficit']:.2f} N·m")
    print(f"  - Energy Stored in Flywheel: {flywheel_result['energy_stored']:.2f} J")
    print(f"\n  *** FLYWHEEL MOMENT OF INERTIA: {flywheel_result['J']:.2f} kg·m² ***")
    print(f"  (Alternative method: {flywheel_result['J_alternative']:.2f} kg·m²)")
    print("\n" + "=" * 70)

    print("\n" + "=" * 70)
    print("MOTOR RATING PROBLEM SOLUTION")
    print("=" * 70)
    result = calculate_motor_rating()
    print(f"\nLoad Cycle:")
    print(f"  - Acceleration: 0 to 2000 hp in 20 sec")
    print(f"  - Full speed: 1000 hp for 40 sec")
    print(f"  - Deceleration: 330 to 0 hp in 10 sec (regenerative)")
    print(f"  - Rest: 0 hp for 20 sec")
    print(f"\nTotal cycle time: {result['total_time']} seconds")
    print(f"\n  *** RMS HORSEPOWER RATING: {result['rms_hp']:.2f} hp ***")
    print(f"  Equivalent to: {result['rms_hp'] * 0.746:.2f} kW")
    print("\n" + "=" * 70)

    # Create and run GUI application
    root = tk.Tk()

    # Configure style
    style = ttk.Style()
    style.theme_use('clam')

    # Create application
    app = AdvancedMotorAnalysisApp(root)

    # Run main loop
    root.mainloop()

if __name__ == "__main__":
    main()
