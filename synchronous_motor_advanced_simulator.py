"""
Advanced Three-Phase Synchronous Motor Simulator
Multi-Physics Simulation with Dynamic ODE Solvers
Features: Real-time simulation, parameter adjustment, visualization
"""

import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from scipy.integrate import ode, solve_ivp
import threading
import time

class SynchronousMotorModel:
    """Mathematical model for synchronous motor"""

    def __init__(self):
        # Default parameters from problem
        self.poles = 6
        self.P_rated = 160000  # W
        self.V_line = 346  # V
        self.freq = 180  # Hz
        self.R1 = 0.011  # Ohm
        self.Xsd = 0.335  # Ohm
        self.Xsq = 0.377  # Ohm
        self.delta = 26  # degrees
        self.Bmg = 0.65  # T
        self.N1 = 24  # turns per phase
        self.kw1 = 0.925  # winding factor
        self.Li = 0.19  # m
        self.D = 0.33  # m
        self.Prot = 1200  # W
        self.Pstr_factor = 0.05

        # Calculated parameters
        self.p = self.poles / 2  # pole pairs
        self.omega_e = 2 * np.pi * self.freq  # electrical rad/s
        self.omega_m = self.omega_e / self.p  # mechanical rad/s
        self.V_phase = self.V_line / np.sqrt(3)  # Phase voltage

        # Dynamic state variables
        self.id = 0  # d-axis current
        self.iq = 0  # q-axis current
        self.theta = 0  # rotor position
        self.omega = self.omega_m  # rotor speed

        self.calculate_steady_state()

    def calculate_steady_state(self):
        """Calculate steady-state operating point"""
        # Calculate flux linkage
        self.phi = self.Bmg * np.pi * self.D * self.Li / self.p

        # Back EMF
        self.Ef = 4.44 * self.freq * self.N1 * self.kw1 * self.phi

        # Convert power angle to radians
        delta_rad = np.radians(self.delta)

        # For PM synchronous motor with salient poles
        # Voltage equation in dq frame:
        # Vd = R1*Id - omega_e*Lq*Iq
        # Vq = R1*Iq + omega_e*Ld*Id + omega_e*lambda_pm

        # Inductances from reactances
        self.Ld = self.Xsd / self.omega_e
        self.Lq = self.Xsq / self.omega_e

        # Lambda_pm (PM flux linkage)
        self.lambda_pm = self.Ef / self.omega_e

        # For steady state with power angle delta:
        # Using approximate method for PM motor
        # Assuming V_phase is the terminal voltage

        # Current magnitude estimation
        # For PM motor: I = (V - Ef*cos(delta)) / Z_eff
        # where Z_eff considers both axes

        # Simplified approach: iterate to find Id and Iq
        # Using power equation: P = 3/2 * (Vd*Id + Vq*Iq)

        # Assume load angle relative to q-axis
        # Vq = V_phase * cos(delta_v)
        # Vd = -V_phase * sin(delta_v)

        # For this problem, using iterative solution
        delta_v = delta_rad  # voltage angle

        # Initial guess using simplified equations
        Vq = self.V_phase * np.cos(delta_v)
        Vd = -self.V_phase * np.sin(delta_v)

        # Solve for currents
        # Vd = R1*Id - omega_e*Lq*Iq
        # Vq = R1*Iq + omega_e*Ld*Id + omega_e*lambda_pm

        # Matrix solution: [R1, -omega_e*Lq] [Id]   [Vd]
        #                  [omega_e*Ld, R1]  [Iq] = [Vq - omega_e*lambda_pm]

        A = np.array([[self.R1, -self.omega_e * self.Lq],
                      [self.omega_e * self.Ld, self.R1]])
        B = np.array([Vd, Vq - self.omega_e * self.lambda_pm])

        try:
            currents = np.linalg.solve(A, B)
            self.Id = currents[0]
            self.Iq = currents[1]
        except:
            # Fallback method
            self.Iq = (Vq - self.omega_e * self.lambda_pm) / (self.R1 + 1e-10)
            self.Id = (Vd + self.omega_e * self.Lq * self.Iq) / (self.R1 + 1e-10)

        # Armature current magnitude
        self.Ia = np.sqrt(self.Id**2 + self.Iq**2)

        # Current angle
        self.phi_i = np.arctan2(self.Id, self.Iq)

        # Electromagnetic power
        self.Pelm = 1.5 * (Vd * self.Id + Vq * self.Iq)

        # Electromagnetic torque
        self.Telm = 1.5 * self.p * (self.lambda_pm * self.Iq + (self.Ld - self.Lq) * self.Id * self.Iq)

        # Copper losses
        self.Pcu = 3 * self.R1 * self.Ia**2

        # Output power (iterative because Pstr depends on Pout)
        # Pout = Pelm - Pcu - Prot - Pstr
        # Pstr = 0.05 * Pout
        # Pout = Pelm - Pcu - Prot - 0.05*Pout
        # 1.05*Pout = Pelm - Pcu - Prot
        self.Pout = (self.Pelm - self.Pcu - self.Prot) / 1.05

        # Stray losses
        self.Pstr = self.Pstr_factor * self.Pout

        # Shaft torque
        self.Tsh = self.Pout / self.omega_m if self.omega_m > 0 else 0

        # Input power
        self.Pin = self.Pelm

        # Efficiency
        self.efficiency = (self.Pout / self.Pin * 100) if self.Pin > 0 else 0

        # Power factor
        # Phase angle between voltage and current
        V_angle = delta_v
        I_angle = self.phi_i
        phi_angle = V_angle - I_angle
        self.power_factor = np.cos(phi_angle)

        return self.get_results()

    def get_results(self):
        """Return calculation results as dictionary"""
        return {
            'Ia': self.Ia,
            'Id': self.Id,
            'Iq': self.Iq,
            'Pout': self.Pout,
            'Pelm': self.Pelm,
            'Telm': self.Telm,
            'Tsh': self.Tsh,
            'efficiency': self.efficiency,
            'power_factor': self.power_factor,
            'Ef': self.Ef,
            'Pcu': self.Pcu,
            'Prot': self.Prot,
            'Pstr': self.Pstr,
            'omega_m': self.omega_m,
            'V_phase': self.V_phase
        }

    def motor_dynamics_dq(self, t, state, Vd, Vq, TL):
        """
        Differential equations for motor dynamics in dq frame
        state = [id, iq, omega, theta]
        """
        id, iq, omega, theta = state

        omega_e = self.p * omega

        # Electrical equations (dq frame)
        did_dt = (Vd - self.R1 * id + omega_e * self.Lq * iq) / self.Ld
        diq_dt = (Vq - self.R1 * iq - omega_e * self.Ld * id - omega_e * self.lambda_pm) / self.Lq

        # Electromagnetic torque
        Te = 1.5 * self.p * (self.lambda_pm * iq + (self.Ld - self.Lq) * id * iq)

        # Mechanical equation (simplified, assuming constant inertia)
        J = 0.5  # kg*m^2 (estimated)
        B = 0.01  # friction coefficient
        domega_dt = (Te - TL - B * omega) / J

        # Rotor angle
        dtheta_dt = omega

        return [did_dt, diq_dt, domega_dt, dtheta_dt]

    def motor_dynamics_abc(self, t, state, Va, Vb, Vc, TL):
        """
        Differential equations for motor dynamics in abc frame
        state = [ia, ib, ic, omega, theta]
        """
        ia, ib, ic, omega, theta = state

        # Back EMF in abc frame
        ea = self.Ef * np.sin(self.p * theta)
        eb = self.Ef * np.sin(self.p * theta - 2*np.pi/3)
        ec = self.Ef * np.sin(self.p * theta + 2*np.pi/3)

        # Average inductance for simplified model
        Ls = (self.Ld + self.Lq) / 2

        # Electrical equations
        dia_dt = (Va - self.R1 * ia - ea) / Ls
        dib_dt = (Vb - self.R1 * ib - eb) / Ls
        dic_dt = (Vc - self.R1 * ic - ec) / Ls

        # Electromagnetic torque (simplified)
        Te = (ea * ia + eb * ib + ec * ic) / omega if omega > 0.1 else 0

        # Mechanical equation
        J = 0.5
        B = 0.01
        domega_dt = (Te - TL - B * omega) / J

        dtheta_dt = omega

        return [dia_dt, dib_dt, dic_dt, domega_dt, dtheta_dt]


class AdvancedMotorSimulator(tk.Tk):
    """Advanced Synchronous Motor Simulator with GUI"""

    def __init__(self):
        super().__init__()

        self.title("Advanced Synchronous Motor Multi-Physics Simulator")
        self.geometry("1400x900")

        # Motor model
        self.motor = SynchronousMotorModel()

        # Simulation control
        self.sim_running = False
        self.sim_thread = None
        self.sim_time = 0
        self.sim_data = {
            'time': [],
            'ia': [], 'ib': [], 'ic': [],
            'id': [], 'iq': [],
            'torque': [],
            'speed': [],
            'power': [],
            'efficiency': [],
            'temp': []
        }

        # Temperature model (thermal physics)
        self.temperature = 25  # Celsius
        self.temp_ambient = 25
        self.thermal_resistance = 0.5  # K/W
        self.thermal_capacitance = 1000  # J/K

        # Create UI
        self.create_ui()

        # Bind resize event
        self.bind('<Configure>', self.on_window_resize)

        # Initial calculation
        self.calculate_and_display()

    def create_ui(self):
        """Create comprehensive user interface"""

        # Main container with notebook (tabs)
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Tab 1: Main Control
        self.tab_main = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_main, text='Main Control')

        # Tab 2: Dynamic Simulation
        self.tab_simulation = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_simulation, text='Dynamic Simulation')

        # Tab 3: Multi-Physics
        self.tab_physics = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_physics, text='Multi-Physics')

        # Tab 4: Analysis
        self.tab_analysis = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_analysis, text='Analysis')

        # Create content for each tab
        self.create_main_tab()
        self.create_simulation_tab()
        self.create_physics_tab()
        self.create_analysis_tab()

    def create_main_tab(self):
        """Create main control tab"""

        # Left panel - Parameters
        left_frame = ttk.LabelFrame(self.tab_main, text="Input Parameters", padding=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Parameter entries
        self.param_vars = {}
        parameters = [
            ('Poles', 'poles', self.motor.poles),
            ('Rated Power (kW)', 'P_rated', self.motor.P_rated/1000),
            ('Line Voltage (V)', 'V_line', self.motor.V_line),
            ('Frequency (Hz)', 'freq', self.motor.freq),
            ('R1 (Ω)', 'R1', self.motor.R1),
            ('Xsd (Ω)', 'Xsd', self.motor.Xsd),
            ('Xsq (Ω)', 'Xsq', self.motor.Xsq),
            ('Power Angle (°)', 'delta', self.motor.delta),
            ('Bmg (T)', 'Bmg', self.motor.Bmg),
            ('Turns/Phase', 'N1', self.motor.N1),
            ('Winding Factor', 'kw1', self.motor.kw1),
            ('Core Length (m)', 'Li', self.motor.Li),
            ('Diameter (m)', 'D', self.motor.D),
            ('Rotational Loss (W)', 'Prot', self.motor.Prot)
        ]

        for i, (label, key, value) in enumerate(parameters):
            ttk.Label(left_frame, text=label + ':').grid(row=i, column=0, sticky='w', pady=2)
            var = tk.DoubleVar(value=value)
            self.param_vars[key] = var
            entry = ttk.Entry(left_frame, textvariable=var, width=15)
            entry.grid(row=i, column=1, sticky='ew', pady=2, padx=5)

        # Calculate button
        calc_btn = ttk.Button(left_frame, text="Calculate", command=self.calculate_and_display)
        calc_btn.grid(row=len(parameters), column=0, columnspan=2, pady=10, sticky='ew')

        # Middle panel - Sliders
        middle_frame = ttk.LabelFrame(self.tab_main, text="Control Sliders", padding=10)
        middle_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        self.sliders = {}
        slider_params = [
            ('Load Torque (%)', 'load_torque', 0, 150, 100),
            ('Voltage (%)', 'voltage', 50, 120, 100),
            ('Frequency (%)', 'frequency', 50, 150, 100),
            ('Power Angle (°)', 'power_angle', 0, 90, self.motor.delta)
        ]

        for i, (label, key, min_val, max_val, init_val) in enumerate(slider_params):
            ttk.Label(middle_frame, text=label + ':').grid(row=i*2, column=0, sticky='w', pady=2)

            slider_frame = ttk.Frame(middle_frame)
            slider_frame.grid(row=i*2+1, column=0, sticky='ew', pady=2)

            var = tk.DoubleVar(value=init_val)
            self.sliders[key] = var

            slider = ttk.Scale(slider_frame, from_=min_val, to=max_val,
                             variable=var, orient='horizontal',
                             command=lambda v, k=key: self.on_slider_change(k))
            slider.pack(side='left', fill='x', expand=True)

            value_label = ttk.Label(slider_frame, text=f'{init_val:.1f}', width=8)
            value_label.pack(side='right')
            self.sliders[key + '_label'] = value_label

        # Right panel - Results
        right_frame = ttk.LabelFrame(self.tab_main, text="Results", padding=10)
        right_frame.grid(row=0, column=2, sticky='nsew', padx=5, pady=5)

        self.result_labels = {}
        results = [
            ('Armature Current (A)', 'Ia'),
            ('Id Current (A)', 'Id'),
            ('Iq Current (A)', 'Iq'),
            ('Output Power (kW)', 'Pout'),
            ('Electromagnetic Power (kW)', 'Pelm'),
            ('Electromagnetic Torque (Nm)', 'Telm'),
            ('Shaft Torque (Nm)', 'Tsh'),
            ('Efficiency (%)', 'efficiency'),
            ('Power Factor', 'power_factor'),
            ('Back EMF (V)', 'Ef'),
            ('Copper Loss (W)', 'Pcu'),
            ('Speed (rpm)', 'speed')
        ]

        for i, (label, key) in enumerate(results):
            ttk.Label(right_frame, text=label + ':').grid(row=i, column=0, sticky='w', pady=2)
            value_label = ttk.Label(right_frame, text='0.00', width=15,
                                   relief='sunken', anchor='e')
            value_label.grid(row=i, column=1, sticky='ew', pady=2, padx=5)
            self.result_labels[key] = value_label

        # Configure grid weights
        self.tab_main.grid_rowconfigure(0, weight=1)
        self.tab_main.grid_columnconfigure(0, weight=1)
        self.tab_main.grid_columnconfigure(1, weight=1)
        self.tab_main.grid_columnconfigure(2, weight=1)

    def create_simulation_tab(self):
        """Create dynamic simulation tab"""

        # Control frame
        control_frame = ttk.Frame(self.tab_simulation)
        control_frame.pack(side='top', fill='x', padx=5, pady=5)

        # Buttons
        self.btn_start = ttk.Button(control_frame, text="Start Simulation",
                                    command=self.start_simulation)
        self.btn_start.pack(side='left', padx=5)

        self.btn_stop = ttk.Button(control_frame, text="Stop Simulation",
                                   command=self.stop_simulation, state='disabled')
        self.btn_stop.pack(side='left', padx=5)

        self.btn_reset = ttk.Button(control_frame, text="Reset",
                                    command=self.reset_simulation)
        self.btn_reset.pack(side='left', padx=5)

        # Solver selection
        ttk.Label(control_frame, text="Solver:").pack(side='left', padx=5)
        self.solver_var = tk.StringVar(value='RK45')
        solver_combo = ttk.Combobox(control_frame, textvariable=self.solver_var,
                                    values=['RK45', 'Euler', 'RK23', 'DOP853'],
                                    width=10, state='readonly')
        solver_combo.pack(side='left', padx=5)

        # Time step
        ttk.Label(control_frame, text="Time Step (ms):").pack(side='left', padx=5)
        self.dt_var = tk.DoubleVar(value=1.0)
        ttk.Entry(control_frame, textvariable=self.dt_var, width=10).pack(side='left', padx=5)

        # Status
        self.status_label = ttk.Label(control_frame, text="Ready", relief='sunken')
        self.status_label.pack(side='right', padx=5, fill='x', expand=True)

        # Plots frame
        plots_frame = ttk.Frame(self.tab_simulation)
        plots_frame.pack(side='top', fill='both', expand=True, padx=5, pady=5)

        # Create matplotlib figure
        self.fig_sim = Figure(figsize=(12, 8))

        self.ax_currents = self.fig_sim.add_subplot(3, 2, 1)
        self.ax_currents.set_title('Phase Currents')
        self.ax_currents.set_xlabel('Time (s)')
        self.ax_currents.set_ylabel('Current (A)')
        self.ax_currents.grid(True)

        self.ax_dq = self.fig_sim.add_subplot(3, 2, 2)
        self.ax_dq.set_title('dq Currents')
        self.ax_dq.set_xlabel('Time (s)')
        self.ax_dq.set_ylabel('Current (A)')
        self.ax_dq.grid(True)

        self.ax_torque = self.fig_sim.add_subplot(3, 2, 3)
        self.ax_torque.set_title('Electromagnetic Torque')
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (Nm)')
        self.ax_torque.grid(True)

        self.ax_speed = self.fig_sim.add_subplot(3, 2, 4)
        self.ax_speed.set_title('Rotor Speed')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rpm)')
        self.ax_speed.grid(True)

        self.ax_power = self.fig_sim.add_subplot(3, 2, 5)
        self.ax_power.set_title('Output Power')
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.grid(True)

        self.ax_efficiency = self.fig_sim.add_subplot(3, 2, 6)
        self.ax_efficiency.set_title('Efficiency')
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.grid(True)

        self.fig_sim.tight_layout()

        self.canvas_sim = FigureCanvasTkAgg(self.fig_sim, plots_frame)
        self.canvas_sim.draw()
        self.canvas_sim.get_tk_widget().pack(fill='both', expand=True)

        # Toolbar
        toolbar_frame = ttk.Frame(plots_frame)
        toolbar_frame.pack(side='bottom', fill='x')
        toolbar = NavigationToolbar2Tk(self.canvas_sim, toolbar_frame)
        toolbar.update()

    def create_physics_tab(self):
        """Create multi-physics simulation tab"""

        # Left panel - Thermal model
        left_frame = ttk.LabelFrame(self.tab_physics, text="Thermal Model", padding=10)
        left_frame.pack(side='left', fill='both', expand=True, padx=5, pady=5)

        # Thermal parameters
        thermal_params = [
            ('Ambient Temp (°C)', 'temp_ambient', 25),
            ('Thermal Resistance (K/W)', 'thermal_resistance', 0.5),
            ('Thermal Capacitance (J/K)', 'thermal_capacitance', 1000)
        ]

        self.thermal_vars = {}
        for i, (label, key, value) in enumerate(thermal_params):
            ttk.Label(left_frame, text=label + ':').grid(row=i, column=0, sticky='w', pady=5)
            var = tk.DoubleVar(value=value)
            self.thermal_vars[key] = var
            ttk.Entry(left_frame, textvariable=var, width=15).grid(row=i, column=1,
                                                                    sticky='ew', pady=5, padx=5)

        # Temperature display
        ttk.Label(left_frame, text='Current Temperature:',
                 font=('Arial', 12, 'bold')).grid(row=3, column=0, pady=10)
        self.temp_display = ttk.Label(left_frame, text='25.0 °C',
                                     font=('Arial', 16), foreground='blue')
        self.temp_display.grid(row=3, column=1, pady=10)

        # Thermal plot
        thermal_plot_frame = ttk.LabelFrame(left_frame, text="Temperature Profile", padding=5)
        thermal_plot_frame.grid(row=4, column=0, columnspan=2, sticky='nsew', pady=5)

        self.fig_thermal = Figure(figsize=(6, 4))
        self.ax_thermal = self.fig_thermal.add_subplot(111)
        self.ax_thermal.set_title('Motor Temperature vs Time')
        self.ax_thermal.set_xlabel('Time (s)')
        self.ax_thermal.set_ylabel('Temperature (°C)')
        self.ax_thermal.grid(True)

        self.canvas_thermal = FigureCanvasTkAgg(self.fig_thermal, thermal_plot_frame)
        self.canvas_thermal.draw()
        self.canvas_thermal.get_tk_widget().pack(fill='both', expand=True)

        left_frame.grid_rowconfigure(4, weight=1)
        left_frame.grid_columnconfigure(1, weight=1)

        # Right panel - Magnetic and mechanical
        right_frame = ttk.LabelFrame(self.tab_physics, text="Magnetic & Mechanical", padding=10)
        right_frame.pack(side='right', fill='both', expand=True, padx=5, pady=5)

        # Flux density plot
        flux_frame = ttk.LabelFrame(right_frame, text="Air Gap Flux Density", padding=5)
        flux_frame.pack(fill='both', expand=True, pady=5)

        self.fig_flux = Figure(figsize=(6, 3))
        self.ax_flux = self.fig_flux.add_subplot(111, projection='polar')
        self.ax_flux.set_title('Flux Density Distribution')

        self.canvas_flux = FigureCanvasTkAgg(self.fig_flux, flux_frame)
        self.canvas_flux.draw()
        self.canvas_flux.get_tk_widget().pack(fill='both', expand=True)

        # Vibration analysis
        vib_frame = ttk.LabelFrame(right_frame, text="Vibration Analysis", padding=5)
        vib_frame.pack(fill='both', expand=True, pady=5)

        self.fig_vib = Figure(figsize=(6, 3))
        self.ax_vib = self.fig_vib.add_subplot(111)
        self.ax_vib.set_title('Vibration Spectrum')
        self.ax_vib.set_xlabel('Frequency (Hz)')
        self.ax_vib.set_ylabel('Amplitude')
        self.ax_vib.grid(True)

        self.canvas_vib = FigureCanvasTkAgg(self.fig_vib, vib_frame)
        self.canvas_vib.draw()
        self.canvas_vib.get_tk_widget().pack(fill='both', expand=True)

        # Update button
        ttk.Button(right_frame, text="Update Multi-Physics",
                  command=self.update_multiphysics).pack(pady=10)

    def create_analysis_tab(self):
        """Create analysis tab"""

        # Phasor diagram and characteristic curves

        # Top frame - Phasor diagram
        top_frame = ttk.LabelFrame(self.tab_analysis, text="Phasor Diagram", padding=10)
        top_frame.pack(side='top', fill='both', expand=True, padx=5, pady=5)

        self.fig_phasor = Figure(figsize=(8, 4))
        self.ax_phasor = self.fig_phasor.add_subplot(121)
        self.ax_phasor.set_title('Voltage-Current Phasor')
        self.ax_phasor.set_xlabel('Real')
        self.ax_phasor.set_ylabel('Imaginary')
        self.ax_phasor.grid(True)
        self.ax_phasor.axis('equal')

        self.ax_circle = self.fig_phasor.add_subplot(122, projection='polar')
        self.ax_circle.set_title('Power Circle Diagram')

        self.canvas_phasor = FigureCanvasTkAgg(self.fig_phasor, top_frame)
        self.canvas_phasor.draw()
        self.canvas_phasor.get_tk_widget().pack(fill='both', expand=True)

        # Bottom frame - Characteristic curves
        bottom_frame = ttk.LabelFrame(self.tab_analysis, text="Characteristic Curves", padding=10)
        bottom_frame.pack(side='bottom', fill='both', expand=True, padx=5, pady=5)

        self.fig_curves = Figure(figsize=(10, 4))

        self.ax_torque_speed = self.fig_curves.add_subplot(131)
        self.ax_torque_speed.set_title('Torque-Speed Characteristic')
        self.ax_torque_speed.set_xlabel('Speed (rpm)')
        self.ax_torque_speed.set_ylabel('Torque (Nm)')
        self.ax_torque_speed.grid(True)

        self.ax_eff_load = self.fig_curves.add_subplot(132)
        self.ax_eff_load.set_title('Efficiency vs Load')
        self.ax_eff_load.set_xlabel('Load (%)')
        self.ax_eff_load.set_ylabel('Efficiency (%)')
        self.ax_eff_load.grid(True)

        self.ax_pf_load = self.fig_curves.add_subplot(133)
        self.ax_pf_load.set_title('Power Factor vs Load')
        self.ax_pf_load.set_xlabel('Load (%)')
        self.ax_pf_load.set_ylabel('Power Factor')
        self.ax_pf_load.grid(True)

        self.fig_curves.tight_layout()

        self.canvas_curves = FigureCanvasTkAgg(self.fig_curves, bottom_frame)
        self.canvas_curves.draw()
        self.canvas_curves.get_tk_widget().pack(fill='both', expand=True)

        # Generate curves button
        ttk.Button(bottom_frame, text="Generate Characteristic Curves",
                  command=self.generate_curves).pack(pady=5)

    def on_slider_change(self, key):
        """Handle slider value changes"""
        value = self.sliders[key].get()
        self.sliders[key + '_label'].config(text=f'{value:.1f}')

        # Update motor parameters based on slider
        if key == 'voltage':
            self.motor.V_line = self.param_vars['V_line'].get() * value / 100
            self.motor.V_phase = self.motor.V_line / np.sqrt(3)
        elif key == 'frequency':
            self.motor.freq = self.param_vars['freq'].get() * value / 100
            self.motor.omega_e = 2 * np.pi * self.motor.freq
            self.motor.omega_m = self.motor.omega_e / self.motor.p
        elif key == 'power_angle':
            self.motor.delta = value

        # Recalculate if not simulating
        if not self.sim_running:
            self.calculate_and_display()

    def calculate_and_display(self):
        """Update motor parameters and calculate"""
        try:
            # Update motor parameters from entries
            self.motor.poles = int(self.param_vars['poles'].get())
            self.motor.P_rated = self.param_vars['P_rated'].get() * 1000
            self.motor.V_line = self.param_vars['V_line'].get()
            self.motor.freq = self.param_vars['freq'].get()
            self.motor.R1 = self.param_vars['R1'].get()
            self.motor.Xsd = self.param_vars['Xsd'].get()
            self.motor.Xsq = self.param_vars['Xsq'].get()
            self.motor.delta = self.param_vars['delta'].get()
            self.motor.Bmg = self.param_vars['Bmg'].get()
            self.motor.N1 = int(self.param_vars['N1'].get())
            self.motor.kw1 = self.param_vars['kw1'].get()
            self.motor.Li = self.param_vars['Li'].get()
            self.motor.D = self.param_vars['D'].get()
            self.motor.Prot = self.param_vars['Prot'].get()

            # Update calculated parameters
            self.motor.p = self.motor.poles / 2
            self.motor.omega_e = 2 * np.pi * self.motor.freq
            self.motor.omega_m = self.motor.omega_e / self.motor.p
            self.motor.V_phase = self.motor.V_line / np.sqrt(3)

            # Calculate
            results = self.motor.calculate_steady_state()

            # Display results
            self.result_labels['Ia'].config(text=f'{results["Ia"]:.3f}')
            self.result_labels['Id'].config(text=f'{results["Id"]:.3f}')
            self.result_labels['Iq'].config(text=f'{results["Iq"]:.3f}')
            self.result_labels['Pout'].config(text=f'{results["Pout"]/1000:.3f}')
            self.result_labels['Pelm'].config(text=f'{results["Pelm"]/1000:.3f}')
            self.result_labels['Telm'].config(text=f'{results["Telm"]:.3f}')
            self.result_labels['Tsh'].config(text=f'{results["Tsh"]:.3f}')
            self.result_labels['efficiency'].config(text=f'{results["efficiency"]:.3f}')
            self.result_labels['power_factor'].config(text=f'{results["power_factor"]:.4f}')
            self.result_labels['Ef'].config(text=f'{results["Ef"]:.3f}')
            self.result_labels['Pcu'].config(text=f'{results["Pcu"]:.3f}')

            speed_rpm = results["omega_m"] * 60 / (2 * np.pi)
            self.result_labels['speed'].config(text=f'{speed_rpm:.1f}')

            # Update phasor diagram
            self.update_phasor_diagram(results)

        except Exception as e:
            messagebox.showerror("Calculation Error", str(e))

    def start_simulation(self):
        """Start dynamic simulation"""
        if not self.sim_running:
            self.sim_running = True
            self.btn_start.config(state='disabled')
            self.btn_stop.config(state='normal')
            self.status_label.config(text="Simulation Running...")

            # Start simulation thread
            self.sim_thread = threading.Thread(target=self.run_simulation, daemon=True)
            self.sim_thread.start()

    def stop_simulation(self):
        """Stop dynamic simulation"""
        self.sim_running = False
        self.btn_start.config(state='normal')
        self.btn_stop.config(state='disabled')
        self.status_label.config(text="Simulation Stopped")

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()
        self.sim_time = 0
        self.temperature = self.temp_ambient

        # Clear data
        for key in self.sim_data:
            self.sim_data[key] = []

        # Clear plots
        self.ax_currents.clear()
        self.ax_dq.clear()
        self.ax_torque.clear()
        self.ax_speed.clear()
        self.ax_power.clear()
        self.ax_efficiency.clear()

        self.ax_currents.set_title('Phase Currents')
        self.ax_currents.set_xlabel('Time (s)')
        self.ax_currents.set_ylabel('Current (A)')
        self.ax_currents.grid(True)

        self.ax_dq.set_title('dq Currents')
        self.ax_dq.set_xlabel('Time (s)')
        self.ax_dq.set_ylabel('Current (A)')
        self.ax_dq.grid(True)

        self.ax_torque.set_title('Electromagnetic Torque')
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (Nm)')
        self.ax_torque.grid(True)

        self.ax_speed.set_title('Rotor Speed')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rpm)')
        self.ax_speed.grid(True)

        self.ax_power.set_title('Output Power')
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.grid(True)

        self.ax_efficiency.set_title('Efficiency')
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.grid(True)

        self.canvas_sim.draw()

        self.status_label.config(text="Simulation Reset")

    def run_simulation(self):
        """Run dynamic simulation in separate thread"""
        dt = self.dt_var.get() / 1000  # Convert ms to s
        solver_method = self.solver_var.get()

        # Initial conditions
        omega_init = self.motor.omega_m
        theta_init = 0
        id_init = self.motor.Id
        iq_init = self.motor.Iq

        # State vector
        state = [id_init, iq_init, omega_init, theta_init]

        # Load torque
        TL = self.motor.Tsh * self.sliders['load_torque'].get() / 100

        # Voltage in dq frame
        delta_rad = np.radians(self.motor.delta)
        Vq = self.motor.V_phase * np.cos(delta_rad)
        Vd = -self.motor.V_phase * np.sin(delta_rad)

        t = 0
        t_end = 10.0  # Simulate for 10 seconds

        if solver_method == 'Euler':
            # Euler method
            while self.sim_running and t < t_end:
                # Compute derivatives
                derivs = self.motor.motor_dynamics_dq(t, state, Vd, Vq, TL)

                # Update state
                state = [state[i] + derivs[i] * dt for i in range(len(state))]

                t += dt
                self.sim_time = t

                # Store data
                self.store_simulation_data(t, state)

                # Update plots periodically
                if len(self.sim_data['time']) % 50 == 0:
                    self.after(0, self.update_simulation_plots)

                time.sleep(dt / 10)  # Slow down for visualization

        else:
            # Use scipy integrators
            def dynamics(t, y):
                return self.motor.motor_dynamics_dq(t, y, Vd, Vq, TL)

            # Create integrator
            if solver_method == 'RK45':
                method = 'RK45'
            elif solver_method == 'RK23':
                method = 'RK23'
            elif solver_method == 'DOP853':
                method = 'DOP853'
            else:
                method = 'RK45'

            # Time points
            t_eval = np.arange(0, t_end, dt)

            # Solve
            sol = solve_ivp(dynamics, [0, t_end], state, method=method,
                          t_eval=t_eval, dense_output=True)

            # Process solution
            for i, t in enumerate(sol.t):
                if not self.sim_running:
                    break

                state_i = sol.y[:, i]
                self.sim_time = t
                self.store_simulation_data(t, state_i)

                # Update plots periodically
                if i % 50 == 0:
                    self.after(0, self.update_simulation_plots)

                time.sleep(dt / 10)

        self.after(0, self.stop_simulation)

    def store_simulation_data(self, t, state):
        """Store simulation data"""
        id_val, iq, omega, theta = state

        # Store time
        self.sim_data['time'].append(t)

        # dq currents
        self.sim_data['id'].append(id_val)
        self.sim_data['iq'].append(iq)

        # abc currents (Park inverse transform)
        ia = id_val * np.cos(theta) - iq * np.sin(theta)
        ib = id_val * np.cos(theta - 2*np.pi/3) - iq * np.sin(theta - 2*np.pi/3)
        ic = id_val * np.cos(theta + 2*np.pi/3) - iq * np.sin(theta + 2*np.pi/3)

        self.sim_data['ia'].append(ia)
        self.sim_data['ib'].append(ib)
        self.sim_data['ic'].append(ic)

        # Torque
        Te = 1.5 * self.motor.p * (self.motor.lambda_pm * iq +
                                    (self.motor.Ld - self.motor.Lq) * id_val * iq)
        self.sim_data['torque'].append(Te)

        # Speed (rpm)
        speed_rpm = omega * 60 / (2 * np.pi)
        self.sim_data['speed'].append(speed_rpm)

        # Power
        power = Te * omega / 1000  # kW
        self.sim_data['power'].append(power)

        # Efficiency (simplified)
        Pcu = 3 * self.motor.R1 * (id_val**2 + iq**2)
        Pin = power * 1000 + Pcu + self.motor.Prot
        eff = (power * 1000 / Pin * 100) if Pin > 0 else 0
        self.sim_data['efficiency'].append(eff)

        # Update temperature (thermal model)
        P_loss = Pcu + self.motor.Prot
        dT_dt = (P_loss * self.thermal_resistance -
                (self.temperature - self.temp_ambient)) / self.thermal_capacitance
        self.temperature += dT_dt * 0.001  # Assuming 1ms time step
        self.sim_data['temp'].append(self.temperature)

    def update_simulation_plots(self):
        """Update simulation plots"""
        if len(self.sim_data['time']) < 2:
            return

        time_data = self.sim_data['time']

        # Phase currents
        self.ax_currents.clear()
        self.ax_currents.plot(time_data, self.sim_data['ia'], 'r-', label='Ia', linewidth=1.5)
        self.ax_currents.plot(time_data, self.sim_data['ib'], 'g-', label='Ib', linewidth=1.5)
        self.ax_currents.plot(time_data, self.sim_data['ic'], 'b-', label='Ic', linewidth=1.5)
        self.ax_currents.set_title('Phase Currents')
        self.ax_currents.set_xlabel('Time (s)')
        self.ax_currents.set_ylabel('Current (A)')
        self.ax_currents.legend()
        self.ax_currents.grid(True)

        # dq currents
        self.ax_dq.clear()
        self.ax_dq.plot(time_data, self.sim_data['id'], 'r-', label='Id', linewidth=1.5)
        self.ax_dq.plot(time_data, self.sim_data['iq'], 'b-', label='Iq', linewidth=1.5)
        self.ax_dq.set_title('dq Currents')
        self.ax_dq.set_xlabel('Time (s)')
        self.ax_dq.set_ylabel('Current (A)')
        self.ax_dq.legend()
        self.ax_dq.grid(True)

        # Torque
        self.ax_torque.clear()
        self.ax_torque.plot(time_data, self.sim_data['torque'], 'g-', linewidth=2)
        self.ax_torque.set_title('Electromagnetic Torque')
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (Nm)')
        self.ax_torque.grid(True)

        # Speed
        self.ax_speed.clear()
        self.ax_speed.plot(time_data, self.sim_data['speed'], 'm-', linewidth=2)
        self.ax_speed.set_title('Rotor Speed')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rpm)')
        self.ax_speed.grid(True)

        # Power
        self.ax_power.clear()
        self.ax_power.plot(time_data, self.sim_data['power'], 'c-', linewidth=2)
        self.ax_power.set_title('Output Power')
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.grid(True)

        # Efficiency
        self.ax_efficiency.clear()
        self.ax_efficiency.plot(time_data, self.sim_data['efficiency'], 'orange', linewidth=2)
        self.ax_efficiency.set_title('Efficiency')
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.grid(True)

        self.fig_sim.tight_layout()
        self.canvas_sim.draw()

        # Update status
        self.status_label.config(text=f"Simulation Time: {self.sim_time:.3f} s")

    def update_multiphysics(self):
        """Update multi-physics visualizations"""
        # Update thermal parameters
        self.temp_ambient = self.thermal_vars['temp_ambient'].get()
        self.thermal_resistance = self.thermal_vars['thermal_resistance'].get()
        self.thermal_capacitance = self.thermal_vars['thermal_capacitance'].get()

        # Temperature plot
        if len(self.sim_data['time']) > 0:
            self.ax_thermal.clear()
            self.ax_thermal.plot(self.sim_data['time'], self.sim_data['temp'],
                               'r-', linewidth=2)
            self.ax_thermal.axhline(y=self.temp_ambient, color='b',
                                   linestyle='--', label='Ambient')
            self.ax_thermal.set_title('Motor Temperature vs Time')
            self.ax_thermal.set_xlabel('Time (s)')
            self.ax_thermal.set_ylabel('Temperature (°C)')
            self.ax_thermal.legend()
            self.ax_thermal.grid(True)
            self.canvas_thermal.draw()

            # Update temperature display
            self.temp_display.config(text=f'{self.temperature:.1f} °C')

        # Flux density distribution (polar plot)
        self.ax_flux.clear()
        theta = np.linspace(0, 2*np.pi, 100)
        # Approximate flux density with harmonics
        B_theta = self.motor.Bmg * np.cos(self.motor.p * theta)
        self.ax_flux.plot(theta, np.abs(B_theta), 'b-', linewidth=2)
        self.ax_flux.fill(theta, np.abs(B_theta), alpha=0.3)
        self.ax_flux.set_title('Air Gap Flux Density Distribution', pad=20)
        self.canvas_flux.draw()

        # Vibration spectrum (FFT of torque ripple)
        if len(self.sim_data['time']) > 100:
            torque_data = np.array(self.sim_data['torque'][-1000:])
            dt = np.mean(np.diff(self.sim_data['time'][-1000:]))

            # FFT
            fft_torque = np.fft.fft(torque_data - np.mean(torque_data))
            freqs = np.fft.fftfreq(len(torque_data), dt)

            # Positive frequencies only
            pos_mask = freqs > 0
            freqs_pos = freqs[pos_mask]
            fft_mag = np.abs(fft_torque[pos_mask])

            self.ax_vib.clear()
            self.ax_vib.stem(freqs_pos[:50], fft_mag[:50], basefmt=' ')
            self.ax_vib.set_title('Vibration Spectrum (Torque Ripple)')
            self.ax_vib.set_xlabel('Frequency (Hz)')
            self.ax_vib.set_ylabel('Amplitude')
            self.ax_vib.grid(True)
            self.canvas_vib.draw()

    def update_phasor_diagram(self, results):
        """Update phasor diagram"""
        self.ax_phasor.clear()

        # Voltage phasor (reference)
        V = results['V_phase']
        self.ax_phasor.arrow(0, 0, V, 0, head_width=10, head_length=10,
                            fc='red', ec='red', linewidth=2, label='V')

        # Current phasor
        I = results['Ia']
        phi = np.arccos(results['power_factor'])
        if results['power_factor'] < 0:
            phi = -phi

        Ix = I * np.cos(-phi)
        Iy = I * np.sin(-phi)
        self.ax_phasor.arrow(0, 0, Ix * 50, Iy * 50, head_width=10, head_length=10,
                            fc='blue', ec='blue', linewidth=2, label='I (×50)')

        # Back EMF phasor
        Ef = results['Ef']
        delta_rad = np.radians(self.motor.delta)
        Ex = Ef * np.cos(-delta_rad)
        Ey = Ef * np.sin(-delta_rad)
        self.ax_phasor.arrow(0, 0, Ex, Ey, head_width=10, head_length=10,
                            fc='green', ec='green', linewidth=2, label='Ef')

        self.ax_phasor.set_title('Voltage-Current Phasor Diagram')
        self.ax_phasor.set_xlabel('Real')
        self.ax_phasor.set_ylabel('Imaginary')
        self.ax_phasor.legend()
        self.ax_phasor.grid(True)
        self.ax_phasor.axis('equal')

        # Power circle
        self.ax_circle.clear()
        angles = np.linspace(0, 2*np.pi, 100)
        P_max = 3 * V * Ef / self.motor.Xsd
        radius = P_max / 2000  # Scale for visualization

        self.ax_circle.plot(angles, np.ones_like(angles) * radius, 'b--', linewidth=1)

        # Current operating point
        P_current = results['Pelm'] / 2000
        angle_current = delta_rad
        self.ax_circle.plot(angle_current, P_current, 'ro', markersize=10,
                           label='Operating Point')

        self.ax_circle.set_title('Power Circle Diagram')
        self.ax_circle.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))

        self.canvas_phasor.draw()

    def generate_curves(self):
        """Generate characteristic curves"""
        # Torque-Speed curve
        speeds = np.linspace(0, self.motor.omega_m * 1.5, 50)
        torques = []

        for omega in speeds:
            # Simplified torque calculation
            if omega > 0:
                T = self.motor.Telm * (self.motor.omega_m / omega)
            else:
                T = 0
            torques.append(T)

        self.ax_torque_speed.clear()
        speeds_rpm = speeds * 60 / (2 * np.pi)
        self.ax_torque_speed.plot(speeds_rpm, torques, 'b-', linewidth=2)
        self.ax_torque_speed.axvline(x=self.motor.omega_m * 60 / (2*np.pi),
                                     color='r', linestyle='--', label='Rated Speed')
        self.ax_torque_speed.set_title('Torque-Speed Characteristic')
        self.ax_torque_speed.set_xlabel('Speed (rpm)')
        self.ax_torque_speed.set_ylabel('Torque (Nm)')
        self.ax_torque_speed.legend()
        self.ax_torque_speed.grid(True)

        # Efficiency vs Load
        loads = np.linspace(10, 150, 50)
        efficiencies = []

        for load_pct in loads:
            # Approximate efficiency curve
            # Efficiency typically peaks around 75-100% load
            if load_pct < 100:
                eff = self.motor.efficiency * (0.8 + 0.2 * load_pct / 100)
            else:
                eff = self.motor.efficiency * (1.0 - 0.001 * (load_pct - 100))
            efficiencies.append(eff)

        self.ax_eff_load.clear()
        self.ax_eff_load.plot(loads, efficiencies, 'g-', linewidth=2)
        self.ax_eff_load.axvline(x=100, color='r', linestyle='--', label='Rated Load')
        self.ax_eff_load.set_title('Efficiency vs Load')
        self.ax_eff_load.set_xlabel('Load (%)')
        self.ax_eff_load.set_ylabel('Efficiency (%)')
        self.ax_eff_load.legend()
        self.ax_eff_load.grid(True)

        # Power Factor vs Load
        power_factors = []

        for load_pct in loads:
            # Approximate power factor curve
            if load_pct < 100:
                pf = self.motor.power_factor * (0.7 + 0.3 * load_pct / 100)
            else:
                pf = self.motor.power_factor * (1.0 - 0.002 * (load_pct - 100))
            power_factors.append(pf)

        self.ax_pf_load.clear()
        self.ax_pf_load.plot(loads, power_factors, 'm-', linewidth=2)
        self.ax_pf_load.axvline(x=100, color='r', linestyle='--', label='Rated Load')
        self.ax_pf_load.set_title('Power Factor vs Load')
        self.ax_pf_load.set_xlabel('Load (%)')
        self.ax_pf_load.set_ylabel('Power Factor')
        self.ax_pf_load.legend()
        self.ax_pf_load.grid(True)

        self.fig_curves.tight_layout()
        self.canvas_curves.draw()

    def on_window_resize(self, event):
        """Handle window resize for autoscaling"""
        # This will be called on window resize
        # Matplotlib canvases automatically handle resize
        pass


def main():
    """Main entry point"""
    app = AdvancedMotorSimulator()
    app.mainloop()


if __name__ == '__main__':
    main()
