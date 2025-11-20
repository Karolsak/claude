# Advanced Multi-Physics Electrical Engineering Simulator

## Overview
A comprehensive Python + Tkinter application combining illumination analysis with multi-physics electrical machine simulation. Features real-time dynamic modeling with advanced ODE solvers.

## Features

### 1. Illumination Analysis (Photometry)
- **Problem Solved**: Lamp with uniform candle power and reflector illuminating a circular disc
- **Calculations**:
  - Illumination at center without reflector
  - Illumination at edge without reflector
  - Illumination at center with reflector (50% light redirected)
  - Illumination at edge with reflector
- **Visualization**: 3D surface plots showing illumination distribution
- **Interactive Controls**: Adjustable sliders for candle power, height, and disc diameter

### 2. Induction Motor Simulation
- **Model**: Three-phase squirrel-cage induction motor
- **State Variables**: dq-axis stator currents, rotor currents, speed, position
- **Parameters**: 
  - RMS voltage and frequency
  - Stator and rotor resistances
  - Load torque, inertia
- **Outputs**: 
  - Real-time current waveforms
  - Speed response
  - Electromagnetic torque

### 3. Synchronous Machine Simulation
- **Model**: Synchronous generator/motor with field excitation
- **State Variables**: dq-axis currents, angular velocity, power angle δ
- **Parameters**:
  - RMS voltage and frequency
  - Field voltage
  - d-axis and q-axis reactances
  - Mechanical torque
- **Outputs**:
  - dq current dynamics
  - Speed and power angle evolution
  - Phase portrait analysis

### 4. Transformer with Thermal Analysis
- **Model**: Single-phase transformer with coupled thermal dynamics
- **State Variables**: Primary current, secondary current, winding temperature
- **Parameters**:
  - Primary voltage (RMS)
  - Winding resistances
  - Load resistance
  - Thermal properties (capacitance, resistance)
  - Ambient temperature
- **Outputs**:
  - Primary and secondary current waveforms
  - Winding temperature rise
  - Copper loss analysis

## Mathematical Models

### Illumination Calculation
```
Without Reflector:
E_center = I × cos(0°) / h²
E_edge = I × cos(θ) / d²
where d = √(h² + r²), cos(θ) = h/d

With Reflector:
E_total = E_direct + E_reflected
E_reflected = (2πI) / (πr²) = 2I/r² (uniform)
```

### Induction Motor (dq-frame)
```
dids/dt = (vds - Rs·ids + ωe·λqs) / Ls
diqs/dt = (vqs - Rs·iqs - ωe·λds) / Ls
didr/dt = (-Rr·idr + ωslip·λqr) / Lr
diqr/dt = (-Rr·iqr - ωslip·λdr) / Lr
dω/dt = (Te - TL - B·ω) / J

Te = 1.5·(P/2)·Lm·(iqs·idr - ids·iqr)
```

### Synchronous Machine
```
did/dt = (vd - Ra·id + ω·Xq·iq) / Xd
diq/dt = (vq - Ra·iq - ω·Xd·id - Eq') / Xq
dω/dt = (Tm - Te - D·ω) / J
dδ/dt = ω - ωe

Te = Eq'·iq + (Xd - Xq)·id·iq
```

### Transformer with Thermal
```
di1/dt = (v1 - R1·i1 - ω·M·i2) / L1
di2/dt = (ω·M·i1 - R2·i2 - RL·i2) / L2
dT/dt = (Ploss - (T - Tamb)/Rth) / Cth

Ploss = R1·i1² + R2·i2²
```

## ODE Solvers

### 1. RK45 (Runge-Kutta 4-5)
- **Method**: Adaptive step-size Dormand-Prince method
- **Features**: High accuracy, automatic error control
- **Use Case**: Recommended for all simulations
- **Tolerance**: rtol=1e-6, atol=1e-9

### 2. Euler Method
- **Method**: First-order explicit integration
- **Features**: Simple, fast, educational
- **Use Case**: Quick simulations, stability analysis
- **Note**: Fixed time step

## User Interface

### Main Menu
- **File**: Reset all simulations, Exit
- **Solver**: Select ODE solver (RK45 or Euler)
- **Help**: About dialog

### Control Elements
- **Sliders**: Real-time parameter adjustment with live feedback
- **Buttons**:
  - **Start**: Begin dynamic simulation
  - **Stop**: Halt running simulation
  - **Reset**: Clear results and reset states
- **Status Bar**: Shows current operation and solver status

### Visualization
- **Auto-scaling**: Plots automatically adjust to window size
- **Multi-panel**: Up to 4 subplots per simulation
- **Real-time**: Updates during simulation
- **3D Plots**: Surface plots for illumination distribution

## Installation

### Requirements
```bash
pip install numpy matplotlib scipy
```

Note: tkinter is usually included with Python. On Linux:
```bash
sudo apt-get install python3-tk  # Ubuntu/Debian
sudo yum install python3-tkinter  # Fedora/RHEL
```

### Running the Application
```bash
python3 illumination_multiphysics_simulator.py
```

## Usage Guide

### Illumination Analysis
1. Navigate to "Illumination Analysis" tab
2. Adjust parameters:
   - Candle Power (100-1000 cd)
   - Height (5-50 m)
   - Disc Diameter (5-40 m)
3. Click "Calculate Illumination"
4. View results in text panel and 3D visualizations

### Electrical Machine Simulations
1. Select desired machine tab:
   - Induction Motor
   - Synchronous Machine
   - Transformer + Thermal
2. Adjust machine parameters using sliders
3. Select solver from menu (RK45 recommended)
4. Click "Start" to begin simulation
5. Observe real-time plots
6. Click "Stop" to halt or "Reset" to clear

## Technical Specifications

### Illumination Module
- **Input**: Candle power (cd), height (m), diameter (m)
- **Output**: Illumination (lux)
- **Validation**: Physical photometry principles

### Dynamic Simulation
- **Time Span**: 0-2 seconds
- **Time Points**: 500 evaluation points
- **State Dimensions**: 
  - Induction Motor: 6 states
  - Synchronous Machine: 4 states
  - Transformer: 3 states
- **RMS Values**: All voltage/current inputs use RMS for practical engineering

### Performance
- **Thread Safety**: Background simulation threads
- **Responsiveness**: Non-blocking UI during computation
- **Memory**: Efficient numpy array operations

## Practical Applications

1. **Lighting Design**: Evaluate reflector effectiveness for industrial/architectural lighting
2. **Motor Control**: Analyze startup transients, torque response, speed regulation
3. **Power System Stability**: Study synchronous machine dynamics, power angle behavior
4. **Transformer Design**: Thermal management, loading capacity, cooling requirements
5. **Education**: Visualize complex electrical engineering concepts
6. **Research**: Multi-physics coupling analysis

## Example Results

### Illumination (Default Parameters)
- Lamp: 300 cd at 20m height
- Disc: 20m diameter
- **Without Reflector**:
  - Center: 0.75 lux
  - Edge: 0.537 lux
- **With Reflector**:
  - Center: 6.75 lux (9x improvement)
  - Edge: 6.537 lux (12x improvement)

### Induction Motor (Default)
- 230V RMS, 50 Hz
- Startup transient: ~0.5s to steady-state
- Speed: Converges to near-synchronous (1450 RPM for 4-pole, 50Hz)
- Torque: Initial surge then settles to load torque

## Troubleshooting

### Import Errors
- Ensure all packages installed: `pip install -r requirements.txt`
- Check tkinter availability: `python3 -m tkinter`

### Simulation Errors
- Reduce time span if solver fails
- Try Euler method for stiff systems
- Check parameter ranges (avoid zero/negative values)

### Performance Issues
- Close unused tabs
- Reduce simulation time span
- Use RK45 with default tolerances

## Future Enhancements
- Multi-machine power system simulation
- Harmonic analysis and FFT
- Control system integration (PID, FOC)
- Export results to CSV/MATLAB
- 3D animation of rotating machinery
- Thermal imaging visualization

## Author
Advanced electrical engineering simulation tool for research and education.

## License
Open source - Educational and research use
