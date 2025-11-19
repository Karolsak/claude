# Advanced Motor Analysis & Simulation System

A comprehensive Python + Tkinter application for electrical motor analysis with multi-physics simulation capabilities.

## Problems Solved

### 1. Flywheel Inertia Problem ✨
- **Problem**: Motor fitted with flywheel supplies load torque of 150 kg-m for 15 sec
  - Motor torque limited to: 85 kg-m
  - No load speed: 500 rpm
  - Full load slip: 10%
  - Find: Moment of inertia of flywheel

- **Solution**: **J = 1825.14 kg·m²** ✅
  - Method: Energy balance approach
  - Torque deficit: 637.65 N·m (supplied by flywheel)
  - Speed range: 450-500 rpm (47.12-52.36 rad/s)
  - Energy stored: 26,080.8 J

### 2. Motor Rating Problem Solution
- Solves the RMS horsepower rating for motors with variable load cycles
- **Problem**: Motor with load cycle:
  - Acceleration: 0 to 2000 hp linearly over 20 sec
  - Full speed: 1000 hp for 40 sec
  - Deceleration/Regenerative: 330 to 0 hp over 10 sec
  - Rest: 0 hp for 20 sec
- **Solution**: RMS HP = 1154.70 hp (approx 861.41 kW)

## Features

### 1. User Interface (Tkinter GUI)
- **Main Control Tab**: Real-time motor control and monitoring
- **Dynamic Simulation Tab**: Multi-state variable visualization
- **Thermal Analysis Tab**: Temperature tracking and derating curves
- **Loss Analysis Tab**: Detailed breakdown of motor losses
- **Economic Analysis Tab**: Cost calculations and lifecycle analysis
- **Advanced Controls Tab**: Various control methods including PID

### 2. Mathematical Modeling
- **DC Motor Model**: Complete differential equations for:
  - Electrical dynamics (armature and field circuits)
  - Mechanical dynamics (torque, speed, position)
  - Thermal dynamics (temperature rise)

- **State Variables**: [ia, if, ω, θ, T]
  - ia: Armature current (A)
  - if: Field current (A)
  - ω: Angular velocity (rad/s)
  - θ: Angular position (rad)
  - T: Temperature (°C)

### 3. Dynamic Simulation
- **Multiple ODE Solvers**:
  - **RK45**: Adaptive Runge-Kutta (default, most accurate)
  - **Euler**: Simple forward Euler method
  - **RK4**: Classical 4th-order Runge-Kutta

- Real-time simulation with adjustable parameters
- Interactive control sliders for voltage and load torque

### 4. Multi-Physics Simulation
- **Electromagnetic Model**: Torque production, back-EMF
- **Thermal Model**: Heat generation and dissipation
  - Coupled electromagnetic-thermal equations
  - Thermal resistance and capacitance modeling
  - Derating curves for overtemperature protection
- **Mechanical Model**: Shaft torque, inertia, friction
  - Bearing load calculations
  - Transient analysis

### 5. Loss Analysis
Detailed breakdown of motor losses:
- **Copper Losses**: I²R losses in armature and field windings
- **Iron Losses**: Core losses proportional to speed²
- **Mechanical Losses**: Friction and windage
- **Stray Load Losses**: Additional losses under load
- Real-time efficiency calculation
- Visual pie chart and time-series plots

### 6. Visualization
- **Dynamic Graphs**: Real-time updating plots
- **Main Plots**: Speed, current, torque, power vs time
- **Phase Portrait**: Speed vs current state-space plot
- **Loss Breakdown**: Bar charts and pie charts
- **Thermal Curves**: Temperature rise and derating factor

### 7. Economic Analysis
- Energy cost calculation ($/kWh)
- Maintenance cost tracking
- Lifecycle cost analysis (10-year projection)
- Cost per operating hour
- Cumulative cost visualization
- Payback period estimation

### 8. Advanced Controls
Multiple control strategies:
- Open Loop Control
- Armature Voltage Control
- Field Flux Control
- Armature Resistance Control
- **PID Speed Control**: Proportional-Integral-Derivative
- Vector Control
- Field-Oriented Control (FOC)

### 9. Thermal & Derating
- Real-time temperature monitoring
- Automatic derating curve:
  - Full power: T < 100°C
  - Linear derating: 100-120°C (20% reduction per 20°C)
  - Shutdown protection: T > 120°C
- Thermal time constant calculation

### 10. Power Consumption Tracking
- Input power monitoring
- Output power calculation
- Real-time efficiency
- Energy consumption integration

### 11. Auto-Scaling
- Automatic width and height adjustment on window resize
- Responsive layout using Tkinter pack geometry manager
- Canvas auto-scaling for plots

## Installation

```bash
# Install required packages
pip install -r requirements.txt

# Run the application
python3 advanced_motor_analysis.py
```

## Usage

1. **Start**: Click "Start" button to begin simulation
2. **Adjust Parameters**: Use sliders to change:
   - Armature Voltage (0-440V)
   - Field Voltage (0-440V)
   - Load Torque (0-100 N·m)
3. **Select Solver**: Choose from RK45, Euler, or RK4
4. **Monitor**: Real-time status display shows current values
5. **Stop/Reset**: Control simulation flow
6. **Analyze**: Switch between tabs for different analyses

## Controls

- **Start**: Begin/resume simulation
- **Stop**: Pause simulation
- **Reset**: Clear all data and restart

## Tabs Overview

### Main Control
- Parameter sliders
- Real-time status display
- 4-panel dynamic graphs

### Dynamic Simulation
- 6-panel detailed state visualization
- Phase portrait analysis
- Complete system dynamics

### Thermal Analysis
- Temperature rise curves
- Derating factor visualization
- Thermal protection information

### Loss Analysis
- Real-time loss breakdown
- Efficiency tracking
- Pie chart distribution

### Economic Analysis
- Cost parameter input
- Lifecycle calculations
- Cost visualization charts

### Advanced Controls
- Control method selection
- PID parameter tuning
- Control signal visualization

## Motor Parameters

Default DC motor parameters:
- Ra = 0.5 Ω (Armature resistance)
- La = 0.01 H (Armature inductance)
- Rf = 100 Ω (Field resistance)
- Lf = 10 H (Field inductance)
- J = 0.5 kg·m² (Inertia)
- B = 0.1 N·m·s/rad (Friction)
- Kt = 1.0 N·m/A (Torque constant)
- Kb = 1.0 V·s/rad (Back-EMF constant)

## Technical Details

### Differential Equations

**Electrical:**
```
dia/dt = (Va - ia·Ra - Eb) / La
dif/dt = (Vf - if·Rf) / Lf
```

**Mechanical:**
```
dω/dt = (Te - Tload - B·ω) / J
dθ/dt = ω
```

**Thermal:**
```
dT/dt = (Ploss - (T-Tamb)/Rth) / Cth
```

Where:
- Eb = Kb·if·ω (Back-EMF)
- Te = Kt·if·ia (Electromagnetic torque)
- Ploss = Total losses

### RMS Calculation

For the motor rating problem:
```
RMS HP = √(∫P²dt / T)
```

Result: **1154.70 hp** or **861.41 kW**

## Applications

- Motor design and sizing
- Educational tool for electrical engineering
- Control system design and testing
- Thermal analysis and protection design
- Economic feasibility studies
- Load cycle analysis
- Performance optimization

## Future Enhancements

Potential additions:
- Database storage for simulation results
- Export to CSV/Excel
- Custom load profile editor
- Multi-motor comparison
- Fault simulation modes
- Harmonics analysis
- Three-phase induction motor complete implementation

## License

Educational and research purposes.

## Author

Created using advanced electrical engineering principles and Python scientific computing libraries.
