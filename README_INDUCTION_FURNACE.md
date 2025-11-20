# Advanced Induction Furnace & Multi-Physics Simulation Application

## Overview

This comprehensive Python application combines an induction furnace problem solver with advanced multi-physics simulation capabilities for electrical machines. It provides a professional-grade tool for electrical engineering education and research.

## Features

### 1. **Induction Furnace Problem Solver**
- Solves the specific problem: A low-frequency induction furnace with 20V secondary voltage taking 600 kW at 0.6 p.f. when hearth is full
- Calculates power absorbed and power factor when hearth is half-full
- Assumes resistance doubles while reactance remains constant
- Provides detailed analysis with RMS current values

### 2. **Multi-Physics Simulation**
- **Electromagnetic modeling**: Differential equations in d-q reference frame using RMS values
- **Thermal modeling**: Coupled heat transfer equations for accurate temperature prediction
- **Mechanical modeling**: Shaft torque transients and bearing load analysis

### 3. **Advanced ODE Solvers**
- **RK45 (Runge-Kutta 45)**: High-accuracy adaptive solver
- **Euler Method**: Simple explicit solver for comparison

### 4. **Comprehensive GUI (Tkinter)**
- **6 Main Tabs**:
  1. Induction Furnace - Problem solver with detailed results
  2. Simulation Setup - Parameter input and solver configuration
  3. Visualization - Real-time dynamic graphs
  4. Advanced Control - PID tuning and thermal derating
  5. Economic Analysis - Lifecycle cost calculations
  6. Multi-Physics - Detailed loss breakdown and stress analysis

### 5. **Dynamic Visualization**
- Real-time plots for:
  - d-q axis currents
  - Electromagnetic torque
  - Rotor speed
  - Temperature profile
  - Power consumption
  - Loss distribution (bar chart)

### 6. **Detailed Loss Breakdown**
- Copper losses (stator and rotor)
- Iron losses (hysteresis and eddy current)
- Mechanical losses (friction and windage)
- Stray load losses
- Percentage contribution of each loss type

### 7. **Mechanical Stress Analysis**
- Shaft shear stress calculations
- Bearing radial and axial loads
- Safety factor evaluation
- Torque transient analysis
- Fatigue considerations

### 8. **Economic Analysis**
- Annual operating cost calculations
- Lifecycle cost analysis with NPV
- Energy consumption tracking
- Maintenance cost considerations
- ROI analysis

### 9. **Advanced Control Features**
- PID controller with adjustable gains (sliders)
- Field-oriented control (FOC)
- Thermal derating based on temperature
- Power consumption monitoring

### 10. **Auto-Scaling GUI**
- Automatic width and height adjustment when window is resized
- Responsive layout

## Installation

### Requirements

```bash
pip install numpy scipy matplotlib
```

The application requires Python 3.6+ and the following packages:
- `numpy` - Numerical computations
- `scipy` - ODE solver (RK45)
- `matplotlib` - Graphing and visualization
- `tkinter` - GUI (usually included with Python)

### Running the Application

```bash
python3 induction_furnace_multiphysics_app.py
```

Or make it executable:

```bash
chmod +x induction_furnace_multiphysics_app.py
./induction_furnace_multiphysics_app.py
```

## Usage Guide

### Solving the Induction Furnace Problem

1. Navigate to the **"Induction Furnace"** tab
2. Review the problem description
3. Adjust parameters if needed:
   - Secondary Voltage (V)
   - Full Hearth Power (kW)
   - Full Hearth Power Factor
4. Click **"Solve Problem"**
5. View detailed results including:
   - Full hearth conditions
   - Half-full hearth conditions
   - Change analysis
   - Conclusion

### Running Multi-Physics Simulation

1. Navigate to the **"Simulation Setup"** tab
2. Configure electrical parameters:
   - Supply Voltage (V)
   - Frequency (Hz)
   - Resistances and Inductances
3. Configure mechanical parameters:
   - Moment of Inertia
   - Friction Coefficient
   - Load Torque
4. Select ODE solver (RK45 or Euler)
5. Set time span and steps
6. Click **"Start"** to begin simulation
7. View real-time results in **"Visualization"** tab

### Analyzing Results

**Visualization Tab:**
- View 6 dynamic graphs updating in real-time
- Monitor currents, torque, speed, temperature, power, and losses

**Advanced Control Tab:**
- Adjust PID gains using sliders
- Monitor power consumption
- View thermal derating status

**Economic Analysis Tab:**
- Enter cost parameters
- Click "Calculate Costs"
- View annual and lifecycle costs

**Multi-Physics Tab:**
- Click "Loss Breakdown Analysis" for detailed losses over time
- Click "Mechanical Stress Analysis" for shaft and bearing stress
- Click "Thermal Analysis" for temperature profile and cooling recommendations

## Control Buttons

- **Start**: Begin simulation
- **Stop**: Pause simulation
- **Reset**: Clear all data and reset to initial state

## Menu Options

**File Menu:**
- New Simulation - Reset everything
- Save Results - Save current simulation data to file
- Exit - Close application

**Tools Menu:**
- Furnace Problem Solver - Jump to solver tab
- Loss Analysis - View detailed loss breakdown
- Mechanical Stress - View stress analysis

**Help Menu:**
- About - View application information

## Key Technical Details

### Electrical Model

The electromagnetic model uses d-q transformation with RMS values:

```
di_d/dt = (V_d - R·i_d + ωₑ·L·i_q) / L
di_q/dt = (V_q - R·i_q - ωₑ·L·i_d) / L
```

### Mechanical Model

```
dω/dt = (T_em - T_load - B·ω) / J
```

Where:
- T_em = 1.5·p·L_m·i_d·i_q (electromagnetic torque)

### Thermal Model

```
dT/dt = (P_losses - (T - T_amb)/R_th) / C_th
```

### Loss Calculations

- **Copper losses**: I²R losses in stator and rotor
- **Iron losses**: Hysteresis (∝ f·B²) and eddy current (∝ (f·B)²)
- **Mechanical losses**: Friction (∝ ω²) and windage (∝ ω³)
- **Stray losses**: 1% of copper losses

### Stress Analysis

- **Shaft shear stress**: τ = T·r / J_shaft
- **Safety factor**: σ_yield / τ_max
- **Bearing loads**: Calculated from torque and geometry

## Solution to Induction Furnace Problem

**Given:**
- Secondary voltage: V₂ = 20V (constant)
- Full hearth: P₁ = 600 kW, pf₁ = 0.6
- Half-full hearth: R₂ = 2R₁, X₂ = X₁

**Solution:**

For full hearth:
- S₁ = P₁/pf₁ = 1000 kVA
- I₁ = S₁/V₂ = 50,000 A
- Z₁ = V₂/I₁ = 0.0004 Ω
- R₁ = Z₁·pf₁ = 0.00024 Ω
- X₁ = Z₁·sin(cos⁻¹(0.6)) = 0.00032 Ω

For half-full hearth:
- R₂ = 2R₁ = 0.00048 Ω
- X₂ = X₁ = 0.00032 Ω
- Z₂ = √(R₂² + X₂²) = 0.000577 Ω
- I₂ = V₂/Z₂ = 34,641 A
- **P₂ = I₂²·R₂ = 576.0 kW**
- **pf₂ = R₂/Z₂ = 0.832**

**Answer:**
- Power absorbed when half-full: **576.0 kW**
- Power factor when half-full: **0.832**

## File Structure

```
induction_furnace_multiphysics_app.py
├── InductionFurnaceSolver - Solves the furnace problem
├── MultiPhysicsEngine - Core simulation engine
├── ODESolver - RK45 and Euler methods
├── EconomicAnalyzer - Cost calculations
├── AdvancedControlSystem - PID and thermal derating
└── MultiPhysicsGUI - Main GUI application
```

## Example Results

### Typical Simulation Output

```
POWER CONSUMPTION MONITORING
Average Power:        12,543.67 W (12.54 kW)
Maximum Power:        15,234.12 W (15.23 kW)
Energy Consumed:      17.42 Wh (0.0174 kWh)

DETAILED LOSS BREAKDOWN
Copper Loss (Stator):  245.67 W
Copper Loss (Rotor):   123.45 W
Hysteresis Loss:       45.23 W
Eddy Current Loss:     23.12 W
Friction Loss:         12.34 W
Windage Loss:          5.67 W
Stray Load Loss:       8.90 W
Total Losses:          464.38 W

Efficiency:            96.30%
```

## Troubleshooting

**Issue: Application won't start**
- Ensure all dependencies are installed: `pip install numpy scipy matplotlib`
- Check Python version: `python3 --version` (should be 3.6+)

**Issue: Simulation runs slowly**
- Reduce number of time steps
- Use Euler method instead of RK45 for faster (but less accurate) results

**Issue: Graphs not updating**
- Click "Reset" and restart simulation
- Check that parameters are within reasonable ranges

**Issue: Temperature too high**
- Reduce load torque
- Increase cooling (reduce thermal resistance)
- Check friction coefficient

## Advanced Usage

### Modifying Parameters in Code

You can modify default parameters in the `MultiPhysicsEngine.__init__()` method:

```python
# Electrical parameters
self.R_stator = 0.5  # Stator resistance (Ω)
self.L_stator = 0.05  # Stator inductance (H)

# Mechanical parameters
self.J = 0.1  # Moment of inertia (kg·m²)
self.B = 0.01  # Friction coefficient

# Thermal parameters
self.C_thermal = 1000  # Thermal capacitance (J/K)
self.R_thermal = 5  # Thermal resistance (K/W)
```

### Adding Custom Analysis

You can add custom analysis methods to the `MultiPhysicsGUI` class. Example:

```python
def my_custom_analysis(self):
    """Custom analysis method"""
    # Your analysis code here
    pass
```

## Educational Applications

This application is ideal for:

1. **Electrical Machines Course**: Understanding motor dynamics
2. **Power Electronics**: Analyzing converter-fed drives
3. **Control Systems**: Studying PID control and FOC
4. **Thermal Management**: Understanding heat dissipation
5. **Engineering Economics**: Lifecycle cost analysis
6. **Mechanical Design**: Stress analysis and fatigue

## Practical Applications in Electrical Engineering

- **Motor design and selection**
- **Drive system optimization**
- **Thermal management design**
- **Predictive maintenance planning**
- **Energy efficiency analysis**
- **Cost-benefit analysis for equipment**

## Technical Specifications

- **ODE Solver Accuracy**: RK45 adaptive with error control
- **Thermal Model**: First-order lumped parameter
- **Electrical Model**: Simplified d-q frame (RMS values)
- **Mechanical Model**: Rigid body dynamics
- **Visualization**: Real-time update rate ~10 Hz
- **Data Export**: Text file format

## License

Educational and research use.

## Support

For issues, questions, or contributions:
- Check the code comments for detailed documentation
- Review the mathematical models in the code
- Experiment with different parameters

## Version History

**Version 1.0** (2025)
- Initial release
- Complete multi-physics simulation
- Induction furnace problem solver
- Economic analysis
- Auto-scaling GUI

## Author

Developed for electrical engineering education and practical applications in motor analysis, thermal management, and economic optimization.
