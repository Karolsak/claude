"""
Synchronous Reluctance Motor Analysis
======================================
Solve for a three-phase, Y-connected synchronous reluctance motor
at a given power angle.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Circle
import matplotlib.patches as mpatches

# Given parameters
V_LL = 380  # Line-to-line voltage (V)
f = 50  # Frequency (Hz)
P_rated = 5.5e3  # Rated power (W)
poles = 4  # Number of poles
R1 = 0.49  # Stator resistance per phase (Ω)
X_sd = 15.5  # d-axis synchronous reactance (Ω)
X_sq = 5.7  # q-axis synchronous reactance (Ω)
P_rot = 110  # Rotational losses (W)
k_str = 0.06  # Stray loss coefficient (Pstr = k_str * Pout)
delta_deg = 21  # Power angle (degrees)

print("=" * 70)
print("SYNCHRONOUS RELUCTANCE MOTOR ANALYSIS")
print("=" * 70)
print("\nGiven Parameters:")
print(f"  Line-to-line voltage: V_LL = {V_LL} V")
print(f"  Frequency: f = {f} Hz")
print(f"  Number of poles: p = {poles}")
print(f"  Rated power: P_rated = {P_rated/1000} kW")
print(f"  Stator resistance: R1 = {R1} Ω")
print(f"  d-axis reactance: X_sd = {X_sd} Ω")
print(f"  q-axis reactance: X_sq = {X_sq} Ω")
print(f"  Rotational losses: P_rot = {P_rot} W")
print(f"  Stray loss coefficient: k_str = {k_str}")
print(f"  Power angle: δ = {delta_deg}°")

# Calculate derived quantities
V_ph = V_LL / np.sqrt(3)  # Phase voltage (V)
omega_s = 2 * np.pi * f  # Synchronous angular frequency (rad/s)
n_s = 120 * f / poles  # Synchronous speed (rpm)
omega_m = 2 * np.pi * n_s / 60  # Mechanical angular velocity (rad/s)
# For motor operation in reluctance motors, the load angle is taken as negative
# (d-axis lags voltage phasor). Given δ as positive, we use -δ for motor mode.
delta_rad = -np.radians(delta_deg)  # Power angle in radians (negative for motor)

print("\n" + "-" * 70)
print("Calculated Base Quantities:")
print(f"  Phase voltage: V_ph = {V_ph:.2f} V")
print(f"  Synchronous speed: n_s = {n_s:.0f} rpm")
print(f"  Mechanical angular velocity: ω_m = {omega_m:.2f} rad/s")
print(f"  Electrical angular frequency: ω_s = {omega_s:.2f} rad/s")

# Solve for d-q axis currents
# In synchronous reluctance motor, d-q reference frame voltage equations:
# For motor operation, the d-axis typically lags the voltage by angle δ
# Using motor convention: d-axis lags voltage phasor by δ
# V_d = V * sin(-δ) = -V * sin(δ)
# V_q = V * cos(-δ) = V * cos(δ)
# But for positive motor torque with given δ, we use:
# V_d = V * sin(δ)
# V_q = V * cos(δ)
# V_d = R1 * I_d - X_sq * I_q
# V_q = R1 * I_q + X_sd * I_d

V_d = V_ph * np.sin(delta_rad)
V_q = V_ph * np.cos(delta_rad)

print("\n" + "-" * 70)
print("d-q Frame Voltage Components:")
print(f"  V_d = V_ph × sin(δ) = {V_d:.2f} V")
print(f"  V_q = V_ph × cos(δ) = {V_q:.2f} V")

# Solving the system of equations:
# V_d = R1 * I_d - X_sq * I_q  ... (1)
# V_q = R1 * I_q + X_sd * I_d  ... (2)
#
# From (1): I_d = (V_d + X_sq * I_q) / R1
# Substitute into (2):
# V_q = R1 * I_q + X_sd * (V_d + X_sq * I_q) / R1
# V_q * R1 = R1^2 * I_q + X_sd * V_d + X_sd * X_sq * I_q
# V_q * R1 - X_sd * V_d = (R1^2 + X_sd * X_sq) * I_q
# I_q = (V_q * R1 - X_sd * V_d) / (R1^2 + X_sd * X_sq)

I_q = (V_q * R1 - X_sd * V_d) / (R1**2 + X_sd * X_sq)
I_d = (V_d + X_sq * I_q) / R1

# Calculate armature current magnitude and angle
I_a = np.sqrt(I_d**2 + I_q**2)
current_angle_rad = np.arctan2(I_q, I_d)
current_angle_deg = np.degrees(current_angle_rad)

print("\n" + "-" * 70)
print("d-q Frame Current Components:")
print(f"  I_d = {I_d:.4f} A")
print(f"  I_q = {I_q:.4f} A")
print(f"\n  Armature current magnitude: I_a = {I_a:.4f} A")
print(f"  Current angle (from d-axis): {current_angle_deg:.2f}°")

# Verify voltage equations
V_d_check = R1 * I_d - X_sq * I_q
V_q_check = R1 * I_q + X_sd * I_d
print(f"\nVerification of voltage equations:")
print(f"  V_d calculated: {V_d:.2f} V, from currents: {V_d_check:.2f} V")
print(f"  V_q calculated: {V_q:.2f} V, from currents: {V_q_check:.2f} V")

# Power calculations
# Three-phase input power
P_in = 3 * (V_d * I_d + V_q * I_q)

# Copper losses
P_cu = 3 * R1 * I_a**2

# Electromagnetic power (air gap power)
P_em = P_in - P_cu

# Alternative calculation of electromagnetic power
P_em_alt = 3 * (X_sd - X_sq) * I_d * I_q
print(f"\nElectromagnetic power (from P_in - P_cu): {P_em:.2f} W")
print(f"Electromagnetic power (from 3*(X_sd-X_sq)*I_d*I_q): {P_em_alt:.2f} W")

# Output power (considering stray losses)
# P_out = P_em - P_rot - P_str
# P_str = k_str * P_out
# P_out = P_em - P_rot - k_str * P_out
# P_out * (1 + k_str) = P_em - P_rot
P_out = (P_em - P_rot) / (1 + k_str)

# Stray losses
P_str = k_str * P_out

# Total losses
P_loss = P_cu + P_rot + P_str

print("\n" + "=" * 70)
print("POWER ANALYSIS")
print("=" * 70)
print(f"  Input power (3-phase):     P_in  = {P_in:.2f} W ({P_in/1000:.3f} kW)")
print(f"  Copper losses:              P_cu  = {P_cu:.2f} W")
print(f"  Electromagnetic power:      P_em  = {P_em:.2f} W ({P_em/1000:.3f} kW)")
print(f"  Rotational losses:          P_rot = {P_rot:.2f} W")
print(f"  Stray losses:               P_str = {P_str:.2f} W")
print(f"  Total losses:               P_loss = {P_loss:.2f} W")
print(f"  Output power:               P_out = {P_out:.2f} W ({P_out/1000:.3f} kW)")

# Shaft torque
T = P_out / omega_m

print("\n" + "=" * 70)
print("TORQUE")
print("=" * 70)
print(f"  Shaft torque: T = {T:.4f} N·m")

# Efficiency
eta = P_out / P_in * 100

print("\n" + "=" * 70)
print("EFFICIENCY")
print("=" * 70)
print(f"  Efficiency: η = {eta:.2f}%")

# Power factor
# The power factor angle is the angle between voltage and current phasors
# Voltage angle from d-axis: arctan(V_q / V_d)
# Current angle from d-axis: arctan(I_q / I_d)
voltage_angle_rad = np.arctan2(V_q, V_d)
phi_rad = voltage_angle_rad - current_angle_rad
cos_phi = np.cos(phi_rad)

# Alternative calculation using power definition
cos_phi_alt = P_in / (3 * V_ph * I_a)

print("\n" + "=" * 70)
print("POWER FACTOR")
print("=" * 70)
print(f"  Voltage angle (from d-axis): {np.degrees(voltage_angle_rad):.2f}°")
print(f"  Current angle (from d-axis): {current_angle_deg:.2f}°")
print(f"  Power factor angle: φ = {np.degrees(phi_rad):.2f}°")
print(f"  Power factor: cos(φ) = {cos_phi:.4f}")
print(f"  Power factor (from power): cos(φ) = {cos_phi_alt:.4f}")

# Summary of results
print("\n" + "=" * 70)
print("SUMMARY OF RESULTS")
print("=" * 70)
print(f"  1. Armature current:        I_a = {I_a:.4f} A")
print(f"  2. Output power:            P_out = {P_out/1000:.4f} kW")
print(f"  3. Shaft torque:            T = {T:.4f} N·m")
print(f"  4. Efficiency:              η = {eta:.2f}%")
print(f"  5. Power factor:            cos(φ) = {cos_phi:.4f}")
print("=" * 70)

# ============================================================================
# VISUALIZATIONS
# ============================================================================

# Create figure with subplots
fig = plt.figure(figsize=(16, 12))
fig.suptitle('Synchronous Reluctance Motor Analysis at δ = 21°',
             fontsize=16, fontweight='bold')

# 1. Phasor Diagram in d-q reference frame
ax1 = plt.subplot(2, 3, 1)
scale_V = 1  # Voltage scale
scale_I = 50  # Current scale (to make currents visible)

# Draw axes
ax1.axhline(y=0, color='k', linewidth=0.5, linestyle='--', alpha=0.3)
ax1.axvline(x=0, color='k', linewidth=0.5, linestyle='--', alpha=0.3)

# d-axis and q-axis labels
ax1.text(250, 10, 'd-axis', fontsize=10, color='gray')
ax1.text(10, 250, 'q-axis', fontsize=10, color='gray')

# Voltage phasor
ax1.arrow(0, 0, V_d * scale_V, V_q * scale_V,
         head_width=10, head_length=8, fc='red', ec='red', linewidth=2, label='V_ph')
ax1.text(V_d * scale_V + 10, V_q * scale_V + 10,
        f'V_ph={V_ph:.1f}V\nδ={delta_deg}°', fontsize=9, color='red')

# Current phasor (scaled)
ax1.arrow(0, 0, I_d * scale_I, I_q * scale_I,
         head_width=10, head_length=8, fc='blue', ec='blue', linewidth=2, label='I_a')
ax1.text(I_d * scale_I + 10, I_q * scale_I + 10,
        f'I_a={I_a:.2f}A\n(scaled ×{scale_I})', fontsize=9, color='blue')

# Voltage components
ax1.plot([0, V_d], [0, 0], 'r--', linewidth=1, alpha=0.5)
ax1.plot([V_d, V_d], [0, V_q], 'r--', linewidth=1, alpha=0.5)
ax1.text(V_d/2, -15, f'V_d={V_d:.1f}V', fontsize=8, ha='center', color='red')
ax1.text(V_d + 15, V_q/2, f'V_q={V_q:.1f}V', fontsize=8, color='red')

# Current components
ax1.plot([0, I_d * scale_I], [0, 0], 'b--', linewidth=1, alpha=0.5)
ax1.plot([I_d * scale_I, I_d * scale_I], [0, I_q * scale_I], 'b--', linewidth=1, alpha=0.5)
ax1.text(I_d * scale_I/2, -25, f'I_d={I_d:.2f}A', fontsize=8, ha='center', color='blue')
ax1.text(I_d * scale_I + 20, I_q * scale_I/2, f'I_q={I_q:.2f}A', fontsize=8, color='blue')

# Power factor angle
from matplotlib.patches import Arc
angle_arc = Arc((0, 0), 60, 60, angle=0, theta1=current_angle_deg,
               theta2=delta_deg, color='green', linewidth=2)
ax1.add_patch(angle_arc)
ax1.text(35, 15, f'φ={np.degrees(phi_rad):.1f}°', fontsize=9, color='green')

ax1.set_xlim(-50, 300)
ax1.set_ylim(-50, 250)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.set_xlabel('d-axis', fontsize=10)
ax1.set_ylabel('q-axis', fontsize=10)
ax1.set_title('Phasor Diagram (d-q Frame)', fontsize=11, fontweight='bold')
ax1.legend(loc='upper left')

# 2. Power Flow Diagram
ax2 = plt.subplot(2, 3, 2)
ax2.axis('off')

# Power flow boxes
box_props = dict(boxstyle='round,pad=0.5', facecolor='lightblue', edgecolor='black', linewidth=2)
loss_props = dict(boxstyle='round,pad=0.3', facecolor='lightcoral', edgecolor='black', linewidth=1)

# Input power
ax2.text(0.1, 0.85, f'Input Power\nP_in = {P_in/1000:.3f} kW',
        ha='center', va='center', fontsize=10, bbox=box_props, transform=ax2.transAxes)

# Copper losses
ax2.text(0.5, 0.95, f'Copper Losses\nP_cu = {P_cu:.1f} W',
        ha='center', va='center', fontsize=9, bbox=loss_props, transform=ax2.transAxes)

# Electromagnetic power
ax2.text(0.5, 0.7, f'Air Gap Power\nP_em = {P_em/1000:.3f} kW',
        ha='center', va='center', fontsize=10, bbox=box_props, transform=ax2.transAxes)

# Rotational losses
ax2.text(0.25, 0.5, f'Rotational\nLosses\nP_rot = {P_rot:.1f} W',
        ha='center', va='center', fontsize=8, bbox=loss_props, transform=ax2.transAxes)

# Stray losses
ax2.text(0.75, 0.5, f'Stray\nLosses\nP_str = {P_str:.1f} W',
        ha='center', va='center', fontsize=8, bbox=loss_props, transform=ax2.transAxes)

# Output power
ax2.text(0.5, 0.25, f'Output Power\nP_out = {P_out/1000:.3f} kW',
        ha='center', va='center', fontsize=10, bbox=box_props, transform=ax2.transAxes)

# Efficiency
ax2.text(0.5, 0.05, f'Efficiency: η = {eta:.2f}%',
        ha='center', va='center', fontsize=11, fontweight='bold', transform=ax2.transAxes)

# Arrows
arrow_props = dict(arrowstyle='->', lw=2, color='black')
ax2.annotate('', xy=(0.5, 0.82), xytext=(0.1, 0.82),
            arrowprops=arrow_props, transform=ax2.transAxes)
ax2.annotate('', xy=(0.5, 0.62), xytext=(0.5, 0.78),
            arrowprops=arrow_props, transform=ax2.transAxes)
ax2.annotate('', xy=(0.5, 0.35), xytext=(0.5, 0.58),
            arrowprops=arrow_props, transform=ax2.transAxes)

ax2.set_title('Power Flow Diagram', fontsize=11, fontweight='bold')

# 3. Torque vs Power Angle Characteristic
ax3 = plt.subplot(2, 3, 3)
delta_range = np.linspace(0, 90, 200)
delta_range_rad = np.radians(delta_range)

# For each angle, calculate torque
T_range = []
for delta_i in delta_range_rad:
    V_d_i = V_ph * np.sin(delta_i)
    V_q_i = V_ph * np.cos(delta_i)

    I_q_i = (V_q_i * R1 - X_sd * V_d_i) / (R1**2 + X_sd * X_sq)
    I_d_i = (V_d_i + X_sq * I_q_i) / R1

    P_em_i = 3 * (X_sd - X_sq) * I_d_i * I_q_i
    P_out_i = (P_em_i - P_rot) / (1 + k_str)
    T_i = P_out_i / omega_m
    T_range.append(T_i)

ax3.plot(delta_range, T_range, 'b-', linewidth=2, label='T(δ)')
ax3.plot(delta_deg, T, 'ro', markersize=10, label=f'Operating point (δ={delta_deg}°)')
ax3.axvline(x=delta_deg, color='r', linestyle='--', alpha=0.3)
ax3.axhline(y=T, color='r', linestyle='--', alpha=0.3)
ax3.grid(True, alpha=0.3)
ax3.set_xlabel('Power Angle δ (degrees)', fontsize=10)
ax3.set_ylabel('Shaft Torque (N·m)', fontsize=10)
ax3.set_title('Torque vs Power Angle', fontsize=11, fontweight='bold')
ax3.legend()
ax3.set_xlim(0, 90)

# 4. Power vs Power Angle
ax4 = plt.subplot(2, 3, 4)
P_out_range = []
P_in_range = []
for delta_i in delta_range_rad:
    V_d_i = V_ph * np.sin(delta_i)
    V_q_i = V_ph * np.cos(delta_i)

    I_q_i = (V_q_i * R1 - X_sd * V_d_i) / (R1**2 + X_sd * X_sq)
    I_d_i = (V_d_i + X_sq * I_q_i) / R1

    P_in_i = 3 * (V_d_i * I_d_i + V_q_i * I_q_i)
    P_em_i = 3 * (X_sd - X_sq) * I_d_i * I_q_i
    P_out_i = (P_em_i - P_rot) / (1 + k_str)

    P_out_range.append(P_out_i / 1000)
    P_in_range.append(P_in_i / 1000)

ax4.plot(delta_range, P_in_range, 'r-', linewidth=2, label='Input Power P_in')
ax4.plot(delta_range, P_out_range, 'b-', linewidth=2, label='Output Power P_out')
ax4.plot(delta_deg, P_in/1000, 'ro', markersize=8)
ax4.plot(delta_deg, P_out/1000, 'bo', markersize=8)
ax4.axvline(x=delta_deg, color='gray', linestyle='--', alpha=0.3)
ax4.grid(True, alpha=0.3)
ax4.set_xlabel('Power Angle δ (degrees)', fontsize=10)
ax4.set_ylabel('Power (kW)', fontsize=10)
ax4.set_title('Power vs Power Angle', fontsize=11, fontweight='bold')
ax4.legend()
ax4.set_xlim(0, 90)

# 5. Efficiency and Power Factor vs Power Angle
ax5 = plt.subplot(2, 3, 5)
eta_range = []
pf_range = []
for delta_i in delta_range_rad:
    V_d_i = V_ph * np.sin(delta_i)
    V_q_i = V_ph * np.cos(delta_i)

    I_q_i = (V_q_i * R1 - X_sd * V_d_i) / (R1**2 + X_sd * X_sq)
    I_d_i = (V_d_i + X_sq * I_q_i) / R1
    I_a_i = np.sqrt(I_d_i**2 + I_q_i**2)

    P_in_i = 3 * (V_d_i * I_d_i + V_q_i * I_q_i)
    P_em_i = 3 * (X_sd - X_sq) * I_d_i * I_q_i
    P_out_i = (P_em_i - P_rot) / (1 + k_str)

    eta_i = P_out_i / P_in_i * 100 if P_in_i > 0 else 0
    pf_i = P_in_i / (3 * V_ph * I_a_i) if I_a_i > 0 else 0

    eta_range.append(eta_i)
    pf_range.append(pf_i)

ax5_twin = ax5.twinx()
line1 = ax5.plot(delta_range, eta_range, 'g-', linewidth=2, label='Efficiency η')
line2 = ax5_twin.plot(delta_range, pf_range, 'm-', linewidth=2, label='Power Factor cos(φ)')
ax5.plot(delta_deg, eta, 'go', markersize=8)
ax5_twin.plot(delta_deg, cos_phi, 'mo', markersize=8)
ax5.axvline(x=delta_deg, color='gray', linestyle='--', alpha=0.3)
ax5.grid(True, alpha=0.3)
ax5.set_xlabel('Power Angle δ (degrees)', fontsize=10)
ax5.set_ylabel('Efficiency (%)', fontsize=10, color='g')
ax5_twin.set_ylabel('Power Factor', fontsize=10, color='m')
ax5.tick_params(axis='y', labelcolor='g')
ax5_twin.tick_params(axis='y', labelcolor='m')
ax5.set_title('Efficiency and Power Factor vs Power Angle', fontsize=11, fontweight='bold')
lines = line1 + line2
labels = [l.get_label() for l in lines]
ax5.legend(lines, labels, loc='lower right')
ax5.set_xlim(0, 90)

# 6. Current Components vs Power Angle
ax6 = plt.subplot(2, 3, 6)
I_d_range = []
I_q_range = []
I_a_range = []
for delta_i in delta_range_rad:
    V_d_i = V_ph * np.sin(delta_i)
    V_q_i = V_ph * np.cos(delta_i)

    I_q_i = (V_q_i * R1 - X_sd * V_d_i) / (R1**2 + X_sd * X_sq)
    I_d_i = (V_d_i + X_sq * I_q_i) / R1
    I_a_i = np.sqrt(I_d_i**2 + I_q_i**2)

    I_d_range.append(I_d_i)
    I_q_range.append(I_q_i)
    I_a_range.append(I_a_i)

ax6.plot(delta_range, I_d_range, 'r-', linewidth=2, label='I_d')
ax6.plot(delta_range, I_q_range, 'b-', linewidth=2, label='I_q')
ax6.plot(delta_range, I_a_range, 'k--', linewidth=2, label='I_a (magnitude)')
ax6.plot(delta_deg, I_d, 'ro', markersize=8)
ax6.plot(delta_deg, I_q, 'bo', markersize=8)
ax6.plot(delta_deg, I_a, 'ko', markersize=8)
ax6.axvline(x=delta_deg, color='gray', linestyle='--', alpha=0.3)
ax6.grid(True, alpha=0.3)
ax6.set_xlabel('Power Angle δ (degrees)', fontsize=10)
ax6.set_ylabel('Current (A)', fontsize=10)
ax6.set_title('Current Components vs Power Angle', fontsize=11, fontweight='bold')
ax6.legend()
ax6.set_xlim(0, 90)

plt.tight_layout()
plt.savefig('/home/user/claude/reluctance_motor_analysis.png', dpi=300, bbox_inches='tight')
print(f"\nPlot saved as 'reluctance_motor_analysis.png'")
plt.show()

print("\nAnalysis complete!")
