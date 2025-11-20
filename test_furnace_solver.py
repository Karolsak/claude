#!/usr/bin/env python3
"""
Test script for the induction furnace solver
This can run without GUI dependencies
"""

import math

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


def main():
    """Test the solver"""
    print("=" * 80)
    print("INDUCTION FURNACE PROBLEM SOLVER - TEST")
    print("=" * 80)
    print()

    solver = InductionFurnaceSolver()
    results = solver.solve()

    print("FULL HEARTH CONDITIONS:")
    print("-" * 80)
    print(f"Power:                {results['full_hearth']['power']/1000:.2f} kW")
    print(f"Power Factor:         {results['full_hearth']['pf']:.4f}")
    print(f"Current (RMS):        {results['full_hearth']['current']:.2f} A")
    print(f"Impedance:            {results['full_hearth']['impedance']:.6f} Ω")
    print(f"Resistance:           {results['full_hearth']['resistance']:.6f} Ω")
    print(f"Reactance:            {results['full_hearth']['reactance']:.6f} Ω")
    print()

    print("=" * 80)
    print("HALF-FULL HEARTH CONDITIONS:")
    print("-" * 80)
    print(f"Power:                {results['half_hearth']['power']/1000:.2f} kW")
    print(f"Power Factor:         {results['half_hearth']['pf']:.4f}")
    print(f"Current (RMS):        {results['half_hearth']['current']:.2f} A")
    print(f"Impedance:            {results['half_hearth']['impedance']:.6f} Ω")
    print(f"Resistance:           {results['half_hearth']['resistance']:.6f} Ω (doubled)")
    print(f"Reactance:            {results['half_hearth']['reactance']:.6f} Ω (unchanged)")
    print()

    print("=" * 80)
    print("ANALYSIS:")
    print("-" * 80)

    power_change = ((results['half_hearth']['power'] - results['full_hearth']['power']) /
                   results['full_hearth']['power'] * 100)
    print(f"Power Change:         {power_change:.2f}%")

    pf_change = ((results['half_hearth']['pf'] - results['full_hearth']['pf']) /
                results['full_hearth']['pf'] * 100)
    print(f"Power Factor Change:  {pf_change:.2f}%")

    current_change = ((results['half_hearth']['current'] - results['full_hearth']['current']) /
                     results['full_hearth']['current'] * 100)
    print(f"Current Change:       {current_change:.2f}%")
    print()

    print("=" * 80)
    print("CONCLUSION:")
    print("-" * 80)
    print(f"When the hearth is half-full:")
    print(f"  • Power absorbed: {results['half_hearth']['power']/1000:.2f} kW")
    print(f"  • Power factor:   {results['half_hearth']['pf']:.4f}")
    print("=" * 80)
    print()
    print("✓ Test completed successfully!")


if __name__ == "__main__":
    main()
