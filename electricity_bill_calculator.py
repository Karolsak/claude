"""
Example 50.20: Electricity Bill Calculator with Load Factor Analysis

This program calculates electricity bills based on:
- Maximum demand charge (Rs. per kW)
- Energy consumption charge (paisa per kWh)
- Load factor variations
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pandas as pd

# Constants
HOURS_PER_YEAR = 8760  # 365 days × 24 hours

class ElectricityBillCalculator:
    def __init__(self, demand_charge_per_kw, energy_charge_per_kwh):
        """
        Initialize the calculator with tariff rates.

        Parameters:
        - demand_charge_per_kw: Charge per kW of maximum demand (Rs.)
        - energy_charge_per_kwh: Charge per kWh of energy consumed (Rs.)
        """
        self.demand_charge_per_kw = demand_charge_per_kw
        self.energy_charge_per_kwh = energy_charge_per_kwh

    def calculate_maximum_demand(self, annual_consumption_kwh, load_factor):
        """
        Calculate maximum demand from annual consumption and load factor.

        Load Factor = Average Demand / Maximum Demand
        Average Demand = Annual Consumption / Hours per year

        Therefore:
        Maximum Demand = Average Demand / Load Factor
        """
        average_demand = annual_consumption_kwh / HOURS_PER_YEAR
        maximum_demand = average_demand / load_factor
        return maximum_demand

    def calculate_bill(self, annual_consumption_kwh, load_factor):
        """
        Calculate the total annual bill and cost per kWh.

        Returns:
        - Dictionary containing all billing components
        """
        # Calculate maximum demand
        average_demand = annual_consumption_kwh / HOURS_PER_YEAR
        maximum_demand = self.calculate_maximum_demand(annual_consumption_kwh, load_factor)

        # Calculate charges
        demand_charge = maximum_demand * self.demand_charge_per_kw
        energy_charge = annual_consumption_kwh * self.energy_charge_per_kwh
        total_bill = demand_charge + energy_charge

        # Calculate overall cost per kWh
        cost_per_kwh = total_bill / annual_consumption_kwh

        return {
            'annual_consumption_kwh': annual_consumption_kwh,
            'load_factor': load_factor,
            'average_demand_kw': average_demand,
            'maximum_demand_kw': maximum_demand,
            'demand_charge_rs': demand_charge,
            'energy_charge_rs': energy_charge,
            'total_bill_rs': total_bill,
            'cost_per_kwh_rs': cost_per_kwh
        }

    def print_bill_details(self, result, scenario_name):
        """Print detailed bill breakdown."""
        print(f"\n{'='*70}")
        print(f"{scenario_name}")
        print(f"{'='*70}")
        print(f"Annual Consumption:        {result['annual_consumption_kwh']:>15,.2f} kWh")
        print(f"Load Factor:               {result['load_factor']:>15.2%}")
        print(f"Average Demand:            {result['average_demand_kw']:>15,.2f} kW")
        print(f"Maximum Demand:            {result['maximum_demand_kw']:>15,.2f} kW")
        print(f"{'-'*70}")
        print(f"Demand Charge:             Rs. {result['demand_charge_rs']:>12,.2f}")
        print(f"Energy Charge:             Rs. {result['energy_charge_rs']:>12,.2f}")
        print(f"{'-'*70}")
        print(f"Total Annual Bill:         Rs. {result['total_bill_rs']:>12,.2f}")
        print(f"Overall Cost per kWh:      Rs. {result['cost_per_kwh_rs']:>12,.4f}")
        print(f"{'='*70}")


def create_visualizations(results_dict):
    """Create comprehensive visualizations for all scenarios."""

    # Extract data for plotting
    scenarios = list(results_dict.keys())

    # Create figure with multiple subplots
    fig = plt.figure(figsize=(16, 12))

    # 1. Cost Breakdown - Stacked Bar Chart
    ax1 = plt.subplot(2, 3, 1)
    demand_charges = [results_dict[s]['demand_charge_rs'] for s in scenarios]
    energy_charges = [results_dict[s]['energy_charge_rs'] for s in scenarios]

    x_pos = np.arange(len(scenarios))
    width = 0.6

    bars1 = ax1.bar(x_pos, demand_charges, width, label='Demand Charge',
                    color='#FF6B6B', alpha=0.8, edgecolor='black', linewidth=1.5)
    bars2 = ax1.bar(x_pos, energy_charges, width, bottom=demand_charges,
                    label='Energy Charge', color='#4ECDC4', alpha=0.8,
                    edgecolor='black', linewidth=1.5)

    ax1.set_xlabel('Scenario', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Cost (Rs.)', fontsize=11, fontweight='bold')
    ax1.set_title('Cost Breakdown by Scenario', fontsize=13, fontweight='bold', pad=15)
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels([f'({s})' for s in scenarios], fontsize=10)
    ax1.legend(fontsize=10, loc='upper left')
    ax1.grid(axis='y', alpha=0.3, linestyle='--')

    # Add value labels on bars
    for i, (d, e) in enumerate(zip(demand_charges, energy_charges)):
        ax1.text(i, d/2, f'Rs. {d:,.0f}', ha='center', va='center',
                fontweight='bold', fontsize=9, color='white')
        ax1.text(i, d + e/2, f'Rs. {e:,.0f}', ha='center', va='center',
                fontweight='bold', fontsize=9, color='white')

    # 2. Total Bill Comparison
    ax2 = plt.subplot(2, 3, 2)
    total_bills = [results_dict[s]['total_bill_rs'] for s in scenarios]
    colors = ['#FF6B6B', '#95E1D3', '#F38181']

    bars = ax2.bar(x_pos, total_bills, width, color=colors, alpha=0.8,
                   edgecolor='black', linewidth=1.5)

    ax2.set_xlabel('Scenario', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Total Annual Bill (Rs.)', fontsize=11, fontweight='bold')
    ax2.set_title('Total Annual Bill Comparison', fontsize=13, fontweight='bold', pad=15)
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels([f'({s})' for s in scenarios], fontsize=10)
    ax2.grid(axis='y', alpha=0.3, linestyle='--')

    # Add value labels
    for i, (bar, bill) in enumerate(zip(bars, total_bills)):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'Rs. {bill:,.2f}', ha='center', va='bottom',
                fontweight='bold', fontsize=9)

    # 3. Cost per kWh Comparison
    ax3 = plt.subplot(2, 3, 3)
    cost_per_kwh = [results_dict[s]['cost_per_kwh_rs'] for s in scenarios]

    bars = ax3.bar(x_pos, cost_per_kwh, width, color=colors, alpha=0.8,
                   edgecolor='black', linewidth=1.5)

    ax3.set_xlabel('Scenario', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Cost per kWh (Rs.)', fontsize=11, fontweight='bold')
    ax3.set_title('Overall Cost per kWh Comparison', fontsize=13, fontweight='bold', pad=15)
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels([f'({s})' for s in scenarios], fontsize=10)
    ax3.grid(axis='y', alpha=0.3, linestyle='--')

    # Add value labels
    for bar, cost in zip(bars, cost_per_kwh):
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'Rs. {cost:.4f}', ha='center', va='bottom',
                fontweight='bold', fontsize=9)

    # 4. Load Factor vs Maximum Demand
    ax4 = plt.subplot(2, 3, 4)
    load_factors = [results_dict[s]['load_factor'] * 100 for s in scenarios]
    max_demands = [results_dict[s]['maximum_demand_kw'] for s in scenarios]

    ax4.scatter(load_factors, max_demands, s=300, c=colors, alpha=0.8,
               edgecolor='black', linewidth=2, zorder=3)

    for i, (lf, md, s) in enumerate(zip(load_factors, max_demands, scenarios)):
        ax4.annotate(f'({s})', (lf, md), fontsize=10, fontweight='bold',
                    xytext=(10, 10), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor=colors[i], alpha=0.3))

    ax4.set_xlabel('Load Factor (%)', fontsize=11, fontweight='bold')
    ax4.set_ylabel('Maximum Demand (kW)', fontsize=11, fontweight='bold')
    ax4.set_title('Load Factor vs Maximum Demand', fontsize=13, fontweight='bold', pad=15)
    ax4.grid(True, alpha=0.3, linestyle='--')

    # 5. Consumption Comparison
    ax5 = plt.subplot(2, 3, 5)
    consumptions = [results_dict[s]['annual_consumption_kwh'] for s in scenarios]

    bars = ax5.barh(x_pos, consumptions, width, color=colors, alpha=0.8,
                    edgecolor='black', linewidth=1.5)

    ax5.set_ylabel('Scenario', fontsize=11, fontweight='bold')
    ax5.set_xlabel('Annual Consumption (kWh)', fontsize=11, fontweight='bold')
    ax5.set_title('Annual Consumption Comparison', fontsize=13, fontweight='bold', pad=15)
    ax5.set_yticks(x_pos)
    ax5.set_yticklabels([f'({s})' for s in scenarios], fontsize=10)
    ax5.grid(axis='x', alpha=0.3, linestyle='--')

    # Add value labels
    for bar, consumption in zip(bars, consumptions):
        width_val = bar.get_width()
        ax5.text(width_val, bar.get_y() + bar.get_height()/2.,
                f'{consumption:,.0f} kWh', ha='left', va='center',
                fontweight='bold', fontsize=9, bbox=dict(boxstyle='round,pad=0.3',
                facecolor='white', alpha=0.8))

    # 6. Load Factor Impact Analysis
    ax6 = plt.subplot(2, 3, 6)

    # Create a curve showing how cost per kWh varies with load factor
    # Using the consumption from scenario (i)
    base_consumption = results_dict['i']['annual_consumption_kwh']
    test_load_factors = np.linspace(0.2, 0.6, 50)

    calculator = ElectricityBillCalculator(120, 0.04)
    test_costs = []

    for lf in test_load_factors:
        result = calculator.calculate_bill(base_consumption, lf)
        test_costs.append(result['cost_per_kwh_rs'])

    ax6.plot(test_load_factors * 100, test_costs, linewidth=3,
            color='#4A90E2', label='Cost Curve', zorder=2)

    # Mark the actual scenarios on the curve
    for i, s in enumerate(scenarios):
        lf = results_dict[s]['load_factor'] * 100
        cost = results_dict[s]['cost_per_kwh_rs']
        if results_dict[s]['annual_consumption_kwh'] == base_consumption:
            ax6.scatter(lf, cost, s=300, c=colors[i], alpha=0.9,
                       edgecolor='black', linewidth=2, zorder=3)
            ax6.annotate(f'({s})', (lf, cost), fontsize=10, fontweight='bold',
                        xytext=(10, -15), textcoords='offset points',
                        bbox=dict(boxstyle='round,pad=0.5', facecolor=colors[i], alpha=0.3),
                        arrowprops=dict(arrowstyle='->', lw=1.5))

    ax6.set_xlabel('Load Factor (%)', fontsize=11, fontweight='bold')
    ax6.set_ylabel('Cost per kWh (Rs.)', fontsize=11, fontweight='bold')
    ax6.set_title('Load Factor Impact on Cost per kWh', fontsize=13, fontweight='bold', pad=15)
    ax6.grid(True, alpha=0.3, linestyle='--')
    ax6.legend(fontsize=10)

    plt.suptitle('Electricity Bill Analysis - Example 50.20',
                fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout(rect=[0, 0, 1, 0.99])

    return fig


def create_summary_table(results_dict):
    """Create a summary comparison table."""

    data = []
    for scenario, result in results_dict.items():
        data.append({
            'Scenario': f'({scenario})',
            'Consumption (kWh)': f"{result['annual_consumption_kwh']:,.0f}",
            'Load Factor': f"{result['load_factor']:.1%}",
            'Max Demand (kW)': f"{result['maximum_demand_kw']:.2f}",
            'Demand Charge (Rs.)': f"{result['demand_charge_rs']:,.2f}",
            'Energy Charge (Rs.)': f"{result['energy_charge_rs']:,.2f}",
            'Total Bill (Rs.)': f"{result['total_bill_rs']:,.2f}",
            'Cost/kWh (Rs.)': f"{result['cost_per_kwh_rs']:.4f}"
        })

    df = pd.DataFrame(data)

    # Create table visualization
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.axis('tight')
    ax.axis('off')

    table = ax.table(cellText=df.values, colLabels=df.columns,
                    cellLoc='center', loc='center',
                    colWidths=[0.08, 0.13, 0.1, 0.12, 0.14, 0.14, 0.13, 0.12])

    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2.5)

    # Style header
    for i in range(len(df.columns)):
        cell = table[(0, i)]
        cell.set_facecolor('#4A90E2')
        cell.set_text_props(weight='bold', color='white')

    # Style rows with alternating colors
    colors = ['#FF6B6B', '#95E1D3', '#F38181']
    for i in range(len(df)):
        for j in range(len(df.columns)):
            cell = table[(i+1, j)]
            cell.set_facecolor(colors[i])
            cell.set_alpha(0.3)
            cell.set_edgecolor('black')
            cell.set_linewidth(1.5)

    plt.title('Summary Comparison Table', fontsize=14, fontweight='bold', pad=20)
    plt.tight_layout()

    return fig, df


def main():
    """Main execution function."""

    print("\n" + "="*70)
    print(" "*15 + "ELECTRICITY BILL CALCULATOR")
    print(" "*20 + "Example 50.20")
    print("="*70)

    # Given data
    annual_consumption_base = 176400  # kWh
    demand_charge = 120  # Rs. per kW
    energy_charge = 0.04  # Rs. per kWh (4 paisa = 0.04 rupees)

    print(f"\nGiven Information:")
    print(f"  • Annual Consumption: {annual_consumption_base:,} kWh")
    print(f"  • Demand Charge: Rs. {demand_charge} per kW of maximum demand")
    print(f"  • Energy Charge: {energy_charge} Rs. per kWh (4 paisa per kWh)")

    # Initialize calculator
    calculator = ElectricityBillCalculator(demand_charge, energy_charge)

    # Scenario (i): Base case with 36% load factor
    print("\n" + "="*70)
    print("SCENARIO (i): Base Case")
    print("="*70)
    result_i = calculator.calculate_bill(annual_consumption_base, 0.36)
    calculator.print_bill_details(result_i, "Scenario (i): Load Factor = 36%")

    # Scenario (ii): 25% reduction in consumption, same load factor
    print("\n" + "="*70)
    print("SCENARIO (ii): Consumption Reduced by 25%")
    print("="*70)
    reduced_consumption = annual_consumption_base * 0.75
    result_ii = calculator.calculate_bill(reduced_consumption, 0.36)
    calculator.print_bill_details(result_ii,
                                 "Scenario (ii): Consumption Reduced by 25%, Load Factor = 36%")

    # Scenario (iii): Same consumption as (i), but load factor = 27%
    print("\n" + "="*70)
    print("SCENARIO (iii): Load Factor Changed to 27%")
    print("="*70)
    result_iii = calculator.calculate_bill(annual_consumption_base, 0.27)
    calculator.print_bill_details(result_iii,
                                 "Scenario (iii): Same Consumption, Load Factor = 27%")

    # Store all results
    results = {
        'i': result_i,
        'ii': result_ii,
        'iii': result_iii
    }

    # Analysis Section
    print("\n" + "="*70)
    print(" "*25 + "KEY INSIGHTS")
    print("="*70)

    # Cost comparison
    print(f"\n1. COST PER kWh COMPARISON:")
    print(f"   Scenario (i):   Rs. {result_i['cost_per_kwh_rs']:.4f}/kWh")
    print(f"   Scenario (ii):  Rs. {result_ii['cost_per_kwh_rs']:.4f}/kWh")
    print(f"   Scenario (iii): Rs. {result_iii['cost_per_kwh_rs']:.4f}/kWh")

    # Savings analysis
    savings_ii = result_i['total_bill_rs'] - result_ii['total_bill_rs']
    savings_percent_ii = (savings_ii / result_i['total_bill_rs']) * 100

    print(f"\n2. CONSUMPTION REDUCTION IMPACT (Scenario ii):")
    print(f"   • Consumption reduced by: 25%")
    print(f"   • Bill reduced by: Rs. {savings_ii:,.2f} ({savings_percent_ii:.2f}%)")
    print(f"   • However, cost per kWh INCREASED by: Rs. {result_ii['cost_per_kwh_rs'] - result_i['cost_per_kwh_rs']:.4f}")
    print(f"   • Reason: Fixed demand charge spread over fewer kWh")

    # Load factor impact
    extra_cost_iii = result_iii['total_bill_rs'] - result_i['total_bill_rs']

    print(f"\n3. LOAD FACTOR IMPACT (Scenario iii):")
    print(f"   • Load factor decreased from 36% to 27%")
    print(f"   • Maximum demand increased from {result_i['maximum_demand_kw']:.2f} kW to {result_iii['maximum_demand_kw']:.2f} kW")
    print(f"   • Additional cost: Rs. {extra_cost_iii:,.2f}")
    print(f"   • Cost per kWh increased by: Rs. {result_iii['cost_per_kwh_rs'] - result_i['cost_per_kwh_rs']:.4f}")
    print(f"   • Reason: Higher maximum demand charge with same energy consumption")

    print(f"\n4. KEY TAKEAWAY:")
    print(f"   Higher load factor = Better utilization = Lower cost per kWh")
    print(f"   A higher load factor means more efficient use of the maximum demand,")
    print(f"   spreading the fixed demand charge over more kWh of energy consumed.")

    # Create visualizations
    print("\n" + "="*70)
    print("Generating visualizations...")
    print("="*70)

    fig1 = create_visualizations(results)
    plt.savefig('/home/user/claude/electricity_bill_analysis.png', dpi=300, bbox_inches='tight')
    print("✓ Saved: electricity_bill_analysis.png")

    fig2, df = create_summary_table(results)
    plt.savefig('/home/user/claude/electricity_bill_summary_table.png', dpi=300, bbox_inches='tight')
    print("✓ Saved: electricity_bill_summary_table.png")

    # Save summary to CSV
    df.to_csv('/home/user/claude/electricity_bill_summary.csv', index=False)
    print("✓ Saved: electricity_bill_summary.csv")

    print("\n" + "="*70)
    print(" "*20 + "ANALYSIS COMPLETE!")
    print("="*70)

    plt.show()


if __name__ == "__main__":
    main()
