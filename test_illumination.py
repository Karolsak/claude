#!/usr/bin/env python3
"""
Quick test script to verify illumination calculations
"""

import math

def test_illumination():
    """Test the illumination problem with given parameters"""
    print("=" * 70)
    print("ILLUMINATION CALCULATION TEST")
    print("=" * 70)
    
    # Given parameters
    candle_power = 300  # cd
    height = 20  # m
    disc_diameter = 20  # m
    radius = disc_diameter / 2
    
    print(f"\nInput Parameters:")
    print(f"  Candle Power: {candle_power} cd")
    print(f"  Height: {height} m")
    print(f"  Disc Diameter: {disc_diameter} m")
    print(f"  Disc Radius: {radius} m")
    
    # WITHOUT REFLECTOR
    print("\n" + "=" * 70)
    print("WITHOUT REFLECTOR (Direct Light Only)")
    print("=" * 70)
    
    # At center
    center_without = candle_power * math.cos(0) / (height ** 2)
    print(f"\nAt Center:")
    print(f"  Distance: {height} m")
    print(f"  Angle: 0° (directly below)")
    print(f"  Illumination: {center_without:.4f} lux")
    
    # At edge
    d_edge = math.sqrt(height ** 2 + radius ** 2)
    cos_theta = height / d_edge
    theta_deg = math.degrees(math.acos(cos_theta))
    edge_without = candle_power * cos_theta / (d_edge ** 2)
    print(f"\nAt Edge:")
    print(f"  Distance: {d_edge:.3f} m")
    print(f"  Angle: {theta_deg:.2f}°")
    print(f"  cos(θ): {cos_theta:.4f}")
    print(f"  Illumination: {edge_without:.4f} lux")
    
    # WITH REFLECTOR
    print("\n" + "=" * 70)
    print("WITH REFLECTOR (50% of Total Light Redirected Uniformly)")
    print("=" * 70)
    
    # Calculate uniform reflected component
    total_flux = 4 * math.pi * candle_power  # Total flux from lamp
    reflected_flux = 0.5 * total_flux  # 50% redirected
    disc_area = math.pi * radius ** 2
    uniform_illumination = reflected_flux / disc_area
    
    print(f"\nReflector Analysis:")
    print(f"  Total Luminous Flux: {total_flux:.2f} lumens")
    print(f"  Reflected Flux (50%): {reflected_flux:.2f} lumens")
    print(f"  Disc Area: {disc_area:.2f} m²")
    print(f"  Uniform Additional Illumination: {uniform_illumination:.4f} lux")
    
    # Total illumination with reflector
    center_with = center_without + uniform_illumination
    edge_with = edge_without + uniform_illumination
    
    print(f"\nAt Center (with reflector):")
    print(f"  Direct: {center_without:.4f} lux")
    print(f"  Reflected: {uniform_illumination:.4f} lux")
    print(f"  Total: {center_with:.4f} lux")
    print(f"  Improvement Factor: {center_with/center_without:.2f}x")
    
    print(f"\nAt Edge (with reflector):")
    print(f"  Direct: {edge_without:.4f} lux")
    print(f"  Reflected: {uniform_illumination:.4f} lux")
    print(f"  Total: {edge_with:.4f} lux")
    print(f"  Improvement Factor: {edge_with/edge_without:.2f}x")
    
    # Summary comparison
    print("\n" + "=" * 70)
    print("SUMMARY COMPARISON")
    print("=" * 70)
    
    print(f"\n{'Location':<20} {'Without Reflector':<20} {'With Reflector':<20} {'Improvement':<15}")
    print("-" * 70)
    print(f"{'Center':<20} {center_without:<20.4f} {center_with:<20.4f} {center_with/center_without:<15.2f}x")
    print(f"{'Edge':<20} {edge_without:<20.4f} {edge_with:<20.4f} {edge_with/edge_without:<15.2f}x")
    
    print("\n" + "=" * 70)
    print("TEST COMPLETED SUCCESSFULLY")
    print("=" * 70)

if __name__ == "__main__":
    test_illumination()
