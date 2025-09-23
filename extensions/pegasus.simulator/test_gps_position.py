"""
Test GPS position calculation to verify North/East movements work correctly.
Run this in Script Editor to test the GPS position fix.

FIXED: Now using direct FLU→ENU transformation instead of FLU→NED→ENU
"""

print("\n" + "="*60)
print("GPS POSITION TRANSFORMATION TEST")
print("="*60)

try:
    import numpy as np
    from pegasus.simulator.logic.sensors.calculations.gps_calc import calculate_gps_measurements

    # San Diego coordinates
    origin_lat = 32.7746
    origin_lon = -117.079
    origin_alt = 90.0

    print("\nOrigin coordinates:")
    print(f"  Lat: {origin_lat}°")
    print(f"  Lon: {origin_lon}°")
    print(f"  Alt: {origin_alt} m")

    # Test 1: Vehicle at origin
    print("\n1. Vehicle at origin (0,0,0):")
    result = calculate_gps_measurements(
        position_flu=np.zeros(3),
        linear_velocity_flu=np.zeros(3),
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        origin_alt=origin_alt
    )
    print(f"   Lat: {result['latitude']:.6f}° (should be ~{origin_lat:.6f}°)")
    print(f"   Lon: {result['longitude']:.6f}° (should be ~{origin_lon:.6f}°)")

    # Test 2: Move 100m North (positive X in FLU)
    print("\n2. Move 100m North (X=100 in FLU):")
    result = calculate_gps_measurements(
        position_flu=np.array([100, 0, 0]),  # 100m North
        linear_velocity_flu=np.zeros(3),
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        origin_alt=origin_alt
    )
    lat_change = result['latitude'] - origin_lat
    lon_change = result['longitude'] - origin_lon
    print(f"   Lat: {result['latitude']:.6f}° (Δ = {lat_change:.6f}°)")
    print(f"   Lon: {result['longitude']:.6f}° (Δ = {lon_change:.6f}°)")

    if lat_change > 0 and abs(lon_change) < abs(lat_change) * 0.1:
        print("   ✓ Latitude increased, Longitude ~unchanged (CORRECT!)")
    else:
        print("   ✗ WRONG! North movement should increase latitude!")

    # Test 3: Move 100m East (negative Y in FLU, since Y is Left)
    print("\n3. Move 100m East (Y=-100 in FLU):")
    result = calculate_gps_measurements(
        position_flu=np.array([0, -100, 0]),  # 100m East (Right)
        linear_velocity_flu=np.zeros(3),
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        origin_alt=origin_alt
    )
    lat_change = result['latitude'] - origin_lat
    lon_change = result['longitude'] - origin_lon
    print(f"   Lat: {result['latitude']:.6f}° (Δ = {lat_change:.6f}°)")
    print(f"   Lon: {result['longitude']:.6f}° (Δ = {lon_change:.6f}°)")

    if lon_change > 0 and abs(lat_change) < abs(lon_change) * 0.1:
        print("   ✓ Longitude increased, Latitude ~unchanged (CORRECT!)")
    else:
        print("   ✗ WRONG! East movement should increase longitude!")

    # Test 4: Move 100m West (positive Y in FLU, since Y is Left)
    print("\n4. Move 100m West (Y=100 in FLU):")
    result = calculate_gps_measurements(
        position_flu=np.array([0, 100, 0]),  # 100m West (Left)
        linear_velocity_flu=np.zeros(3),
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        origin_alt=origin_alt
    )
    lat_change = result['latitude'] - origin_lat
    lon_change = result['longitude'] - origin_lon
    print(f"   Lat: {result['latitude']:.6f}° (Δ = {lat_change:.6f}°)")
    print(f"   Lon: {result['longitude']:.6f}° (Δ = {lon_change:.6f}°)")

    if lon_change < 0 and abs(lat_change) < abs(lon_change) * 0.1:
        print("   ✓ Longitude decreased, Latitude ~unchanged (CORRECT!)")
    else:
        print("   ✗ WRONG! West movement should decrease longitude!")

    print("\n" + "="*60)
    print("GPS POSITION TEST COMPLETE")
    print("="*60)

except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()