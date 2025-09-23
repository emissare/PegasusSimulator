"""
Test GPS position calculation with proper NED coordinates.
Run this in Script Editor to verify GPS position fix.
"""

print("\n" + "="*60)
print("GPS NED POSITION TEST")
print("="*60)

try:
    import numpy as np
    from pegasus.simulator.logic.sensors.geo_mag_utils import convert_ned_to_geodetic

    # San Diego coordinates
    origin_lat_deg = 32.7746
    origin_lon_deg = -117.079
    origin_alt_m = 90.0

    print(f"\nOrigin: {origin_lat_deg}° lat, {origin_lon_deg}° lon")

    # Test 1: Vehicle at origin
    print("\n1. Vehicle at origin (0,0,0) NED:")
    position_ned_m = np.array([0, 0, 0])
    lat_rad, lon_rad = convert_ned_to_geodetic(
        position_ned_m,
        np.radians(origin_lat_deg),
        np.radians(origin_lon_deg)
    )
    lat_deg = np.degrees(lat_rad)
    lon_deg = np.degrees(lon_rad)
    print(f"   Result: {lat_deg:.6f}° lat, {lon_deg:.6f}° lon")
    if abs(lat_deg - origin_lat_deg) < 0.00001 and abs(lon_deg - origin_lon_deg) < 0.00001:
        print("   ✓ Correct!")
    else:
        print("   ✗ Wrong!")

    # Test 2: Move 1000m North (positive North in NED)
    print("\n2. Move 1000m North (NED=[1000,0,0]):")
    position_ned_m = np.array([1000, 0, 0])
    lat_rad, lon_rad = convert_ned_to_geodetic(
        position_ned_m,
        np.radians(origin_lat_deg),
        np.radians(origin_lon_deg)
    )
    lat_deg = np.degrees(lat_rad)
    lon_deg = np.degrees(lon_rad)
    lat_change = lat_deg - origin_lat_deg
    lon_change = lon_deg - origin_lon_deg
    print(f"   Result: {lat_deg:.6f}° lat, {lon_deg:.6f}° lon")
    print(f"   Δlat: {lat_change:.6f}°, Δlon: {lon_change:.6f}°")

    # 1000m North should increase latitude by ~0.00899° (1000m / 111km per degree)
    expected_lat_change = 1000.0 / 111000.0  # roughly
    if lat_change > 0 and abs(lat_change - expected_lat_change) < 0.001:
        print(f"   ✓ Latitude increased by ~{lat_change:.6f}° (expected ~{expected_lat_change:.6f}°)")
    else:
        print(f"   ✗ Wrong! Expected latitude to increase by ~{expected_lat_change:.6f}°")

    # Test 3: Move 1000m East (positive East in NED)
    print("\n3. Move 1000m East (NED=[0,1000,0]):")
    position_ned_m = np.array([0, 1000, 0])
    lat_rad, lon_rad = convert_ned_to_geodetic(
        position_ned_m,
        np.radians(origin_lat_deg),
        np.radians(origin_lon_deg)
    )
    lat_deg = np.degrees(lat_rad)
    lon_deg = np.degrees(lon_rad)
    lat_change = lat_deg - origin_lat_deg
    lon_change = lon_deg - origin_lon_deg
    print(f"   Result: {lat_deg:.6f}° lat, {lon_deg:.6f}° lon")
    print(f"   Δlat: {lat_change:.6f}°, Δlon: {lon_change:.6f}°")

    # 1000m East should increase longitude (at 32° latitude, ~0.0106°)
    # Longitude degrees per meter varies with latitude: 1/(111km * cos(lat))
    meters_per_lon_degree = 111000.0 * np.cos(np.radians(origin_lat_deg))
    expected_lon_change = 1000.0 / meters_per_lon_degree
    if lon_change > 0 and abs(lon_change - expected_lon_change) < 0.001:
        print(f"   ✓ Longitude increased by ~{lon_change:.6f}° (expected ~{expected_lon_change:.6f}°)")
    else:
        print(f"   ✗ Wrong! Expected longitude to increase by ~{expected_lon_change:.6f}°")

    print("\n" + "="*60)
    print("Test how FLU maps to NED in GPS sensor:")
    print("="*60)

    # Simulate FLU to NED conversion as done in GPS sensor
    print("\nFLU→NED conversion (as in gps.py):")
    print("  North_NED = North_FLU  (X_FLU)")
    print("  East_NED = -West_FLU   (-Y_FLU)")
    print("  Down_NED = -Up_FLU     (-Z_FLU)")

    # Test with FLU position moving in positive X (North)
    print("\nFLU [1000,0,0] (1000m North in Isaac):")
    position_flu_m = np.array([1000, 0, 0])
    position_ned_m = np.array([
        position_flu_m[0],     # North = FLU_X
        -position_flu_m[1],    # East = -FLU_Y
        -position_flu_m[2]     # Down = -FLU_Z
    ])
    print(f"  → NED {position_ned_m}")

    lat_rad, lon_rad = convert_ned_to_geodetic(
        position_ned_m,
        np.radians(origin_lat_deg),
        np.radians(origin_lon_deg)
    )
    lat_deg = np.degrees(lat_rad)
    lon_deg = np.degrees(lon_rad)
    lat_change = lat_deg - origin_lat_deg
    lon_change = lon_deg - origin_lon_deg
    print(f"  → Lat change: {lat_change:.6f}°, Lon change: {lon_change:.6f}°")
    if lat_change > 0 and abs(lon_change) < abs(lat_change) * 0.1:
        print("  ✓ Correct! Moving North in Isaac increases latitude!")
    else:
        print("  ✗ Wrong! North movement should increase latitude!")

    print("\n" + "="*60)

except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()