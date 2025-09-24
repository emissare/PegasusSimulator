"""
Complete Test Suite for Script Editor
Safe version that won't crash Isaac Sim
"""

print("\n" + "="*60)
print("PEGASUS SIMULATOR COORDINATE TRANSFORMATION TESTS")
print("="*60)

passed = 0
failed = 0

# Test 1: Critical Yaw Fix
print("\n1. CRITICAL YAW FIX TEST")
print("-" * 40)
try:
    import numpy as np
    from scipy.spatial.transform import Rotation
    from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

    result = calculate_imu_measurements(
        attitude_flu=np.array([0, 0, 0, 1]),
        angular_velocity_flu=np.zeros(3),
        linear_velocity_flu=np.zeros(3),
        prev_linear_velocity_flu=np.zeros(3),
        dt=0.01
    )

    rot = Rotation.from_quat(result["orientation"])
    euler = rot.as_euler('ZYX', degrees=True)
    yaw = euler[0]

    if abs(yaw) < 1.0:
        print(f"✓ Identity quaternion → yaw = {yaw:.1f}° (CORRECT!)")
        passed += 1
    else:
        print(f"✗ Identity quaternion → yaw = {yaw:.1f}° (SHOULD BE 0°)")
        failed += 1
except Exception as e:
    print(f"✗ Test failed: {e}")
    failed += 1

# Test 2: 90 Degree Rotations
print("\n2. 90° ROTATION TESTS")
print("-" * 40)
try:
    # Left rotation (West)
    result = calculate_imu_measurements(
        attitude_flu=np.array([0, 0, 0.707, 0.707]),
        angular_velocity_flu=np.zeros(3),
        linear_velocity_flu=np.zeros(3),
        prev_linear_velocity_flu=np.zeros(3),
        dt=0.01
    )
    rot = Rotation.from_quat(result["orientation"])
    euler = rot.as_euler('ZYX', degrees=True)
    yaw_left = euler[0]

    if abs(yaw_left + 90) < 1.0:
        print(f"✓ 90° left rotation → yaw = {yaw_left:.1f}°")
        passed += 1
    else:
        print(f"✗ 90° left rotation → yaw = {yaw_left:.1f}° (SHOULD BE -90°)")
        failed += 1

    # Right rotation (East)
    result = calculate_imu_measurements(
        attitude_flu=np.array([0, 0, -0.707, 0.707]),
        angular_velocity_flu=np.zeros(3),
        linear_velocity_flu=np.zeros(3),
        prev_linear_velocity_flu=np.zeros(3),
        dt=0.01
    )
    rot = Rotation.from_quat(result["orientation"])
    euler = rot.as_euler('ZYX', degrees=True)
    yaw_right = euler[0]

    if abs(yaw_right - 90) < 1.0:
        print(f"✓ 90° right rotation → yaw = {yaw_right:.1f}°")
        passed += 1
    else:
        print(f"✗ 90° right rotation → yaw = {yaw_right:.1f}° (SHOULD BE 90°)")
        failed += 1

except Exception as e:
    print(f"✗ Test failed: {e}")
    failed += 1

# Test 3: GPS Velocity Transformations
print("\n3. GPS VELOCITY TESTS")
print("-" * 40)
try:
    from pegasus.simulator.logic.sensors.calculations.gps_calc import calculate_gps_measurements

    # Moving North
    result = calculate_gps_measurements(
        position_flu=np.zeros(3),
        linear_velocity_flu=np.array([10, 0, 0]),  # North in FLU
        origin_lat=47.397742,
        origin_lon=8.545594,
        origin_alt=488.0
    )

    if abs(result["velocity_north"] - 10.0) < 0.1:
        print(f"✓ North velocity: FLU=[10,0,0] → NED north={result['velocity_north']:.1f} m/s")
        passed += 1
    else:
        print(f"✗ North velocity incorrect: {result['velocity_north']:.1f} m/s")
        failed += 1

    # Moving West
    result = calculate_gps_measurements(
        position_flu=np.zeros(3),
        linear_velocity_flu=np.array([0, 10, 0]),  # West in FLU
        origin_lat=47.397742,
        origin_lon=8.545594,
        origin_alt=488.0
    )

    if abs(result["velocity_east"] + 10.0) < 0.1:
        print(f"✓ West velocity: FLU=[0,10,0] → NED east={result['velocity_east']:.1f} m/s")
        passed += 1
    else:
        print(f"✗ West velocity incorrect: {result['velocity_east']:.1f} m/s")
        failed += 1

except Exception as e:
    print(f"✗ Test failed: {e}")
    failed += 1

# Test 4: Angular Velocity Transformation
print("\n4. ANGULAR VELOCITY TRANSFORMATION")
print("-" * 40)
try:
    from pegasus.simulator.logic.rotations import rot_FLU_to_FRD

    # Yaw transformation
    angular_flu = np.array([0, 0, 1.0])
    angular_frd = rot_FLU_to_FRD.apply(angular_flu)

    if abs(angular_frd[2] + 1.0) < 0.001:
        print(f"✓ Yaw: FLU=[0,0,1] → FRD=[{angular_frd[0]:.1f},{angular_frd[1]:.1f},{angular_frd[2]:.1f}]")
        passed += 1
    else:
        print(f"✗ Yaw transformation incorrect: FRD={angular_frd}")
        failed += 1

except Exception as e:
    print(f"✗ Test failed: {e}")
    failed += 1

# Summary
print("\n" + "="*60)
print("TEST SUMMARY")
print("="*60)
print(f"Passed: {passed}")
print(f"Failed: {failed}")

if failed == 0:
    print("\n✓✓✓ ALL TESTS PASSED! ✓✓✓")
    print("✓ The 90° yaw offset bug is FIXED!")
    print("✓ Coordinate transformations are working correctly!")
else:
    print(f"\n✗ {failed} test(s) failed")
    print("✗ Check the failed tests above for details")

print("="*60)