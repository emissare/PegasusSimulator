"""
Simple Yaw Test for Script Editor
This is a minimal test to verify the 90° yaw offset fix without crashing.
"""

print("\n" + "="*50)
print("SIMPLE YAW TEST")
print("="*50)

try:
    import numpy as np
    print("✓ NumPy imported")

    from scipy.spatial.transform import Rotation
    print("✓ SciPy imported")

    # Try to import our calculation module
    from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements
    print("✓ IMU calculations imported")

    # Run the critical test
    print("\nTesting identity quaternion...")

    result = calculate_imu_measurements(
        attitude_flu=np.array([0, 0, 0, 1]),
        angular_velocity_flu=np.zeros(3),
        linear_velocity_flu=np.zeros(3),
        prev_linear_velocity_flu=np.zeros(3),
        dt=0.01
    )

    # Extract yaw
    rot = Rotation.from_quat(result["orientation"])
    euler = rot.as_euler('ZYX', degrees=True)
    yaw = euler[0]

    print(f"\nRESULT: Yaw = {yaw:.1f}°")

    if abs(yaw) < 1.0:
        print("✓ SUCCESS! The 90° yaw offset is FIXED!")
        print("✓ Vehicle pointing along X-axis shows yaw=0°")
    else:
        print("✗ FAILED! The 90° yaw offset bug still exists")
        print(f"✗ Expected yaw=0°, got yaw={yaw:.1f}°")

except ImportError as e:
    print(f"\n✗ Import Error: {e}")
    print("Make sure the Pegasus extension is loaded")

except Exception as e:
    print(f"\n✗ Error during test: {e}")
    print("Check that the calculation functions are working")

print("="*50)