"""
Absolutely minimal test - just check if imports work
"""

print("\nMinimal Import Test")
print("-" * 30)

# Step 1: Test basic imports
try:
    import numpy as np
    print("1. NumPy: OK")
except:
    print("1. NumPy: FAILED")

try:
    from scipy.spatial.transform import Rotation
    print("2. SciPy: OK")
except:
    print("2. SciPy: FAILED")

# Step 2: Test our modules exist
try:
    import pegasus.simulator.logic.rotations as rotations
    print("3. Rotations module: OK")
except Exception as e:
    print(f"3. Rotations module: FAILED - {e}")

try:
    import pegasus.simulator.logic.sensors.calculations.imu_calc as imu_calc
    print("4. IMU calc module: OK")
except Exception as e:
    print(f"4. IMU calc module: FAILED - {e}")

# Step 3: Very simple transformation test
try:
    import numpy as np
    from scipy.spatial.transform import Rotation

    # Just test a basic quaternion to euler conversion
    quat = [0, 0, 0, 1]  # Identity
    rot = Rotation.from_quat(quat)
    euler = rot.as_euler('ZYX', degrees=True)

    print(f"\n5. Basic quaternion test:")
    print(f"   Identity quaternion → Euler ZYX = [{euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}]°")
    print(f"   Yaw (Z) = {euler[0]:.1f}°")

except Exception as e:
    print(f"5. Basic test FAILED: {e}")

print("-" * 30)