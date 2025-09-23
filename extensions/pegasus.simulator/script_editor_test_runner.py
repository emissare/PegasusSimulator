"""
Script Editor Test Runner for Pegasus Simulator
Copy and paste this entire script into Isaac Sim's Script Editor to run tests.

This tests the coordinate transformation fixes and sensor calculations.
"""

import numpy as np
from scipy.spatial.transform import Rotation
import sys


class TestRunner:
    """Simple test runner for Script Editor."""

    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.test_results = []

    def assert_almost_equal(self, actual, expected, places=5, msg=""):
        """Check if values are almost equal."""
        tolerance = 10 ** (-places)
        if abs(actual - expected) > tolerance:
            raise AssertionError(f"{msg}\nExpected: {expected}, Got: {actual}")

    def assert_array_almost_equal(self, actual, expected, decimal=5):
        """Check if arrays are almost equal."""
        np.testing.assert_array_almost_equal(actual, expected, decimal=decimal)

    def run_test(self, test_name, test_func):
        """Run a single test and track results."""
        try:
            test_func()
            self.passed += 1
            self.test_results.append((test_name, True, None))
            print(f"✓ {test_name}")
            return True
        except Exception as e:
            self.failed += 1
            self.test_results.append((test_name, False, str(e)))
            print(f"✗ {test_name}")
            print(f"  Error: {e}")
            return False

    def print_summary(self):
        """Print test summary."""
        print("\n" + "="*60)
        print("TEST SUMMARY")
        print("="*60)
        print(f"Passed: {self.passed}")
        print(f"Failed: {self.failed}")

        if self.failed == 0:
            print("\n✓ ALL TESTS PASSED!")
            print("✓ The 90° yaw offset bug is FIXED!")
            print("✓ Coordinate transformations are working correctly!")
        else:
            print("\n✗ SOME TESTS FAILED")
            print("Failed tests:")
            for name, passed, error in self.test_results:
                if not passed:
                    print(f"  - {name}")
        print("="*60)


def test_critical_yaw_fix():
    """CRITICAL TEST: Identity quaternion should give yaw=0° (not 90°)."""
    from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

    # Identity quaternion - vehicle pointing forward (North)
    result = calculate_imu_measurements(
        attitude_flu=np.array([0, 0, 0, 1]),
        angular_velocity_flu=np.zeros(3),
        linear_velocity_flu=np.zeros(3),
        prev_linear_velocity_flu=np.zeros(3),
        dt=0.01
    )

    # Extract yaw from result
    rot = Rotation.from_quat(result["orientation"])
    euler = rot.as_euler('ZYX', degrees=True)
    yaw = euler[0]

    # This should be 0, not 90!
    if abs(yaw) > 1.0:
        raise AssertionError(f"Identity quaternion should give yaw=0°, got {yaw:.1f}°")

    print(f"  → Identity quaternion gives yaw={yaw:.1f}° (CORRECT!)")


def test_90_degree_rotations():
    """Test 90° left and right rotations."""
    from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

    # Test 90° left rotation (pointing west)
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

    if abs(yaw_left + 90) > 1.0:
        raise AssertionError(f"90° left rotation should give yaw=-90°, got {yaw_left:.1f}°")

    print(f"  → 90° left rotation gives yaw={yaw_left:.1f}°")

    # Test 90° right rotation (pointing east)
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

    if abs(yaw_right - 90) > 1.0:
        raise AssertionError(f"90° right rotation should give yaw=90°, got {yaw_right:.1f}°")

    print(f"  → 90° right rotation gives yaw={yaw_right:.1f}°")


def test_180_degree_rotation():
    """Test 180° rotation (pointing south)."""
    from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

    result = calculate_imu_measurements(
        attitude_flu=np.array([0, 0, 1, 0]),
        angular_velocity_flu=np.zeros(3),
        linear_velocity_flu=np.zeros(3),
        prev_linear_velocity_flu=np.zeros(3),
        dt=0.01
    )

    rot = Rotation.from_quat(result["orientation"])
    euler = rot.as_euler('ZYX', degrees=True)
    yaw = euler[0]

    # Accept both 180 and -180 as they're equivalent
    if abs(yaw - 180) > 1 and abs(yaw + 180) > 1:
        raise AssertionError(f"180° rotation should give yaw=±180°, got {yaw:.1f}°")

    print(f"  → 180° rotation gives yaw={yaw:.1f}°")


def test_imu_stationary():
    """Test IMU for stationary vehicle."""
    from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

    result = calculate_imu_measurements(
        attitude_flu=np.array([0, 0, 0, 1]),
        angular_velocity_flu=np.zeros(3),
        linear_velocity_flu=np.zeros(3),
        prev_linear_velocity_flu=np.zeros(3),
        dt=0.01
    )

    # Check angular velocity (should be zero)
    if np.linalg.norm(result["angular_velocity"]) > 0.001:
        raise AssertionError("Stationary vehicle should have zero angular velocity")

    # Check linear acceleration (should only show gravity)
    accel = result["linear_acceleration"]
    if abs(accel[0]) > 0.1 or abs(accel[1]) > 0.1:
        raise AssertionError("Stationary vehicle should have no horizontal acceleration")

    if abs(accel[2] - 9.80665) > 0.1:
        raise AssertionError(f"Stationary vehicle should show gravity, got {accel[2]:.2f}")

    print(f"  → Stationary IMU shows gravity={accel[2]:.2f} m/s²")


def test_imu_forward_acceleration():
    """Test IMU for forward acceleration."""
    from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

    # Vehicle accelerating from 5 m/s to 10 m/s in 0.1s (50 m/s² acceleration)
    result = calculate_imu_measurements(
        attitude_flu=np.array([0, 0, 0, 1]),
        angular_velocity_flu=np.zeros(3),
        linear_velocity_flu=np.array([10.0, 0, 0]),
        prev_linear_velocity_flu=np.array([5.0, 0, 0]),
        dt=0.1
    )

    accel = result["linear_acceleration"]
    if abs(accel[0] - 50.0) > 1.0:
        raise AssertionError(f"Forward acceleration should be 50 m/s², got {accel[0]:.1f}")

    print(f"  → Forward acceleration={accel[0]:.1f} m/s²")


def test_gps_velocity_north():
    """Test GPS velocity when moving north."""
    from pegasus.simulator.logic.sensors.calculations.gps_calc import calculate_gps_measurements

    result = calculate_gps_measurements(
        position_flu=np.zeros(3),
        linear_velocity_flu=np.array([10, 0, 0]),  # Moving north at 10 m/s
        origin_lat=47.397742,
        origin_lon=8.545594,
        origin_alt=488.0
    )

    if abs(result["velocity_north"] - 10.0) > 0.1:
        raise AssertionError(f"North velocity should be 10 m/s, got {result['velocity_north']:.1f}")

    if abs(result["velocity_east"]) > 0.1:
        raise AssertionError(f"East velocity should be 0, got {result['velocity_east']:.1f}")

    print(f"  → GPS North velocity={result['velocity_north']:.1f} m/s")


def test_gps_velocity_west():
    """Test GPS velocity when moving west."""
    from pegasus.simulator.logic.sensors.calculations.gps_calc import calculate_gps_measurements

    result = calculate_gps_measurements(
        position_flu=np.zeros(3),
        linear_velocity_flu=np.array([0, 10, 0]),  # Moving west at 10 m/s
        origin_lat=47.397742,
        origin_lon=8.545594,
        origin_alt=488.0
    )

    if abs(result["velocity_north"]) > 0.1:
        raise AssertionError(f"North velocity should be 0, got {result['velocity_north']:.1f}")

    if abs(result["velocity_east"] + 10.0) > 0.1:  # West is negative east
        raise AssertionError(f"East velocity should be -10 m/s (west), got {result['velocity_east']:.1f}")

    print(f"  → GPS West velocity={result['velocity_east']:.1f} m/s (negative=west)")


def test_angular_velocity_transformation():
    """Test angular velocity transformations."""
    from pegasus.simulator.logic.rotations import rot_FLU_body_to_FRD_body

    # Test yaw transformation (Z axis)
    angular_flu = np.array([0, 0, 1.0])
    angular_frd = rot_FLU_body_to_FRD_body.apply(angular_flu)

    if abs(angular_frd[2] + 1.0) > 0.001:  # Should flip sign
        raise AssertionError(f"Yaw should flip sign in FRD, got {angular_frd[2]:.3f}")

    print(f"  → Yaw transformation: FLU=1.0 → FRD={angular_frd[2]:.1f}")


def main():
    """Run all tests."""
    print("\n" + "="*60)
    print("PEGASUS SIMULATOR TEST SUITE")
    print("Testing Coordinate Transformations and Sensor Calculations")
    print("="*60 + "\n")

    runner = TestRunner()

    # Critical tests
    print("CRITICAL TESTS (90° Yaw Offset Fix):")
    print("-" * 40)
    runner.run_test("Identity quaternion gives yaw=0°", test_critical_yaw_fix)
    runner.run_test("90° rotations", test_90_degree_rotations)
    runner.run_test("180° rotation", test_180_degree_rotation)

    # IMU tests
    print("\nIMU CALCULATION TESTS:")
    print("-" * 40)
    runner.run_test("Stationary vehicle", test_imu_stationary)
    runner.run_test("Forward acceleration", test_imu_forward_acceleration)

    # GPS tests
    print("\nGPS CALCULATION TESTS:")
    print("-" * 40)
    runner.run_test("North velocity", test_gps_velocity_north)
    runner.run_test("West velocity", test_gps_velocity_west)

    # Transformation tests
    print("\nCOORDINATE TRANSFORMATION TESTS:")
    print("-" * 40)
    runner.run_test("Angular velocity FLU→FRD", test_angular_velocity_transformation)

    # Print summary
    runner.print_summary()

    return runner.failed == 0


if __name__ == "__main__":
    try:
        success = main()
        if not success:
            sys.exit(1)
    except Exception as e:
        print(f"\n✗ TEST RUNNER ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)