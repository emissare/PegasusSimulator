"""
Unit tests for coordinate transformations using omni.kit.test framework.
Tests the fix for the 90° yaw offset issue.
"""

import omni.kit.test
import numpy as np
from scipy.spatial.transform import Rotation


class TestCoordinateTransforms(omni.kit.test.AsyncTestCase):
    """Test coordinate system transformations."""

    async def test_identity_quaternion_gives_zero_yaw(self):
        """CRITICAL TEST: Identity quaternion should give yaw=0° (not 90°).

        This test verifies that the 90° yaw offset bug is fixed.
        Vehicle pointing along Isaac's X-axis should show yaw=0° in PX4.
        """
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
        self.assertAlmostEqual(yaw, 0.0, places=0,
                              msg=f"Identity quaternion should give yaw=0°, got {yaw:.1f}°")
        print(f"✓ YAW FIX VERIFIED: Identity quaternion gives yaw={yaw:.1f}°")

    async def test_90_degree_left_rotation(self):
        """Test 90° left rotation gives yaw=-90°."""
        from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

        # 90° rotation around Z axis (pointing left/west)
        result = calculate_imu_measurements(
            attitude_flu=np.array([0, 0, 0.707, 0.707]),
            angular_velocity_flu=np.zeros(3),
            linear_velocity_flu=np.zeros(3),
            prev_linear_velocity_flu=np.zeros(3),
            dt=0.01
        )

        rot = Rotation.from_quat(result["orientation"])
        euler = rot.as_euler('ZYX', degrees=True)
        yaw = euler[0]

        self.assertAlmostEqual(yaw, -90.0, places=0,
                              msg=f"90° left rotation should give yaw=-90°, got {yaw:.1f}°")
        print(f"✓ 90° left rotation gives yaw={yaw:.1f}°")

    async def test_90_degree_right_rotation(self):
        """Test 90° right rotation gives yaw=90°."""
        from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

        # 90° rotation around Z axis (pointing right/east)
        result = calculate_imu_measurements(
            attitude_flu=np.array([0, 0, -0.707, 0.707]),
            angular_velocity_flu=np.zeros(3),
            linear_velocity_flu=np.zeros(3),
            prev_linear_velocity_flu=np.zeros(3),
            dt=0.01
        )

        rot = Rotation.from_quat(result["orientation"])
        euler = rot.as_euler('ZYX', degrees=True)
        yaw = euler[0]

        self.assertAlmostEqual(yaw, 90.0, places=0,
                              msg=f"90° right rotation should give yaw=90°, got {yaw:.1f}°")
        print(f"✓ 90° right rotation gives yaw={yaw:.1f}°")

    async def test_180_degree_rotation(self):
        """Test 180° rotation gives yaw=180°."""
        from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

        # 180° rotation around Z axis (pointing back/south)
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
        self.assertTrue(abs(yaw - 180) < 1 or abs(yaw + 180) < 1,
                       msg=f"180° rotation should give yaw=±180°, got {yaw:.1f}°")
        print(f"✓ 180° rotation gives yaw={yaw:.1f}°")

    async def test_velocity_transformation_north(self):
        """Test velocity transformation: moving North."""
        from pegasus.simulator.logic.sensors.calculations.gps_calc import calculate_gps_measurements

        # Moving north at 10 m/s
        result = calculate_gps_measurements(
            position_flu=np.zeros(3),
            linear_velocity_flu=np.array([10, 0, 0]),  # X is North in FLU
            origin_lat=47.397742,
            origin_lon=8.545594,
            origin_alt=488.0
        )

        self.assertAlmostEqual(result["velocity_north"], 10.0, places=1)
        self.assertAlmostEqual(result["velocity_east"], 0.0, places=1)
        self.assertAlmostEqual(result["velocity_down"], 0.0, places=1)
        print(f"✓ North velocity transformation correct")

    async def test_velocity_transformation_west(self):
        """Test velocity transformation: moving West."""
        from pegasus.simulator.logic.sensors.calculations.gps_calc import calculate_gps_measurements

        # Moving west at 10 m/s
        result = calculate_gps_measurements(
            position_flu=np.zeros(3),
            linear_velocity_flu=np.array([0, 10, 0]),  # Y is West in FLU
            origin_lat=47.397742,
            origin_lon=8.545594,
            origin_alt=488.0
        )

        self.assertAlmostEqual(result["velocity_north"], 0.0, places=1)
        self.assertAlmostEqual(result["velocity_east"], -10.0, places=1)  # West is -East
        self.assertAlmostEqual(result["velocity_down"], 0.0, places=1)
        print(f"✓ West velocity transformation correct")

    async def test_angular_velocity_transformation(self):
        """Test angular velocity transformation from FLU to FRD."""
        from pegasus.simulator.logic.rotations import rot_FLU_to_FRD

        # Test roll (X axis)
        angular_flu = np.array([1.0, 0, 0])
        angular_frd = rot_FLU_to_FRD.apply(angular_flu)
        self.assertAlmostEqual(angular_frd[0], 1.0, places=5)  # Roll unchanged
        self.assertAlmostEqual(angular_frd[1], 0.0, places=5)
        self.assertAlmostEqual(angular_frd[2], 0.0, places=5)

        # Test pitch (Y axis)
        angular_flu = np.array([0, 1.0, 0])
        angular_frd = rot_FLU_to_FRD.apply(angular_flu)
        self.assertAlmostEqual(angular_frd[0], 0.0, places=5)
        self.assertAlmostEqual(angular_frd[1], -1.0, places=5)  # Pitch flips
        self.assertAlmostEqual(angular_frd[2], 0.0, places=5)

        # Test yaw (Z axis)
        angular_flu = np.array([0, 0, 1.0])
        angular_frd = rot_FLU_to_FRD.apply(angular_flu)
        self.assertAlmostEqual(angular_frd[0], 0.0, places=5)
        self.assertAlmostEqual(angular_frd[1], 0.0, places=5)
        self.assertAlmostEqual(angular_frd[2], -1.0, places=5)  # Yaw flips

        print(f"✓ Angular velocity transformations correct")

    async def test_complete_transformation_summary(self):
        """Summary test showing all transformations work correctly."""
        print("\n" + "="*60)
        print("COORDINATE TRANSFORMATION TEST SUMMARY")
        print("="*60)

        from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

        test_cases = [
            ("Identity (North)", [0, 0, 0, 1], 0),
            ("90° Left (West)", [0, 0, 0.707, 0.707], -90),
            ("90° Right (East)", [0, 0, -0.707, 0.707], 90),
            ("180° (South)", [0, 0, 1, 0], 180),
        ]

        all_pass = True
        for name, quat, expected_yaw in test_cases:
            result = calculate_imu_measurements(
                attitude_flu=np.array(quat),
                angular_velocity_flu=np.zeros(3),
                linear_velocity_flu=np.zeros(3),
                prev_linear_velocity_flu=np.zeros(3),
                dt=0.01
            )

            rot = Rotation.from_quat(result["orientation"])
            euler = rot.as_euler('ZYX', degrees=True)
            actual_yaw = euler[0]

            # Handle ±180 equivalence
            if expected_yaw == 180:
                yaw_correct = abs(actual_yaw - 180) < 1 or abs(actual_yaw + 180) < 1
            else:
                yaw_correct = abs(actual_yaw - expected_yaw) < 1

            status = "✓" if yaw_correct else "✗"
            print(f"{status} {name:20} Expected: {expected_yaw:4}°, Got: {actual_yaw:.1f}°")

            if not yaw_correct:
                all_pass = False

        print("="*60)
        if all_pass:
            print("✓ ALL COORDINATE TRANSFORMATIONS CORRECT!")
            print("✓ The 90° yaw offset bug is FIXED!")
        else:
            print("✗ Some transformations failed")

        self.assertTrue(all_pass, "Not all coordinate transformations passed")