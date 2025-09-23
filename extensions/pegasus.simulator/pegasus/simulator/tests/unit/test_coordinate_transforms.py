"""Unit tests for coordinate transformations.

Tests the fix for the 90° yaw offset issue and verifies
all coordinate transformations are correct.
"""

import pytest
import numpy as np
from scipy.spatial.transform import Rotation


class TestCoordinateTransforms:
    """Test coordinate system transformations."""

    def test_flu_to_ned_inertial_rotation_matrix(self, rot_FLU_inertial_to_NED_inertial):
        """Test that FLU to NED inertial rotation is correct."""
        # Should be 180° rotation around X-axis
        expected_matrix = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])

        actual_matrix = rot_FLU_inertial_to_NED_inertial.as_matrix()
        np.testing.assert_array_almost_equal(actual_matrix, expected_matrix)

    def test_flu_to_frd_body_rotation_matrix(self, rot_FLU_body_to_FRD_body):
        """Test that FLU to FRD body rotation is correct."""
        # Should also be 180° rotation around X-axis
        expected_matrix = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])

        actual_matrix = rot_FLU_body_to_FRD_body.as_matrix()
        np.testing.assert_array_almost_equal(actual_matrix, expected_matrix)

    @pytest.mark.parametrize("test_case", [
        {"flu": [1, 0, 0], "expected_ned": [1, 0, 0]},   # North stays North
        {"flu": [0, 1, 0], "expected_ned": [0, -1, 0]},  # Left becomes West (-East)
        {"flu": [0, 0, 1], "expected_ned": [0, 0, -1]},  # Up becomes -Down
    ])
    def test_position_flu_to_ned(self, rot_FLU_inertial_to_NED_inertial, test_case):
        """Test position transformation from FLU to NED."""
        flu_pos = np.array(test_case["flu"])
        expected_ned = np.array(test_case["expected_ned"])

        actual_ned = rot_FLU_inertial_to_NED_inertial.apply(flu_pos)
        np.testing.assert_array_almost_equal(actual_ned, expected_ned)

    @pytest.mark.parametrize("test_case", [
        {"flu": [1, 0, 0], "expected_frd": [1, 0, 0]},    # Roll axis unchanged
        {"flu": [0, 1, 0], "expected_frd": [0, -1, 0]},   # Pitch axis flips
        {"flu": [0, 0, 1], "expected_frd": [0, 0, -1]},   # Yaw axis flips
    ])
    def test_angular_velocity_flu_to_frd(self, rot_FLU_body_to_FRD_body, test_case):
        """Test angular velocity transformation from FLU to FRD."""
        flu_ang_vel = np.array(test_case["flu"])
        expected_frd = np.array(test_case["expected_frd"])

        actual_frd = rot_FLU_body_to_FRD_body.apply(flu_ang_vel)
        np.testing.assert_array_almost_equal(actual_frd, expected_frd)


class TestYawOffsetFix:
    """Test that the 90° yaw offset bug is fixed."""

    @pytest.mark.parametrize("quat_data", [
        {"quat": [0, 0, 0, 1], "expected_yaw": 0, "name": "Identity"},
        {"quat": [0, 0, 0.707, 0.707], "expected_yaw": -90, "name": "90° left"},
        {"quat": [0, 0, -0.707, 0.707], "expected_yaw": 90, "name": "90° right"},
        {"quat": [0, 0, 1, 0], "expected_yaw": 180, "name": "180°"},
    ])
    def test_attitude_transformation_yaw(self, rot_FLU_inertial_to_NED_inertial,
                                        rot_FLU_body_to_FRD_body, quat_data):
        """Test that attitude transformations produce correct yaw."""
        # Transform from FLU/FLU to FRD/NED
        attitude_flu = Rotation.from_quat(quat_data["quat"])
        attitude_frd_ned = (rot_FLU_inertial_to_NED_inertial *
                           attitude_flu *
                           rot_FLU_body_to_FRD_body)

        # Get Euler angles (ZYX = Yaw, Pitch, Roll)
        euler = attitude_frd_ned.as_euler('ZYX', degrees=True)
        actual_yaw = euler[0]

        # Check yaw is correct (within 1 degree tolerance)
        assert abs(actual_yaw - quat_data["expected_yaw"]) < 1.0, \
            f"{quat_data['name']}: Expected yaw={quat_data['expected_yaw']}°, got {actual_yaw:.1f}°"

    def test_identity_quaternion_gives_zero_yaw(self, rot_FLU_inertial_to_NED_inertial,
                                               rot_FLU_body_to_FRD_body):
        """Specific test: identity quaternion should give yaw=0° (not 90°)."""
        # This is THE critical test for the bug fix
        identity = Rotation.from_quat([0, 0, 0, 1])

        # Transform
        result = rot_FLU_inertial_to_NED_inertial * identity * rot_FLU_body_to_FRD_body

        # Get yaw
        euler = result.as_euler('ZYX', degrees=True)
        yaw = euler[0]

        assert abs(yaw) < 1.0, f"Identity quaternion should give yaw=0°, not {yaw:.1f}°"
        print(f"✓ Identity quaternion correctly gives yaw={yaw:.1f}° (bug is fixed!)")


class TestVelocityTransformations:
    """Test velocity transformations between frames."""

    def test_velocity_transformations_batch(self, rot_FLU_inertial_to_NED_inertial,
                                           test_velocities):
        """Test multiple velocity transformations."""
        for test_case in test_velocities:
            flu_vel = test_case["flu"]
            expected_ned = test_case["expected_ned"]

            actual_ned = rot_FLU_inertial_to_NED_inertial.apply(flu_vel)

            np.testing.assert_allclose(
                actual_ned, expected_ned, rtol=1e-6,
                err_msg=f"Velocity {test_case['name']} transformation failed"
            )


class TestCompleteTransformPipeline:
    """Test the complete transformation pipeline."""

    def test_full_state_transformation(self, rot_FLU_inertial_to_NED_inertial,
                                      rot_FLU_body_to_FRD_body):
        """Test transforming a complete vehicle state."""
        # Vehicle pointing 45° left, moving forward-left, rolling
        attitude_flu = Rotation.from_euler('Z', 45, degrees=True).as_quat()
        position_flu = np.array([10, 5, 2])  # 10m forward, 5m left, 2m up
        velocity_flu = np.array([7.07, 7.07, 0])  # Moving at 45° angle
        angular_velocity_flu = np.array([0.1, 0, 0.5])  # Rolling and yawing

        # Transform position
        position_ned = rot_FLU_inertial_to_NED_inertial.apply(position_flu)
        assert position_ned[0] == 10  # North
        assert position_ned[1] == -5  # East (negative is West)
        assert position_ned[2] == -2  # Down (negative is Up)

        # Transform velocity
        velocity_ned = rot_FLU_inertial_to_NED_inertial.apply(velocity_flu)
        np.testing.assert_almost_equal(velocity_ned, [7.07, -7.07, 0])

        # Transform attitude
        attitude_flu_rot = Rotation.from_quat(attitude_flu)
        attitude_frd_ned = (rot_FLU_inertial_to_NED_inertial *
                           attitude_flu_rot *
                           rot_FLU_body_to_FRD_body)
        euler = attitude_frd_ned.as_euler('ZYX', degrees=True)
        assert abs(euler[0] - (-45)) < 1  # Yaw should be -45°

        # Transform angular velocity
        angular_velocity_frd = rot_FLU_body_to_FRD_body.apply(angular_velocity_flu)
        np.testing.assert_almost_equal(angular_velocity_frd, [0.1, 0, -0.5])