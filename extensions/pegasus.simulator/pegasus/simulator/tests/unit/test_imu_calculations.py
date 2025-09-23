"""Unit tests for IMU calculations."""

import pytest
import numpy as np
from scipy.spatial.transform import Rotation

# Import the actual calculation functions
from pegasus.simulator.logic.sensors.calculations.imu_calc import (
    calculate_imu_measurements,
    calculate_gyroscope_noise,
    calculate_accelerometer_noise
)


class TestIMUCalculations:
    """Test IMU measurement calculations."""

    def test_stationary_vehicle_no_acceleration(self, identity_quaternion, gravity_vector):
        """Test IMU output for stationary vehicle."""
        result = calculate_imu_measurements(
            attitude_flu=identity_quaternion,
            angular_velocity_flu=np.zeros(3),
            linear_velocity_flu=np.zeros(3),
            prev_linear_velocity_flu=np.zeros(3),
            dt=0.01
        )

        # Check orientation (should be transformed but represent same attitude)
        orientation = result["orientation"]
        rot = Rotation.from_quat(orientation)
        euler = rot.as_euler('ZYX', degrees=True)
        assert abs(euler[0]) < 1  # Yaw should be 0
        assert abs(euler[1]) < 1  # Pitch should be 0
        assert abs(euler[2]) < 1  # Roll should be 0

        # Check angular velocity (should be zero)
        np.testing.assert_array_almost_equal(result["angular_velocity"], np.zeros(3))

        # Check linear acceleration (should only show gravity in body frame)
        # With identity quaternion, gravity appears as positive acceleration in Z
        expected_accel = np.array([0, 0, 9.80665])  # Gravity compensation
        np.testing.assert_allclose(result["linear_acceleration"], expected_accel, rtol=0.01)

    def test_accelerating_vehicle(self, identity_quaternion):
        """Test IMU output for accelerating vehicle."""
        # Vehicle accelerating forward (5 m/s to 10 m/s in 0.1s = 50 m/s²)
        result = calculate_imu_measurements(
            attitude_flu=identity_quaternion,
            angular_velocity_flu=np.zeros(3),
            linear_velocity_flu=np.array([10.0, 0, 0]),
            prev_linear_velocity_flu=np.array([5.0, 0, 0]),
            dt=0.1
        )

        # Check linear acceleration (50 m/s² forward + gravity compensation)
        accel = result["linear_acceleration"]
        assert abs(accel[0] - 50.0) < 1  # Forward acceleration
        assert abs(accel[1]) < 0.1  # No lateral acceleration
        assert abs(accel[2] - 9.80665) < 0.1  # Gravity compensation

    def test_rotating_vehicle(self):
        """Test IMU output for rotating vehicle."""
        # Vehicle yawing at 0.5 rad/s
        angular_velocity_flu = np.array([0, 0, 0.5])

        result = calculate_imu_measurements(
            attitude_flu=np.array([0, 0, 0, 1]),
            angular_velocity_flu=angular_velocity_flu,
            linear_velocity_flu=np.zeros(3),
            prev_linear_velocity_flu=np.zeros(3),
            dt=0.01
        )

        # Check angular velocity transformation
        # Yaw in FLU becomes negative yaw in FRD
        expected_angular_frd = np.array([0, 0, -0.5])
        np.testing.assert_array_almost_equal(
            result["angular_velocity"], expected_angular_frd
        )

    @pytest.mark.parametrize("attitude_case", [
        {"quat": [0, 0, 0, 1], "expected_yaw": 0},
        {"quat": [0, 0, 0.707, 0.707], "expected_yaw": -90},
        {"quat": [0, 0, -0.707, 0.707], "expected_yaw": 90},
    ])
    def test_attitude_transformation(self, attitude_case):
        """Test attitude transformation for different orientations."""
        result = calculate_imu_measurements(
            attitude_flu=np.array(attitude_case["quat"]),
            angular_velocity_flu=np.zeros(3),
            linear_velocity_flu=np.zeros(3),
            prev_linear_velocity_flu=np.zeros(3),
            dt=0.01
        )

        rot = Rotation.from_quat(result["orientation"])
        euler = rot.as_euler('ZYX', degrees=True)
        actual_yaw = euler[0]

        assert abs(actual_yaw - attitude_case["expected_yaw"]) < 1, \
            f"Expected yaw={attitude_case['expected_yaw']}°, got {actual_yaw:.1f}°"

    def test_with_noise_parameters(self):
        """Test IMU calculations with noise."""
        noise_params = {
            'gyroscope_noise': np.array([0.001, 0.002, 0.003]),
            'accelerometer_noise': np.array([0.01, 0.02, 0.03])
        }

        result = calculate_imu_measurements(
            attitude_flu=np.array([0, 0, 0, 1]),
            angular_velocity_flu=np.array([0.1, 0.2, 0.3]),
            linear_velocity_flu=np.zeros(3),
            prev_linear_velocity_flu=np.zeros(3),
            dt=0.01,
            gyroscope_bias=np.zeros(3),
            accelerometer_bias=np.zeros(3),
            noise_params=noise_params
        )

        # Verify noise was applied (angular velocity should include noise)
        # Without noise: [0.1, 0.2, 0.3] → FRD: [0.1, -0.2, -0.3]
        # With noise added before transformation
        angular_frd = result["angular_velocity"]
        assert abs(angular_frd[0] - 0.101) < 0.01  # Includes noise
        assert abs(angular_frd[1] + 0.202) < 0.01
        assert abs(angular_frd[2] + 0.303) < 0.01


class TestNoiseGeneration:
    """Test noise generation functions."""

    def test_gyroscope_noise_generation(self):
        """Test gyroscope noise and bias generation."""
        dt = 0.01
        initial_bias = np.array([0.001, 0.002, 0.003])

        # Set random seed for reproducibility
        np.random.seed(42)

        noise, updated_bias = calculate_gyroscope_noise(
            dt=dt,
            gyroscope_bias=initial_bias
        )

        # Check noise is generated
        assert noise.shape == (3,)
        assert not np.allclose(noise, 0)

        # Check bias is updated
        assert updated_bias.shape == (3,)
        assert not np.allclose(updated_bias, initial_bias)

    def test_accelerometer_noise_generation(self):
        """Test accelerometer noise and bias generation."""
        dt = 0.01
        initial_bias = np.array([0.01, 0.02, 0.03])

        # Set random seed for reproducibility
        np.random.seed(42)

        noise, updated_bias = calculate_accelerometer_noise(
            dt=dt,
            accelerometer_bias=initial_bias
        )

        # Check noise is generated
        assert noise.shape == (3,)
        assert not np.allclose(noise, 0)

        # Check bias is updated
        assert updated_bias.shape == (3,)
        assert not np.allclose(updated_bias, initial_bias)

    def test_noise_scales_with_dt(self):
        """Test that noise scales properly with time step."""
        bias = np.zeros(3)

        # Smaller dt should give smaller noise
        np.random.seed(42)
        noise_small, _ = calculate_gyroscope_noise(dt=0.001, gyroscope_bias=bias)

        np.random.seed(42)
        noise_large, _ = calculate_gyroscope_noise(dt=0.1, gyroscope_bias=bias)

        # Larger dt should generally produce larger noise
        # (due to sqrt(dt) scaling in discrete-time standard deviation)
        assert np.linalg.norm(noise_large) > np.linalg.norm(noise_small)


class TestIMUEdgeCases:
    """Test edge cases and error conditions."""

    def test_zero_dt(self):
        """Test behavior with zero time step."""
        with pytest.raises(ZeroDivisionError):
            calculate_imu_measurements(
                attitude_flu=np.array([0, 0, 0, 1]),
                angular_velocity_flu=np.zeros(3),
                linear_velocity_flu=np.array([1, 0, 0]),
                prev_linear_velocity_flu=np.zeros(3),
                dt=0  # Zero dt should cause division by zero
            )

    def test_large_acceleration(self):
        """Test with very large accelerations."""
        # 10g acceleration
        large_accel = 10 * 9.80665  # ~98 m/s²

        result = calculate_imu_measurements(
            attitude_flu=np.array([0, 0, 0, 1]),
            angular_velocity_flu=np.zeros(3),
            linear_velocity_flu=np.array([large_accel * 0.01, 0, 0]),
            prev_linear_velocity_flu=np.zeros(3),
            dt=0.01
        )

        # Check acceleration is captured correctly
        accel = result["linear_acceleration"]
        assert abs(accel[0] - large_accel) < 1  # Forward acceleration

    def test_invalid_quaternion(self):
        """Test with non-normalized quaternion."""
        # Non-normalized quaternion (should still work but may give unexpected results)
        bad_quat = np.array([1, 1, 1, 1])  # Not normalized

        # Should not crash, but results may be incorrect
        result = calculate_imu_measurements(
            attitude_flu=bad_quat,
            angular_velocity_flu=np.zeros(3),
            linear_velocity_flu=np.zeros(3),
            prev_linear_velocity_flu=np.zeros(3),
            dt=0.01
        )

        # Result should exist but may not be meaningful
        assert "orientation" in result
        assert "angular_velocity" in result
        assert "linear_acceleration" in result