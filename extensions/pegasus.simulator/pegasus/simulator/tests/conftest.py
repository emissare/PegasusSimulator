"""Pytest configuration and fixtures for Pegasus Simulator tests."""

import sys
import os
import pytest
import numpy as np
from scipy.spatial.transform import Rotation


# ==================== Rotation Fixtures ====================

@pytest.fixture
def rot_NWU_to_NED():
    """Rotation from NWU world frame to NED world frame."""
    # 180° rotation around X-axis (North axis)
    return Rotation.from_quat([1.0, 0.0, 0.0, 0.0])


@pytest.fixture
def rot_FLU_to_FRD():
    """Rotation from FLU body frame to FRD body frame."""
    # 180° rotation around X-axis (Front axis)
    return Rotation.from_quat([1.0, 0.0, 0.0, 0.0])


# ==================== Test Data Fixtures ====================

@pytest.fixture
def identity_quaternion():
    """Identity quaternion (no rotation)."""
    return np.array([0, 0, 0, 1])


@pytest.fixture
def test_quaternions():
    """Common test quaternions with expected yaw values."""
    return [
        {
            "name": "Identity (North)",
            "quat": np.array([0, 0, 0, 1]),
            "expected_yaw": 0,
            "description": "Vehicle pointing North/Forward"
        },
        {
            "name": "90° left (West)",
            "quat": np.array([0, 0, 0.707, 0.707]),
            "expected_yaw": -90,
            "description": "Vehicle pointing West/Left"
        },
        {
            "name": "90° right (East)",
            "quat": np.array([0, 0, -0.707, 0.707]),
            "expected_yaw": 90,
            "description": "Vehicle pointing East/Right"
        },
        {
            "name": "180° (South)",
            "quat": np.array([0, 0, 1, 0]),
            "expected_yaw": 180,
            "description": "Vehicle pointing South/Back"
        }
    ]


@pytest.fixture
def test_velocities():
    """Common test velocities for transformation testing."""
    return [
        {
            "name": "North/Forward",
            "flu": np.array([10, 0, 0]),
            "expected_ned": np.array([10, 0, 0])
        },
        {
            "name": "West/Left",
            "flu": np.array([0, 10, 0]),
            "expected_ned": np.array([0, -10, 0])
        },
        {
            "name": "Up",
            "flu": np.array([0, 0, 10]),
            "expected_ned": np.array([0, 0, -10])
        }
    ]


@pytest.fixture
def test_angular_velocities():
    """Common test angular velocities."""
    return [
        {
            "name": "Roll only",
            "flu": np.array([1.0, 0, 0]),
            "expected_frd": np.array([1.0, 0, 0])
        },
        {
            "name": "Pitch only",
            "flu": np.array([0, 1.0, 0]),
            "expected_frd": np.array([0, -1.0, 0])
        },
        {
            "name": "Yaw only",
            "flu": np.array([0, 0, 1.0]),
            "expected_frd": np.array([0, 0, -1.0])
        }
    ]


@pytest.fixture
def zurich_coordinates():
    """Zurich, Switzerland coordinates for GPS/Mag testing."""
    return {
        "latitude": 47.397742,
        "longitude": 8.545594,
        "altitude": 488.0
    }


# ==================== Helper Fixtures ====================

@pytest.fixture
def gravity_vector():
    """Standard gravity vector."""
    return np.array([0.0, 0.0, -9.80665])


@pytest.fixture
def tolerance():
    """Default numerical tolerance for comparisons."""
    return {
        "angle": 1.0,      # degrees
        "position": 0.01,  # meters
        "velocity": 0.01,  # m/s
        "general": 1e-6
    }


# ==================== Mock Sensor Parameters ====================

@pytest.fixture
def imu_noise_params():
    """Default IMU noise parameters."""
    return {
        "gyroscope": {
            "noise_density": 0.0003393695767766752,
            "random_walk": 3.878509448876288E-05,
            "bias_correlation_time": 1.0E3,
            "turn_on_bias_sigma": 0.008726646259971648
        },
        "accelerometer": {
            "noise_density": 0.004,
            "random_walk": 0.006,
            "bias_correlation_time": 300.0,
            "turn_on_bias_sigma": 0.196
        }
    }


@pytest.fixture
def gps_noise_params():
    """Default GPS noise parameters."""
    return {
        "xy_random_walk": 2.0,
        "z_random_walk": 4.0,
        "xy_noise_density": 2.0e-4,
        "z_noise_density": 4.0e-4,
        "vxy_noise_density": 0.2,
        "vz_noise_density": 0.4,
        "correlation_time": 60.0
    }