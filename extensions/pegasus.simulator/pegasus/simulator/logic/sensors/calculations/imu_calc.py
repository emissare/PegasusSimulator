"""
Pure IMU calculation functions.
No dependencies on Isaac Sim or State class.
"""

import numpy as np
from scipy.spatial.transform import Rotation
from typing import Dict, Optional, Tuple

# Import the rotation matrices (these are just scipy Rotation objects)
from pegasus.simulator.logic.rotations import (
    rot_FLU_body_to_FRD_body,
    rot_FLU_inertial_to_NED_inertial
)
from pegasus.simulator.logic.sensors.geo_mag_utils import GRAVITY_VECTOR


def calculate_imu_measurements(
    attitude_flu: np.ndarray,
    angular_velocity_flu: np.ndarray,
    linear_velocity_flu: np.ndarray,
    prev_linear_velocity_flu: np.ndarray,
    dt: float,
    gyroscope_bias: Optional[np.ndarray] = None,
    accelerometer_bias: Optional[np.ndarray] = None,
    noise_params: Optional[Dict] = None
) -> Dict[str, np.ndarray]:
    """
    Calculate IMU measurements from vehicle state.

    Args:
        attitude_flu: Quaternion [x,y,z,w] for FLU body in FLU world
        angular_velocity_flu: Angular velocity [p,q,r] in FLU body frame
        linear_velocity_flu: Linear velocity [vx,vy,vz] in FLU world frame
        prev_linear_velocity_flu: Previous linear velocity in FLU world frame
        dt: Time step in seconds
        gyroscope_bias: Optional gyroscope bias vector
        accelerometer_bias: Optional accelerometer bias vector
        noise_params: Optional noise parameters dictionary

    Returns:
        Dictionary containing:
        - orientation: Quaternion [x,y,z,w] for FRD body in NED world
        - angular_velocity: Angular velocity [p,q,r] in FRD body frame
        - linear_acceleration: Linear acceleration [ax,ay,az] in FRD body frame
    """

    # Initialize biases if not provided
    if gyroscope_bias is None:
        gyroscope_bias = np.zeros(3)
    if accelerometer_bias is None:
        accelerometer_bias = np.zeros(3)

    # Apply gyroscope noise and bias
    angular_velocity_with_noise = angular_velocity_flu + gyroscope_bias
    if noise_params and 'gyroscope_noise' in noise_params:
        angular_velocity_with_noise += noise_params['gyroscope_noise']

    # Calculate linear acceleration in world frame
    linear_acceleration_inertial = (linear_velocity_flu - prev_linear_velocity_flu) / dt
    # Remove gravity (gravity points down in world frame)
    linear_acceleration_inertial = linear_acceleration_inertial - GRAVITY_VECTOR

    # Transform acceleration from world frame to body frame
    attitude_rotation = Rotation.from_quat(attitude_flu)
    linear_acceleration_body_flu = attitude_rotation.inv().apply(linear_acceleration_inertial)

    # Apply accelerometer noise and bias
    linear_acceleration_with_noise = linear_acceleration_body_flu + accelerometer_bias
    if noise_params and 'accelerometer_noise' in noise_params:
        linear_acceleration_with_noise += noise_params['accelerometer_noise']

    # Transform orientation from FLU/FLU to FRD/NED
    attitude_flu_flu = Rotation.from_quat(attitude_flu)
    attitude_frd_ned = rot_FLU_inertial_to_NED_inertial * attitude_flu_flu * rot_FLU_body_to_FRD_body

    # Transform angular velocity from FLU body to FRD body
    angular_velocity_frd = rot_FLU_body_to_FRD_body.apply(angular_velocity_with_noise)

    # Transform linear acceleration from FLU body to FRD body
    linear_acceleration_frd = rot_FLU_body_to_FRD_body.apply(linear_acceleration_with_noise)

    return {
        "orientation": attitude_frd_ned.as_quat(),
        "angular_velocity": angular_velocity_frd,
        "linear_acceleration": linear_acceleration_frd
    }


def calculate_gyroscope_noise(
    dt: float,
    gyroscope_bias: np.ndarray,
    noise_density: float = 0.0003393695767766752,
    random_walk: float = 3.878509448876288E-05,
    bias_correlation_time: float = 1.0E3
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate gyroscope noise and update bias.

    Args:
        dt: Time step in seconds
        gyroscope_bias: Current bias state
        noise_density: Noise density parameter
        random_walk: Random walk parameter
        bias_correlation_time: Bias correlation time

    Returns:
        Tuple of (noise, updated_bias)
    """
    tau_g = bias_correlation_time

    # Discrete-time standard deviation
    sigma_g_d = 1 / np.sqrt(dt) * noise_density
    sigma_b_g = random_walk

    # Compute exact covariance of the process after dt
    sigma_b_g_d = np.sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 * (np.exp(-2.0 * dt / tau_g) - 1.0))

    # Compute state-transition
    phi_g_d = np.exp(-1.0 / tau_g * dt)

    # Generate noise and update bias
    noise = np.zeros(3)
    new_bias = np.zeros(3)

    for i in range(3):
        new_bias[i] = phi_g_d * gyroscope_bias[i] + sigma_b_g_d * np.random.randn()
        noise[i] = sigma_g_d * np.random.randn()

    return noise, new_bias


def calculate_accelerometer_noise(
    dt: float,
    accelerometer_bias: np.ndarray,
    noise_density: float = 0.004,
    random_walk: float = 0.006,
    bias_correlation_time: float = 300.0
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate accelerometer noise and update bias.

    Args:
        dt: Time step in seconds
        accelerometer_bias: Current bias state
        noise_density: Noise density parameter
        random_walk: Random walk parameter
        bias_correlation_time: Bias correlation time

    Returns:
        Tuple of (noise, updated_bias)
    """
    tau_a = bias_correlation_time

    # Discrete-time standard deviation
    sigma_a_d = 1.0 / np.sqrt(dt) * noise_density
    sigma_b_a = random_walk

    # Compute exact covariance of the process after dt
    sigma_b_a_d = np.sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 * (np.exp(-2.0 * dt / tau_a) - 1.0))

    # Compute state-transition
    phi_a_d = np.exp(-1.0 / tau_a * dt)

    # Generate noise and update bias
    noise = np.zeros(3)
    new_bias = np.zeros(3)

    for i in range(3):
        new_bias[i] = phi_a_d * accelerometer_bias[i] + sigma_b_a_d * np.random.randn()
        noise[i] = sigma_a_d * np.random.randn()

    return noise, new_bias