"""
Pure magnetometer calculation functions.
No dependencies on Isaac Sim or State class.
"""

import numpy as np
from scipy.spatial.transform import Rotation
from typing import Dict, Optional, Tuple

from pegasus.simulator.logic.rotations import (
    rot_FLU_body_to_FRD_body,
    rot_FLU_inertial_to_NED_inertial
)
from pegasus.simulator.logic.sensors.geo_mag_utils import (
    get_mag_declination,
    get_mag_inclination,
    get_mag_strength
)


def calculate_magnetometer_measurements(
    attitude_flu: np.ndarray,
    latitude: float,
    longitude: float,
    altitude: float,
    magnetometer_bias: Optional[np.ndarray] = None,
    noise_params: Optional[Dict] = None
) -> Dict[str, np.ndarray]:
    """
    Calculate magnetometer measurements from vehicle state.

    Args:
        attitude_flu: Quaternion [x,y,z,w] for FLU body in FLU world
        latitude: Latitude in degrees
        longitude: Longitude in degrees
        altitude: Altitude in meters
        magnetometer_bias: Optional magnetometer bias
        noise_params: Optional noise parameters

    Returns:
        Dictionary containing magnetic field in FRD body frame
    """

    # Initialize bias if not provided
    if magnetometer_bias is None:
        magnetometer_bias = np.zeros(3)

    # Get magnetic field parameters for this location
    declination_rad = get_mag_declination(latitude, longitude)
    inclination_rad = get_mag_inclination(latitude, longitude)
    strength_ga = get_mag_strength(latitude, longitude)

    # Calculate magnetic field components in NED frame
    # H: Horizontal component, Z: Vertical component
    H = strength_ga * np.cos(inclination_rad)
    Z = np.tan(inclination_rad) * H

    # X (North) and Y (East) components
    X = H * np.cos(declination_rad)
    Y = H * np.sin(declination_rad)

    # Magnetic field in NED inertial frame
    magnetic_field_ned = np.array([X, Y, Z])

    # Transform from NED to FLU inertial (inverse of FLU to NED)
    # Since rot_FLU_inertial_to_NED_inertial is 180° around X, its inverse is itself
    magnetic_field_flu = rot_FLU_inertial_to_NED_inertial.inv().apply(magnetic_field_ned)

    # Get the attitude rotation
    attitude_flu_flu = Rotation.from_quat(attitude_flu)

    # Transform to body frame
    # First to FLU body frame
    magnetic_field_flu_body = attitude_flu_flu.inv().apply(magnetic_field_flu)

    # Then from FLU body to FRD body
    magnetic_field_frd_body = rot_FLU_body_to_FRD_body.apply(magnetic_field_flu_body)

    # Add noise and bias
    if noise_params and 'noise' in noise_params:
        magnetic_field_frd_body += noise_params['noise']

    magnetic_field_frd_body += magnetometer_bias

    return {
        "magnetic_field": magnetic_field_frd_body
    }


def calculate_magnetometer_noise(
    dt: float,
    magnetometer_bias: np.ndarray,
    noise_density: float = 0.6e-3,
    random_walk: float = 0.6e-6,
    bias_correlation_time: float = 600.0
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate magnetometer noise and update bias.

    Args:
        dt: Time step in seconds
        magnetometer_bias: Current bias state
        noise_density: Noise density parameter
        random_walk: Random walk parameter
        bias_correlation_time: Bias correlation time

    Returns:
        Tuple of (noise, updated_bias)
    """
    tau = bias_correlation_time

    # Discrete-time standard deviation
    sigma_d = 1 / np.sqrt(dt) * noise_density
    sigma_b = random_walk

    # Compute exact covariance of the process after dt
    sigma_b_d = np.sqrt(-sigma_b * sigma_b * tau / 2.0 * (np.exp(-2.0 * dt / tau) - 1.0))

    # Compute state-transition
    phi_d = np.exp(-1.0 / tau * dt)

    # Generate noise and update bias
    noise = np.zeros(3)
    new_bias = np.zeros(3)

    for i in range(3):
        new_bias[i] = phi_d * magnetometer_bias[i] + sigma_b_d * np.random.randn()
        noise[i] = sigma_d * np.random.randn()

    return noise, new_bias