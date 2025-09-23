"""
Pure GPS calculation functions.
No dependencies on Isaac Sim or State class.
"""

import numpy as np
from typing import Dict, Optional, Tuple

from pegasus.simulator.logic.rotations import rot_FLU_inertial_to_NED_inertial
from pegasus.simulator.logic.sensors.geo_mag_utils import reprojection


def calculate_gps_measurements(
    position_flu: np.ndarray,
    linear_velocity_flu: np.ndarray,
    origin_lat: float,
    origin_lon: float,
    origin_alt: float,
    gps_bias: Optional[np.ndarray] = None,
    noise_params: Optional[Dict] = None
) -> Dict:
    """
    Calculate GPS measurements from vehicle state.

    Args:
        position_flu: Position [x,y,z] in FLU world frame
        linear_velocity_flu: Velocity [vx,vy,vz] in FLU world frame
        origin_lat: Latitude of world origin in degrees
        origin_lon: Longitude of world origin in degrees
        origin_alt: Altitude of world origin in meters
        gps_bias: Optional GPS position bias
        noise_params: Optional noise parameters dictionary

    Returns:
        Dictionary containing GPS measurements
    """

    # Initialize bias if not provided
    if gps_bias is None:
        gps_bias = np.zeros(3)

    # Add noise to position if provided
    noise_pos = noise_params.get('position_noise', np.zeros(3)) if noise_params else np.zeros(3)

    # Apply noise and bias to position
    pos_with_noise = position_flu + noise_pos + gps_bias

    # Reproject position to geographic coordinates
    latitude, longitude = reprojection(
        pos_with_noise,
        np.radians(origin_lat),
        np.radians(origin_lon)
    )

    # Also calculate groundtruth position (without noise)
    latitude_gt, longitude_gt = reprojection(
        position_flu,
        np.radians(origin_lat),
        np.radians(origin_lon)
    )

    # Add noise to velocity if provided
    noise_vel = noise_params.get('velocity_noise', np.zeros(3)) if noise_params else np.zeros(3)
    velocity_with_noise = linear_velocity_flu + noise_vel

    # Transform velocity from FLU inertial to NED inertial
    velocity_ned = rot_FLU_inertial_to_NED_inertial.apply(velocity_with_noise)

    # Compute ground speed (horizontal speed)
    speed = np.linalg.norm(velocity_with_noise[:2])

    # Course over ground (direction of movement)
    # In NED frame: North=X, East=Y
    ve = velocity_ned[1]  # East velocity
    vn = velocity_ned[0]  # North velocity
    cog = np.degrees(np.arctan2(ve, vn))

    if cog < 0.0:
        cog = cog + 360.0

    cog = cog * 100  # Convert to centidegrees

    return {
        "latitude": np.degrees(latitude),
        "longitude": np.degrees(longitude),
        "altitude": position_flu[2] + origin_alt - noise_pos[2] + gps_bias[2],
        "velocity_north": velocity_ned[0],
        "velocity_east": velocity_ned[1],
        "velocity_down": velocity_ned[2],
        "speed": speed,
        "cog": cog,
        "latitude_gt": np.degrees(latitude_gt),
        "longitude_gt": np.degrees(longitude_gt),
        "altitude_gt": position_flu[2] + origin_alt
    }


def calculate_gps_noise(
    dt: float,
    gps_bias: np.ndarray,
    xy_random_walk: float = 2.0,
    z_random_walk: float = 4.0,
    xy_noise_density: float = 2.0e-4,
    z_noise_density: float = 4.0e-4,
    vxy_noise_density: float = 0.2,
    vz_noise_density: float = 0.4,
    correlation_time: float = 60.0
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Calculate GPS noise for position and velocity.

    Args:
        dt: Time step in seconds
        gps_bias: Current GPS bias
        xy_random_walk: XY random walk parameter (m/s)/sqrt(Hz)
        z_random_walk: Z random walk parameter (m/s)/sqrt(Hz)
        xy_noise_density: XY position noise density (m)/sqrt(Hz)
        z_noise_density: Z position noise density (m)/sqrt(Hz)
        vxy_noise_density: XY velocity noise density (m/s)/sqrt(Hz)
        vz_noise_density: Z velocity noise density (m/s)/sqrt(Hz)
        correlation_time: GPS bias correlation time (s)

    Returns:
        Tuple of (position_noise, velocity_noise, random_walk, updated_bias)
    """

    # Generate random walk
    random_walk = np.array([
        xy_random_walk * np.sqrt(dt) * np.random.randn(),
        xy_random_walk * np.sqrt(dt) * np.random.randn(),
        z_random_walk * np.sqrt(dt) * np.random.randn()
    ])

    # Generate position noise
    position_noise = np.array([
        xy_noise_density * np.sqrt(dt) * np.random.randn(),
        xy_noise_density * np.sqrt(dt) * np.random.randn(),
        z_noise_density * np.sqrt(dt) * np.random.randn()
    ])

    # Generate velocity noise
    velocity_noise = np.array([
        vxy_noise_density * np.sqrt(dt) * np.random.randn(),
        vxy_noise_density * np.sqrt(dt) * np.random.randn(),
        vz_noise_density * np.sqrt(dt) * np.random.randn()
    ])

    # Update GPS bias with random walk and decay
    updated_bias = np.zeros(3)
    for i in range(3):
        updated_bias[i] = (
            gps_bias[i] + random_walk[i] * dt - gps_bias[i] / correlation_time
        )

    return position_noise, velocity_noise, random_walk, updated_bias