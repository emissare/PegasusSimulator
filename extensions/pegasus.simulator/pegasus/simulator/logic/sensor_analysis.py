"""
| File: sensor_analysis.py
| Description: Helper functions for analyzing sensor data and comparing with truth values.
|              These functions help convert sensor measurements back to comparable values
|              for debugging and validation purposes.
"""

import numpy as np
from typing import List, Tuple


def calculate_magnetic_heading(mag_field_frd: List[float], roll_rad: float, pitch_rad: float) -> float:
    """
    Calculate magnetic heading from magnetometer readings using tilt compensation.

    Args:
        mag_field_frd: Magnetic field vector in FRD body frame [x, y, z] in Gauss
        roll_rad: Roll angle in radians
        pitch_rad: Pitch angle in radians

    Returns:
        Magnetic heading in radians (0 to 2π, where 0 is magnetic north)
    """
    # Extract magnetometer components
    mx, my, mz = mag_field_frd

    # Tilt compensation
    # Rotate magnetometer readings to compensate for roll and pitch
    mx_comp = mx * np.cos(pitch_rad) + my * np.sin(roll_rad) * np.sin(pitch_rad) + mz * np.cos(roll_rad) * np.sin(pitch_rad)
    my_comp = my * np.cos(roll_rad) - mz * np.sin(roll_rad)

    # Calculate magnetic heading
    # atan2 returns angle in [-π, π], convert to [0, 2π]
    heading = np.arctan2(-my_comp, mx_comp)  # Negative because of FRD convention (Y is right)
    if heading < 0:
        heading += 2 * np.pi

    return heading


def calculate_true_heading(magnetic_heading_rad: float, declination_rad: float) -> float:
    """
    Convert magnetic heading to true heading using magnetic declination.

    Args:
        magnetic_heading_rad: Magnetic heading in radians
        declination_rad: Magnetic declination in radians (positive is east)

    Returns:
        True heading in radians (0 to 2π, where 0 is true north)
    """
    true_heading = magnetic_heading_rad + declination_rad

    # Normalize to [0, 2π]
    while true_heading < 0:
        true_heading += 2 * np.pi
    while true_heading >= 2 * np.pi:
        true_heading -= 2 * np.pi

    return true_heading


def calculate_altitude_from_pressure(pressure_pa: float,
                                    pressure_msl_pa: float = 101325.0,
                                    temperature_c: float = 15.0) -> float:
    """
    Calculate altitude from barometric pressure using the barometric formula.

    Args:
        pressure_pa: Current pressure in Pascals
        pressure_msl_pa: Mean sea level pressure in Pascals (default: standard atmosphere)
        temperature_c: Temperature in Celsius (default: 15°C standard)

    Returns:
        Altitude above mean sea level in meters
    """
    # Constants
    LAPSE_RATE = 0.0065  # K/m
    TEMPERATURE_MSL = 288.15  # K (15°C)
    GAS_CONSTANT = 8.31432  # J/(mol·K)
    MOLAR_MASS = 0.0289644  # kg/mol
    GRAVITY = 9.80665  # m/s²

    # Barometric formula
    # h = (T₀/L) * [1 - (P/P₀)^(R*L/(g*M))]
    exponent = (GAS_CONSTANT * LAPSE_RATE) / (GRAVITY * MOLAR_MASS)
    altitude = (TEMPERATURE_MSL / LAPSE_RATE) * (1 - np.power(pressure_pa / pressure_msl_pa, exponent))

    return altitude


def integrate_angular_rates(angular_vel_frd: List[float], dt: float,
                           prev_attitude_quat: List[float]) -> List[float]:
    """
    Integrate angular rates to estimate attitude change.
    This is what an IMU-based attitude estimator would do.

    Args:
        angular_vel_frd: Angular velocity in FRD body frame [p, q, r] in rad/s
        dt: Time step in seconds
        prev_attitude_quat: Previous attitude quaternion [x, y, z, w]

    Returns:
        Estimated new attitude quaternion [x, y, z, w]
    """
    from scipy.spatial.transform import Rotation

    # Extract angular velocities
    p, q, r = angular_vel_frd

    # Create rotation from angular velocity (small angle approximation for small dt)
    # This represents the change in attitude over dt
    angle = np.sqrt(p*p + q*q + r*r) * dt

    if angle > 1e-10:  # Avoid division by zero
        axis = np.array([p, q, r]) / np.sqrt(p*p + q*q + r*r)
        delta_rotation = Rotation.from_rotvec(axis * angle)
    else:
        delta_rotation = Rotation.identity()

    # Apply the rotation change to previous attitude
    prev_rotation = Rotation.from_quat(prev_attitude_quat)
    new_rotation = prev_rotation * delta_rotation

    return new_rotation.as_quat().tolist()


def estimate_velocity_change_from_acceleration(accel_frd: List[float],
                                              attitude_frd_ned_quat: List[float],
                                              dt: float) -> List[float]:
    """
    Estimate velocity change from accelerometer readings.
    Converts body frame accelerations to world frame and integrates.

    Args:
        accel_frd: Linear acceleration in FRD body frame [ax, ay, az] in m/s²
        attitude_frd_ned_quat: Current attitude quaternion (FRD to NED) [x, y, z, w]
        dt: Time step in seconds

    Returns:
        Estimated velocity change in NED frame [dvn, dve, dvd] in m/s
    """
    from scipy.spatial.transform import Rotation

    # Convert acceleration from FRD body to NED world frame
    rotation_frd_to_ned = Rotation.from_quat(attitude_frd_ned_quat)
    accel_ned = rotation_frd_to_ned.apply(accel_frd)

    # Account for gravity (assuming it's already removed from accelerometer reading)
    # In a real IMU, we'd need to add gravity back: accel_ned[2] += 9.81

    # Integrate acceleration to get velocity change
    velocity_change = accel_ned * dt

    return velocity_change.tolist()


def calculate_position_from_gps_geodetic(latitude_deg: float, longitude_deg: float,
                                        altitude_msl_m: float,
                                        origin_lat_deg: float, origin_lon_deg: float,
                                        origin_alt_m: float) -> List[float]:
    """
    Convert GPS geodetic coordinates to local NED position relative to origin.

    Args:
        latitude_deg: Current latitude in degrees
        longitude_deg: Current longitude in degrees
        altitude_msl_m: Current altitude above MSL in meters
        origin_lat_deg: Origin latitude in degrees
        origin_lon_deg: Origin longitude in degrees
        origin_alt_m: Origin altitude above MSL in meters

    Returns:
        Position in local NED frame [north, east, down] in meters
    """
    # Earth radius (approximate)
    EARTH_RADIUS = 6371000.0  # meters

    # Convert to radians
    lat_rad = np.radians(latitude_deg)
    lon_rad = np.radians(longitude_deg)
    origin_lat_rad = np.radians(origin_lat_deg)
    origin_lon_rad = np.radians(origin_lon_deg)

    # Calculate NED position
    # North: positive northward
    north = EARTH_RADIUS * (lat_rad - origin_lat_rad)

    # East: positive eastward (accounting for latitude)
    east = EARTH_RADIUS * np.cos(origin_lat_rad) * (lon_rad - origin_lon_rad)

    # Down: positive downward
    down = -(altitude_msl_m - origin_alt_m)

    return [north, east, down]


def compare_attitudes(quat1: List[float], quat2: List[float]) -> Tuple[float, List[float]]:
    """
    Compare two attitude quaternions and return the angular difference.

    Args:
        quat1: First quaternion [x, y, z, w]
        quat2: Second quaternion [x, y, z, w]

    Returns:
        Tuple of:
        - Angular difference in degrees
        - Euler angle differences [roll, pitch, yaw] in degrees
    """
    from scipy.spatial.transform import Rotation

    rot1 = Rotation.from_quat(quat1)
    rot2 = Rotation.from_quat(quat2)

    # Calculate relative rotation
    rot_diff = rot1.inv() * rot2

    # Get angle of rotation
    angle_rad = rot_diff.magnitude()
    angle_deg = np.degrees(angle_rad)

    # Get Euler angle differences
    euler1 = rot1.as_euler('xyz', degrees=True)
    euler2 = rot2.as_euler('xyz', degrees=True)
    euler_diff = euler2 - euler1

    # Normalize angle differences to [-180, 180]
    for i in range(3):
        while euler_diff[i] > 180:
            euler_diff[i] -= 360
        while euler_diff[i] < -180:
            euler_diff[i] += 360

    return angle_deg, euler_diff.tolist()