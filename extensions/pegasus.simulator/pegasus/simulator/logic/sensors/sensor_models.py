"""
Pydantic models for sensor state data.
All models include units and coordinate system information in field names.
"""

from typing import List, Optional
from pydantic import BaseModel, Field


class GPSState(BaseModel):
    """GPS sensor state with all measurements properly labeled."""

    # Position (geodetic coordinates)
    latitude_deg: float = Field(description="Latitude in degrees")
    longitude_deg: float = Field(description="Longitude in degrees")
    altitude_msl_m: float = Field(description="Altitude above mean sea level in meters")

    # Velocities in NED (North-East-Down) frame
    velocity_north_mps: float = Field(description="North velocity in meters per second")
    velocity_east_mps: float = Field(description="East velocity in meters per second")
    velocity_down_mps: float = Field(description="Down velocity in meters per second")

    # Speed and course
    ground_speed_mps: float = Field(description="Ground speed (horizontal) in meters per second")
    course_over_ground_cdeg: int = Field(
        description="Course over ground in centidegrees (0-35999), direction of movement not heading"
    )

    # Accuracy and status
    horizontal_position_error_m: float = Field(
        description="Horizontal position error in meters (EPH)"
    )
    vertical_position_error_m: float = Field(
        description="Vertical position error in meters (EPV)"
    )
    fix_type: int = Field(description="GPS fix type (0=no fix, 1=dead reckoning, 2=2D, 3=3D)")
    satellites_visible: int = Field(description="Number of visible satellites")

    # Ground truth values (for simulation/testing)
    latitude_groundtruth_deg: Optional[float] = Field(
        None, description="Ground truth latitude in degrees (without noise)"
    )
    longitude_groundtruth_deg: Optional[float] = Field(
        None, description="Ground truth longitude in degrees (without noise)"
    )
    altitude_groundtruth_msl_m: Optional[float] = Field(
        None, description="Ground truth altitude MSL in meters (without noise)"
    )


class IMUState(BaseModel):
    """IMU sensor state with orientation, angular velocity, and linear acceleration."""

    # Orientation quaternion representing FRD body frame in NED inertial frame
    orientation_quat_frd_ned: List[float] = Field(
        description="Orientation quaternion [qx, qy, qz, qw] representing FRD body in NED inertial"
    )

    # Angular velocity in FRD body frame
    angular_velocity_frd_body_rps: List[float] = Field(
        description="Angular velocity [p, q, r] in FRD body frame, radians per second"
    )

    # Linear acceleration in FRD body frame (includes gravity)
    linear_acceleration_frd_body_mpss: List[float] = Field(
        description="Linear acceleration [ax, ay, az] in FRD body frame, meters per second squared"
    )


class BarometerState(BaseModel):
    """Barometer sensor state with pressure and altitude measurements."""

    # Pressure measurement
    pressure_pa: float = Field(description="Atmospheric pressure in Pascals")

    # Altitude derived from pressure
    altitude_msl_m: float = Field(description="Altitude above mean sea level in meters")

    # Temperature (if available)
    temperature_celsius: Optional[float] = Field(
        None, description="Temperature in degrees Celsius"
    )


class MagnetometerState(BaseModel):
    """Magnetometer sensor state with magnetic field measurements."""

    # Magnetic field in FRD body frame
    magnetic_field_frd_body_gauss: List[float] = Field(
        description="Magnetic field [mx, my, mz] in FRD body frame, Gauss"
    )

    # Magnetic field magnitude
    magnetic_field_magnitude_gauss: float = Field(
        description="Magnitude of magnetic field in Gauss"
    )

    # Declination and inclination (if calculated)
    magnetic_declination_deg: Optional[float] = Field(
        None, description="Magnetic declination in degrees"
    )
    magnetic_inclination_deg: Optional[float] = Field(
        None, description="Magnetic inclination in degrees"
    )


class SimulationState(BaseModel):
    """Groundtruth simulation state for mavlink transmission."""

    # Attitude quaternion in mavlink format [qw, qx, qy, qz]
    attitude_quat_wxyz_frd_ned: List[float] = Field(
        description="Attitude quaternion [qw, qx, qy, qz] in mavlink format, FRD body in NED inertial"
    )

    # Angular velocity in FRD body frame
    angular_velocity_frd_body_rps: List[float] = Field(
        description="Angular velocity [p, q, r] in FRD body frame, radians per second"
    )

    # Linear acceleration in NED frame
    acceleration_ned_mpss: List[float] = Field(
        description="Linear acceleration [ax, ay, az] in NED frame, meters per second squared"
    )

    # Linear velocity in NED frame
    velocity_ned_mps: List[float] = Field(
        description="Linear velocity [vn, ve, vd] in NED frame, meters per second"
    )

    # Position (geodetic)
    latitude_deg: float = Field(description="Latitude in degrees")
    longitude_deg: float = Field(description="Longitude in degrees")
    altitude_msl_m: float = Field(description="Altitude above mean sea level in meters")

    # Airspeed
    indicated_airspeed_mps: float = Field(
        description="Indicated airspeed in meters per second"
    )
    true_airspeed_mps: float = Field(
        description="True airspeed in meters per second"
    )