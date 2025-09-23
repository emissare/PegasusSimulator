"""
| File: gps.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates a gps. Based on the implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo) by Amy Wagoner and Nuno Marques
"""

__all__ = ["GPS"]

import numpy as np
from pegasus.simulator.logic.vehicle_state import VehicleState
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.geo_mag_utils import convert_ned_to_geodetic
from pegasus.simulator.logic.sensors.sensor_models import GPSState

# TODO - Introduce delay on the GPS data


class GPS(Sensor):
    def __init__(self, config={}):
        # Initialize the Super class "object" attributes
        # Default GPS rate: 10 Hz (standard for PX4 SITL)
        super().__init__(
            sensor_type="GPS", update_rate=config.get("update_rate", 10.0)
        )

        # Define the GPS simulated/fixed values
        self._fix_type = config.get("fix_type", 3)
        self._eph = config.get("eph", 1.0)
        self._epv = config.get("epv", 1.0)
        self._sattelites_visible = config.get("sattelites_visible", 10)

        # Parameters for GPS random walk
        self._random_walk_gps = np.array([0.0, 0.0, 0.0])
        self._gps_xy_random_walk = config.get(
            "gps_xy_random_walk", 2.0
        )  # (m/s) / sqrt(hz)
        self._gps_z_random_walk = config.get(
            "gps_z_random_walk", 4.0
        )  # (m/s) / sqrt(hz)

        # Parameters for the position noise
        self._noise_gps_pos = np.array([0.0, 0.0, 0.0])
        self._gps_xy_noise_density = config.get(
            "gps_xy_noise_density", 2.0e-4
        )  # (m) / sqrt(hz)
        self._gps_z_noise_density = config.get(
            "gps_z_noise_density", 4.0e-4
        )  # (m) / sqrt(hz)

        # Parameters for the velocity noise
        self._noise_gps_vel = np.array([0.0, 0.0, 0.0])
        self._gps_vxy_noise_density = config.get(
            "gps_vxy_noise_density", 0.2
        )  # (m/s) / sqrt(hz)
        self._gps_vz_noise_density = config.get(
            "gps_vz_noise_density", 0.4
        )  # (m/s) / sqrt(hz)

        # Parameters for the GPS bias
        self._gps_bias = np.array([0.0, 0.0, 0.0])
        self._gps_correlation_time = config.get("gps_correlation_time", 60)

        # Initialize the GPS state with origin values
        self._state = GPSState(
            latitude_deg=self._origin_lat,
            longitude_deg=self._origin_lon,
            altitude_msl_m=self._origin_alt,
            velocity_north_mps=0.0,
            velocity_east_mps=0.0,
            velocity_down_mps=0.0,
            ground_speed_mps=0.0,
            course_over_ground_cdeg=0,
            horizontal_position_error_m=self._eph,
            vertical_position_error_m=self._epv,
            fix_type=self._fix_type,
            satellites_visible=self._sattelites_visible,
            latitude_groundtruth_deg=self._origin_lat,
            longitude_groundtruth_deg=self._origin_lon,
            altitude_groundtruth_msl_m=self._origin_alt,
        )

    @property
    def state(self) -> GPSState:
        return self._state

    def update(self, state: VehicleState, current_time_s: float):
        # Check if it's time to update
        if not self.should_update(current_time_s):
            return None

        # Calculate dt for this update
        dt = current_time_s - self._prev_update_time_s
        self._prev_update_time_s = current_time_s

        self._random_walk_gps[0] = (
            self._gps_xy_random_walk * np.sqrt(dt) * np.random.randn()
        )
        self._random_walk_gps[1] = (
            self._gps_xy_random_walk * np.sqrt(dt) * np.random.randn()
        )
        self._random_walk_gps[2] = (
            self._gps_z_random_walk * np.sqrt(dt) * np.random.randn()
        )

        self._noise_gps_pos[0] = (
            self._gps_xy_noise_density * np.sqrt(dt) * np.random.randn()
        )
        self._noise_gps_pos[1] = (
            self._gps_xy_noise_density * np.sqrt(dt) * np.random.randn()
        )
        self._noise_gps_pos[2] = (
            self._gps_z_noise_density * np.sqrt(dt) * np.random.randn()
        )

        self._noise_gps_vel[0] = (
            self._gps_vxy_noise_density * np.sqrt(dt) * np.random.randn()
        )
        self._noise_gps_vel[1] = (
            self._gps_vxy_noise_density * np.sqrt(dt) * np.random.randn()
        )
        self._noise_gps_vel[2] = (
            self._gps_vz_noise_density * np.sqrt(dt) * np.random.randn()
        )

        # Perform GPS bias integration (using euler integration -> to be improved)
        self._gps_bias[0] = (
            self._gps_bias[0]
            + self._random_walk_gps[0] * dt
            - self._gps_bias[0] / self._gps_correlation_time
        )
        self._gps_bias[1] = (
            self._gps_bias[1]
            + self._random_walk_gps[1] * dt
            - self._gps_bias[1] / self._gps_correlation_time
        )
        self._gps_bias[2] = (
            self._gps_bias[2]
            + self._random_walk_gps[2] * dt
            - self._gps_bias[2] / self._gps_correlation_time
        )

        # Get NED position directly from state (already converted)
        position_ned_m = state.position_ned_m

        # Add noise and bias in NED frame
        position_with_noise_ned_m = (
            position_ned_m  # + self._noise_gps_pos + self._gps_bias
        )

        # Ground truth position (without noise)
        position_groundtruth_ned_m = position_ned_m

        # Convert NED positions to geodetic coordinates (latitude/longitude)
        latitude_rad, longitude_rad = convert_ned_to_geodetic(
            position_with_noise_ned_m,
            np.radians(self._origin_lat),
            np.radians(self._origin_lon),
        )

        # Compute the groundtruth latitude and longitude (without noise)
        latitude_groundtruth_rad, longitude_groundtruth_rad = convert_ned_to_geodetic(
            position_groundtruth_ned_m,
            np.radians(self._origin_lat),
            np.radians(self._origin_lon),
        )

        # Get NED velocity directly from state (already converted)
        velocity_ned_mps = state.velocity_ned_mps

        # Add noise to velocity
        velocity_with_noise_ned_mps = velocity_ned_mps + self._noise_gps_vel

        # Compute ground speed (horizontal speed in m/s)
        ground_speed_mps: float = np.linalg.norm(velocity_with_noise_ned_mps[:2])

        # Course over ground (direction of movement, not heading)
        # Calculated from NED velocities: atan2(East, North)
        # Output in centidegrees (0-35999)
        course_over_ground_rad = np.arctan2(
            velocity_with_noise_ned_mps[1], velocity_with_noise_ned_mps[0]
        )
        course_over_ground_deg = np.degrees(course_over_ground_rad)

        if course_over_ground_deg < 0.0:
            course_over_ground_deg = course_over_ground_deg + 360.0

        course_over_ground_cdeg = course_over_ground_deg * 100  # centidegrees

        # Altitude is negative of NED down coordinate
        altitude_with_noise_m = (
            -position_with_noise_ned_m[2]  # Negative because NED down is positive
            + self._origin_alt
        )
        altitude_groundtruth_m = -position_ned_m[2] + self._origin_alt

        self._state = GPSState(
            latitude_deg=np.degrees(latitude_rad),
            longitude_deg=np.degrees(longitude_rad),
            altitude_msl_m=altitude_with_noise_m,
            velocity_north_mps=velocity_with_noise_ned_mps[0],
            velocity_east_mps=velocity_with_noise_ned_mps[1],
            velocity_down_mps=velocity_with_noise_ned_mps[2],
            ground_speed_mps=ground_speed_mps,
            course_over_ground_cdeg=int(course_over_ground_cdeg),
            horizontal_position_error_m=self._eph,
            vertical_position_error_m=self._epv,
            fix_type=self._fix_type,
            satellites_visible=self._sattelites_visible,
            latitude_groundtruth_deg=np.degrees(latitude_groundtruth_rad),
            longitude_groundtruth_deg=np.degrees(longitude_groundtruth_rad),
            altitude_groundtruth_msl_m=altitude_groundtruth_m,
        )

        self._has_new_data = True
        return self._state  # Return the GPSState object directly
