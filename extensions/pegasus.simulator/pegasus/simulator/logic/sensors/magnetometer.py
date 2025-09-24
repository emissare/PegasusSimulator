"""
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates a magnetometer. Based on the original implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo) by Elia Tarasov
"""
__all__ = ["Magnetometer"]

import numpy as np
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.vehicle_state import VehicleState
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.sensor_models import MagnetometerState
from pegasus.simulator.logic.rotations import (
    rot_NWU_to_NED,
    rot_FLU_to_FRD
)
from pegasus.simulator.logic.sensors.geo_mag_utils import (
    get_mag_declination,
    get_mag_inclination,
    get_mag_strength,
    convert_ned_to_geodetic,
)

class Magnetometer(Sensor):
    """The class that implements a magnetometer sensor. This class inherits the base class Sensor.
    """

    def __init__(self, config={}):
        """Initialize the Magnetometer class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the Magnetometer - it can be empty or only have some of the parameters used by the Magnetometer.
            
        Examples:
            The dictionary default parameters are

            >>> {"noise_density": 0.4e-3,           # gauss / sqrt(hz)
            >>>  "random_walk": 6.4e-6,             # gauss * sqrt(hz)
            >>>  "bias_correlation_time": 6.0e2,    # s
            >>>  "update_rate": 50.0}               # Hz
        """

        # Initialize the Super class "object" attributes
        # Default magnetometer rate: 50 Hz (standard for PX4 SITL)
        super().__init__(sensor_type="Magnetometer", update_rate=config.get("update_rate", 50.0))

        # Set the noise parameters
        self._bias: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._noise_density = config.get("noise_density", 0.4e-3)  # gauss / sqrt(hz)
        self._random_walk = config.get("random_walk", 6.4e-6)  # gauss * sqrt(hz)
        self._bias_correlation_time = config.get("bias_correlation_time", 6.0e2)  # s

        # Initialize the Magnetometer state with zero field
        self._state = MagnetometerState(
            magnetic_field_frd_body_gauss=[0.0, 0.0, 0.0],
            magnetic_field_magnitude_gauss=0.0,
            magnetic_declination_deg=None,
            magnetic_inclination_deg=None
        )

    @property
    def state(self) -> MagnetometerState:
        """
        Returns:
            MagnetometerState: The current magnetometer sensor state with all measurements properly typed
        """
        return self._state

    def update(self, state: VehicleState, current_time_s: float):
        """Method that implements the logic of a magnetometer. In this method we start by computing the projection
        of the vehicle body frame such in the elipsoidal model of the earth in order to get its current latitude and
        longitude. From here the declination and inclination are computed and used to get the strength of the magnetic
        field, expressed in the inertial frame of reference (in ENU convention). This magnetic field is then rotated
        to the body frame such that it becomes expressed in a FRD body frame relative to a NED inertial reference frame.
        (The convention adopted by PX4). Random noise and bias are added to this magnetic field.

        Args:
            state (VehicleState): The current state of the vehicle.
            current_time_s (float): Current simulation time in seconds.

        Returns:
            (MagnetometerState) The current state of the sensor, or None if not time to update
        """

        # Check if it's time to update
        if not self.should_update(current_time_s):
            return None

        # Calculate dt for this update
        dt = current_time_s - self._prev_update_time_s
        self._prev_update_time_s = current_time_s

        # Get position directly in NED (already converted by VehicleState)
        position_ned_m = state.position_ned_m

        latitude, longitude = convert_ned_to_geodetic(
            position_ned_m,
            np.radians(self._origin_lat),
            np.radians(self._origin_lon)
        )

        declination_rad: float = np.radians(get_mag_declination(np.degrees(latitude), np.degrees(longitude)))
        inclination_rad: float = np.radians(get_mag_inclination(np.degrees(latitude), np.degrees(longitude)))

        strength_ga: float = 0.01 * get_mag_strength(np.degrees(latitude), np.degrees(longitude))

        H: float = strength_ga * np.cos(inclination_rad)
        Z: float = np.tan(inclination_rad) * H
        X: float = H * np.cos(declination_rad)
        Y: float = H * np.sin(declination_rad)

        magnetic_field_inertial: np.ndarray = np.array([X, Y, Z])

        # Get attitude from FRD-NED (what VehicleState stores) and convert to what we need
        # VehicleState stores FRD body in NED world, but we need to apply the magnetic field
        attitude_frd_ned = Rotation.from_quat(state.attitude_frd_ned_quat)

        # The magnetic field is in NED frame, so we can directly transform it to FRD body frame
        magnetic_field_body = attitude_frd_ned.inv().apply(magnetic_field_inertial)

        tau = self._bias_correlation_time

        sigma_d: float = 1 / np.sqrt(dt) * self._noise_density
        sigma_b: float = self._random_walk

        sigma_b_d: float = np.sqrt(-sigma_b * sigma_b * tau / 2.0 * (np.exp(-2.0 * dt / tau) - 1.0))

        phi_d: float = np.exp(-1.0 / tau * dt)

        magnetic_field_noisy: np.ndarray = np.zeros((3,))
        for i in range(3):
            self._bias[i] = phi_d * self._bias[i] + sigma_b_d * np.random.randn()
            magnetic_field_noisy[i] = magnetic_field_body[i] + sigma_d * np.random.randn() + self._bias[i]

        self._state = MagnetometerState(
            magnetic_field_frd_body_gauss=magnetic_field_noisy.tolist(),
            magnetic_field_magnitude_gauss=np.linalg.norm(magnetic_field_noisy),
            magnetic_declination_deg=np.degrees(declination_rad),
            magnetic_inclination_deg=np.degrees(inclination_rad),
        )

        self._has_new_data = True
        return self._state  # Return the MagnetometerState object directly
