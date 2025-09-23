"""
| File: barometer.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates a barometer. Based on the implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo) by Elia Tarasov.
| References: Both the original implementation provided in the gazebo based simulation and this one are based on the following article - 'A brief summary of atmospheric modeling', Cavcar, M., http://fisicaatmo.at.fcen.uba.ar/practicas/ISAweb.pdf
"""

__all__ = ["Barometer"]

import numpy as np
from pegasus.simulator.logic.vehicle_state import VehicleState
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.sensor_models import BarometerState
from pegasus.simulator.logic.sensors.geo_mag_utils import GRAVITY_VECTOR

DEFAULT_HOME_ALT_AMSL = 488.0


class Barometer(Sensor):
    def __init__(self, config={}):
        # Initialize the Super class "object" attributes
        # Default barometer rate: 50 Hz (standard for PX4 SITL)
        super().__init__(
            sensor_type="Barometer", update_rate=config.get("update_rate", 50.0)
        )

        self._z_start: float = None

        # Setup the default home altitude (aka the altitude at the [0.0, 0.0, 0.0] coordinate on the simulated world)
        # If desired, the user can override this default by calling the initialize() method defined inside the Sensor
        # implementation
        self._origin_alt = DEFAULT_HOME_ALT_AMSL

        # Define the constants for the barometer
        # International standard atmosphere (troposphere model - valid up to 11km) see [1]
        self._TEMPERATURE_MSL: float = config.get(
            "temperature_msl", 288.15
        )  # temperature at MSL [K] (15 [C])
        self._PRESSURE_MSL: float = config.get(
            "pressure_msl", 101325.0
        )  # pressure at MSL [Pa]
        self._LAPSE_RATE: float = config.get(
            "lapse_rate", 0.0065
        )  # reduction in temperature with altitude for troposphere [K/m]
        self._AIR_DENSITY_MSL: float = config.get(
            "air_density_msl", 1.225
        )  # air density at MSL [kg/m^3]
        self._ABSOLUTE_ZERO_C: float = config.get("absolute_zero", -273.15)  # [C]

        # Set the drift for the sensor
        self._baro_drift_pa_per_sec: float = config.get("drift_pa_per_sec", 0.0)

        # Auxiliar variables for generating the noise
        self._baro_rnd_use_last: bool = False
        self._baro_rnd_y2: float = 0.0
        self._baro_drift_pa: float = 0.0

        # Initialize the Barometer state with default sea level values
        self._state = BarometerState(
            pressure_pa=self._PRESSURE_MSL,  # Standard sea level pressure
            altitude_msl_m=0.0,
            temperature_celsius=15.0,  # Standard sea level temperature
        )

    @property
    def state(self) -> BarometerState:
        """
        Returns:
            BarometerState: The current barometer sensor state with all measurements properly typed
        """
        return self._state

    def update(self, state: VehicleState, current_time_s: float):
        """Method that implements the logic of a barometer. In this method we compute the relative altitude of the vehicle
        relative to the origin's altitude. Aditionally, we compute the actual altitude of the vehicle, local temperature and
        absolute presure, based on the reference - [A brief summary of atmospheric modeling, Cavcar, M., http://fisicaatmo.at.fcen.uba.ar/practicas/ISAweb.pdf]

        Args:
            state (VehicleState): The current state of the vehicle.
            current_time_s (float): Current simulation time in seconds.

        Returns:
            (BarometerState) The current state of the sensor, or None if not time to update
        """

        # Check if it's time to update
        if not self.should_update(current_time_s):
            return None

        # Calculate dt for this update
        dt = current_time_s - self._prev_update_time_s
        self._prev_update_time_s = current_time_s

        # Get position in NED (already converted by VehicleState)
        # In NED, Z is positive downward, so altitude is negative Z
        if self._z_start is None:
            self._z_start = -state.position_ned_m[2]  # Store initial altitude (negative of down)

        alt_rel: float = -state.position_ned_m[2] - self._z_start
        alt_amsl: float = self._origin_alt + alt_rel
        temperature_local: float = self._TEMPERATURE_MSL - self._LAPSE_RATE * alt_amsl

        pressure_ratio: float = np.power(
            self._TEMPERATURE_MSL / temperature_local, 5.2561
        )
        absolute_pressure: float = self._PRESSURE_MSL / pressure_ratio

        if not self._baro_rnd_use_last:

            w: float = 1.0

            while w >= 1.0:
                x1: float = 2.0 * np.random.randn() - 1.0
                x2: float = 2.0 * np.random.randn() - 1.0
                w = (x1 * x1) + (x2 * x2)

            w = np.sqrt((-2.0 * np.log(w)) / w)
            y1: float = x1 * w
            self._baro_rnd_y2 = x2 * w
            self._baro_rnd_use_last = True
        else:
            y1: float = self._baro_rnd_y2
            self._baro_rnd_use_last = False

        abs_pressure_noise: float = y1  # 1 Pa RMS noise
        self._baro_drift_pa = self._baro_drift_pa + (self._baro_drift_pa_per_sec * dt)
        absolute_pressure_noisy: float = (
            absolute_pressure + abs_pressure_noise + self._baro_drift_pa_per_sec
        )

        absolute_pressure_noisy_hpa: float = absolute_pressure_noisy * 0.01

        density_ratio: float = np.power(
            self._TEMPERATURE_MSL / temperature_local, 4.256
        )
        air_density: float = self._AIR_DENSITY_MSL / density_ratio

        pressure_altitude: float = alt_amsl - (
            abs_pressure_noise + self._baro_drift_pa
        ) / (np.linalg.norm(GRAVITY_VECTOR) * air_density)

        temperature_celsius: float = temperature_local + self._ABSOLUTE_ZERO_C

        self._state = BarometerState(
            pressure_pa=absolute_pressure_noisy,  # In Pascals
            altitude_msl_m=pressure_altitude,
            temperature_celsius=temperature_celsius,
        )

        self._has_new_data = True
        return self._state  # Return the BarometerState object directly
