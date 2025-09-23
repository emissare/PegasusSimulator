"""
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Sensor class which is used as the base for all the sensors.
"""
__all__ = ["Sensor"]

from pegasus.simulator.logic.vehicle_state import VehicleState

class Sensor:
    """The base class for implementing a sensor

    Attributes:
        update_interval_s (float): The time interval between sensor updates in seconds
        origin_lat (float): The latitude of the origin of the world in degrees (might get used by some sensors).
        origin_lon (float): The longitude of the origin of the world in degrees (might get used by some sensors).
        origin_alt (float): The altitude of the origin of the world relative to sea water level (might get used by some sensors)
    """
    def __init__(self, sensor_type: str, update_rate: float):
        """Initialize the Sensor class

        Args:
            sensor_type (str): A name that describes the type of sensor
            update_rate (float): The rate at which the data in the sensor should be refreshed (in Hz)
        """

        # Set the sensor type and update rate
        self._sensor_type = sensor_type
        self._update_rate = update_rate
        self._update_interval_s = 1.0 / self._update_rate

        # Timing control
        self._prev_update_time_s = 0.0
        self._has_new_data = False

        # Set the "configuration of the world" - some sensors might need it
        self._origin_lat = -999
        self._origin_lon = -999
        self._origin_alt = 0.0

        self._vehicle = None

    def initialize(self, vehicle, origin_lat, origin_lon, origin_alt):
        """Method that initializes the sensor latitude, longitude and altitude attributes.

        Note:
            Given that some sensors require the knowledge of the latitude, longitude and altitude of the [0, 0, 0] coordinate
            of the world, then we might as well just save this information for whatever sensor that comes

        Args:
            vehicle (Vehicle): A reference to the vehicle that this sensor is associated with
            origin_lat (float): The latitude of the origin of the world in degrees (might get used by some sensors).
            origin_lon (float): The longitude of the origin of the world in degrees (might get used by some sensors).
            origin_alt (float): The altitude of the origin of the world relative to sea water level (might get used by some sensors).
        """
        self._vehicle = vehicle
        self._origin_lat = origin_lat
        self._origin_lon = origin_lon
        self._origin_alt = origin_alt

    def set_update_rate(self, update_rate: float):
        """Method that changes the update rate and period of the sensor

        Args:
            update_rate (float): The new rate at which the data in the sensor should be refreshed (in Hz)
        """
        self._update_rate = update_rate
        self._update_interval_s = 1.0 / self._update_rate

    def should_update(self, current_time_s: float) -> bool:
        """Check if enough time has passed for a sensor update

        Args:
            current_time_s (float): Current simulation time in seconds

        Returns:
            bool: True if sensor should update, False otherwise
        """
        return (current_time_s - self._prev_update_time_s) >= self._update_interval_s

    def has_new_data(self) -> bool:
        """Check if sensor has new data available

        Returns:
            bool: True if new data is available since last read
        """
        return self._has_new_data

    def clear_new_data_flag(self):
        """Clear the new data flag after reading"""
        self._has_new_data = False

    @property
    def sensor_type(self):
        """
        (str) A name that describes the type of sensor.
        """
        return self._sensor_type

    @property
    def update_rate(self):
        """
        (float) The rate at which the data in the sensor should be refreshed (in Hz).
        """
        return self._update_rate

    @property
    def state(self):
        """
        (object) The sensor state object that contains the data produced by the sensor.
        """
        return None

    def update(self, state: VehicleState, current_time_s: float):
        """Method that should be implemented by the class that inherits Sensor. This is where the actual implementation
        of the sensor should be performed.

        Args:
            state (VehicleState): The current state of the vehicle.
            current_time_s (float): Current simulation time in seconds

        Returns:
            (object) The sensor state object if updated, None if not time to update yet
        """
        pass

    def start(self):
        """Method that when implemented should handle the begining of the simulation of vehicle
        """
        pass

    def stop(self):
        """Method that when implemented should handle the stopping of the simulation of vehicle
        """
        pass

    def reset(self):
        """Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        pass

    def config_from_dict(self, config_dict):
        """Method that should be implemented by the class that inherits Sensor. This is where the configuration of the
        sensor based on a dictionary input should be performed.

        Args:
            config_dict (dict): A dictionary containing the configurations of the sensor
        """
        pass