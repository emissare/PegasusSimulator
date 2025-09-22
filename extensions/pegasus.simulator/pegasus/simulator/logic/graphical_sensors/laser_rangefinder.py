"""
| File: laser_rangefinder.py
| Author: EmissarePegasusSimulator Team
| License: BSD-3-Clause. Copyright (c) 2024. All rights reserved.
| Description: Simulated laser rangefinder sensor using raycasting
"""
__all__ = ["LaserRangefinder"]

import time
import numpy as np
from scipy.spatial.transform import Rotation

# Isaac Sim imports
from omni.physx import get_physx_interface
from omni.usd import get_context
from pxr import Gf

# Pegasus imports
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.graphical_sensors import GraphicalSensor


class LaserRangefinder(GraphicalSensor):
    """
    Simulated laser rangefinder sensor using Isaac Sim's physics raycasting.
    Provides distance measurements for terrain following and obstacle detection.
    """

    def __init__(self, sensor_name: str, config={}):
        """
        Initialize the LaserRangefinder sensor.

        Args:
            sensor_name (str): Name of the sensor
            config (dict): Configuration dictionary with sensor parameters

        Configuration parameters:
            position (array): Sensor position relative to mount [x, y, z]
            orientation (array): Sensor orientation in Euler angles [rx, ry, rz] (degrees)
            max_range (float): Maximum measurement range in meters
            min_range (float): Minimum measurement range in meters
            frequency (float): Measurement frequency in Hz
            beam_width (float): Beam divergence angle in radians
            accuracy (float): Measurement accuracy/noise in meters
        """

        # Initialize the GraphicalSensor base class
        super().__init__(
            sensor_type="LaserRangefinder",
            update_rate=config.get("frequency", 10.0)
        )

        # Sensor configuration
        self._sensor_name = sensor_name
        self._position = np.array(config.get("position", [0.0, 0.0, 0.0]))
        self._orientation = np.array(config.get("orientation", [0.0, 0.0, 0.0]))
        self._max_range = config.get("max_range", 100.0)
        self._min_range = config.get("min_range", 0.1)
        self._frequency = config.get("frequency", 10.0)
        self._beam_width = config.get("beam_width", 0.01)  # Beam divergence in radians
        self._accuracy = config.get("accuracy", 0.02)      # Measurement noise in meters

        # Sensor state
        self._stage_prim_path = ""
        self._state = {
            'distance': 0.0,
            'timestamp': 0.0,
            'valid': False
        }

        # Physics interface for raycasting
        self._physx_interface = None

        # Transform matrices
        self._local_transform = self._compute_local_transform()

    def _compute_local_transform(self):
        """Compute local transform matrix from position and orientation."""

        # Create rotation matrix from Euler angles
        rotation = Rotation.from_euler('xyz', self._orientation, degrees=True)
        rotation_matrix = rotation.as_matrix()

        # Create 4x4 transform matrix
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = self._position

        return transform

    def initialize(self, vehicle):
        """Initialize the sensor when attached to a vehicle."""

        super().initialize(vehicle)

        # Get the complete prim path for the sensor
        self._stage_prim_path = f"{vehicle.prim_path}/{self._sensor_name}"

        # Get PhysX interface for raycasting
        self._physx_interface = get_physx_interface()

        print(f"LaserRangefinder '{self._sensor_name}' initialized at {self._stage_prim_path}")

    def start(self):
        """Start the rangefinder sensor."""
        print(f"LaserRangefinder '{self._sensor_name}' started")

    def stop(self):
        """Stop the rangefinder sensor."""
        print(f"LaserRangefinder '{self._sensor_name}' stopped")

    @GraphicalSensor.update_at_rate
    def update(self, state: State, dt: float):
        """
        Update the laser rangefinder measurement using raycasting.

        Args:
            state (State): Current vehicle state
            dt (float): Time elapsed since last update

        Returns:
            dict: Sensor measurement data
        """

        # Get vehicle world transform
        vehicle_position = state.position_inertial_frame
        vehicle_quaternion = state.attitude_inertial_frame  # [qw, qx, qy, qz]

        # Convert vehicle quaternion to rotation matrix
        vehicle_rot = Rotation.from_quat([
            vehicle_quaternion[1], vehicle_quaternion[2],
            vehicle_quaternion[3], vehicle_quaternion[0]
        ])
        vehicle_rotation_matrix = vehicle_rot.as_matrix()

        # Compute sensor world transform
        sensor_world_transform = self._compute_sensor_world_transform(
            vehicle_position, vehicle_rotation_matrix
        )

        # Perform raycast
        distance, hit_valid = self._perform_raycast(sensor_world_transform)

        # Add measurement noise
        if hit_valid and distance > 0:
            noise = np.random.normal(0, self._accuracy)
            distance = max(0, distance + noise)

        # Update sensor state
        self._state = {
            'distance': distance,
            'timestamp': time.time(),
            'valid': hit_valid,
            'sensor_name': self._sensor_name,
            'max_range': self._max_range,
            'min_range': self._min_range
        }

        return self._state

    def _compute_sensor_world_transform(self, vehicle_position, vehicle_rotation_matrix):
        """
        Compute the sensor's world transform from vehicle state and local transform.
        """

        # Create vehicle world transform matrix
        vehicle_transform = np.eye(4)
        vehicle_transform[:3, :3] = vehicle_rotation_matrix
        vehicle_transform[:3, 3] = vehicle_position

        # Compute sensor world transform
        sensor_world_transform = vehicle_transform @ self._local_transform

        return sensor_world_transform

    def _perform_raycast(self, sensor_transform):
        """
        Perform physics raycast to measure distance.

        Args:
            sensor_transform (np.ndarray): 4x4 sensor world transform matrix

        Returns:
            tuple: (distance, hit_valid)
        """

        # Extract sensor position and forward direction
        sensor_position = sensor_transform[:3, 3]
        sensor_forward = sensor_transform[:3, 2]  # Z-axis is forward

        # Ray start and end points
        ray_start = sensor_position
        ray_end = sensor_position + sensor_forward * self._max_range

        # Convert to Isaac Sim format
        ray_start_carb = Gf.Vec3f(ray_start[0], ray_start[1], ray_start[2])
        ray_end_carb = Gf.Vec3f(ray_end[0], ray_end[1], ray_end[2])

        try:
            # Perform raycast using PhysX
            if self._physx_interface:
                hit_info = self._physx_interface.raycast_closest(
                    ray_start_carb, ray_end_carb, 0xFFFFFFFF  # Hit all collision layers
                )

                if hit_info.get("hit", False):
                    hit_position = hit_info.get("position", ray_end_carb)
                    hit_distance = np.linalg.norm(
                        np.array([hit_position[0], hit_position[1], hit_position[2]]) - sensor_position
                    )

                    # Check if hit is within valid range
                    if self._min_range <= hit_distance <= self._max_range:
                        return hit_distance, True

            # No valid hit found
            return self._max_range, False

        except Exception as e:
            print(f"LaserRangefinder raycast error: {e}")
            return self._max_range, False

    def get_measurement(self):
        """
        Get the latest distance measurement.

        Returns:
            dict: Latest measurement data
        """
        return self._state

    def get_distance(self):
        """
        Get the latest distance measurement value.

        Returns:
            float: Distance in meters, or max_range if no valid measurement
        """
        return self._state.get('distance', self._max_range)

    def is_valid(self):
        """
        Check if the latest measurement is valid.

        Returns:
            bool: True if measurement is valid, False otherwise
        """
        return self._state.get('valid', False)

    @property
    def state(self):
        """
        Get current sensor state.

        Returns:
            dict: Current sensor state with distance, timestamp, and validity
        """
        return self._state

    @property
    def max_range(self):
        """Maximum measurement range in meters."""
        return self._max_range

    @property
    def min_range(self):
        """Minimum measurement range in meters."""
        return self._min_range

    @property
    def frequency(self):
        """Measurement frequency in Hz."""
        return self._frequency

    @property
    def beam_width(self):
        """Beam divergence angle in radians."""
        return self._beam_width

    def config_from_dict(self, config_dict):
        """
        Configure sensor from dictionary (required by base class).

        Args:
            config_dict (dict): Configuration parameters
        """
        self._max_range = config_dict.get("max_range", self._max_range)
        self._min_range = config_dict.get("min_range", self._min_range)
        self._frequency = config_dict.get("frequency", self._frequency)
        self._beam_width = config_dict.get("beam_width", self._beam_width)
        self._accuracy = config_dict.get("accuracy", self._accuracy)