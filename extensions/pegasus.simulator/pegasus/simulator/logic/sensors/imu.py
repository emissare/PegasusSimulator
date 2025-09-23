"""
| File: imu.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Simulates an imu. Based on the implementation provided in PX4 stil_gazebo (https://github.com/PX4/PX4-SITL_gazebo)
"""

__all__ = ["IMU"]

import numpy as np
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.vehicle_state import VehicleState
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.sensor_models import IMUState
from pegasus.simulator.logic.sensors.geo_mag_utils import GRAVITY_VECTOR


class IMU(Sensor):
    """The class that implements the IMU sensor. This class inherits the base class Sensor."""

    def __init__(self, config={}):
        """Initialize the IMU class

        Args:
            config (dict): A Dictionary that contains all the parameters for configuring the IMU - it can be empty or only have some of the parameters used by the IMU.

        Examples:
            The dictionary default parameters are

            >>> {"gyroscope": {
            >>>        "noise_density": 2.0 * 35.0 / 3600.0 / 180.0 * pi,
            >>>        "random_walk": 2.0 * 4.0 / 3600.0 / 180.0 * pi,
            >>>        "bias_correlation_time": 1.0e3,
            >>>        "turn_on_bias_sigma": 0.5 / 180.0 * pi},
            >>>  "accelerometer": {
            >>>         "noise_density": 2.0 * 2.0e-3,
            >>>         "random_walk": 2.0 * 3.0e-3,
            >>>         "bias_correlation_time": 300.0,
            >>>         "turn_on_bias_sigma": 20.0e-3 * 9.8
            >>> },
            >>>  "update_rate": 1.0}                 # Hz
        """

        # Initialize the Super class "object" attributes
        super().__init__(
            sensor_type="IMU", update_rate=config.get("update_rate", 250.0)
        )

        # Orientation noise constant
        self._orientation_noise: float = 0.0

        # Gyroscope noise constants
        self._gyroscope_bias: np.ndarray = np.zeros((3,))
        gyroscope_config = config.get("gyroscope", {})
        self._gyroscope_noise_density = gyroscope_config.get(
            "noise_density", 0.0003393695767766752
        )
        self._gyroscope_random_walk = gyroscope_config.get(
            "random_walk", 3.878509448876288e-05
        )
        self._gyroscope_bias_correlation_time = gyroscope_config.get(
            "bias_correlation_time", 1.0e3
        )
        self._gyroscope_turn_on_bias_sigma = gyroscope_config.get(
            "turn_on_bias_sigma", 0.008726646259971648
        )

        # Accelerometer noise constants
        self._accelerometer_bias: np.ndarray = np.zeros((3,))
        accelerometer_config = config.get("accelerometer", {})
        self._accelerometer_noise_density = accelerometer_config.get(
            "noise_density", 0.004
        )
        self._accelerometer_random_walk = accelerometer_config.get("random_walk", 0.006)
        self._accelerometer_bias_correlation_time = accelerometer_config.get(
            "bias_correlation_time", 300.0
        )
        self._accelerometer_turn_on_bias_sigma = accelerometer_config.get(
            "turn_on_bias_sigma", 0.196
        )

        # Initialize the IMU state with default values
        self._state = IMUState(
            orientation_quat_frd_ned=[
                0.0,
                0.0,
                0.0,
                1.0,
            ],  # Identity quaternion [qx, qy, qz, qw]
            angular_velocity_frd_body_rps=[0.0, 0.0, 0.0],
            linear_acceleration_frd_body_mpss=[
                0.0,
                0.0,
                9.80665,
            ],  # Gravity compensation
        )

    @property
    def state(self) -> IMUState:
        """
        Returns:
            IMUState: The current IMU sensor state with all measurements properly typed
        """
        return self._state

    def update(self, state: VehicleState, current_time_s: float):
        """Method that implements the logic of an IMU. In this method we start by generating the random walk of the
        gyroscope. This value is then added to the real angular velocity of the vehicle (FLU relative to ENU inertial frame
        expressed in FLU body frame). The same logic is followed for the accelerometer and the accelerations. After this step,
        the angular velocity is rotated such that it expressed a FRD body frame, relative to a NED inertial frame, expressed
        in the FRD body frame. Additionally, the acceleration is also rotated, such that it becomes expressed in the body
        FRD frame of the vehicle. This sensor outputs data that follows the PX4 adopted standard.

        Args:
            state (VehicleState): The current state of the vehicle.
            current_time_s (float): Current simulation time in seconds.

        Returns:
            (IMUState) The current state of the sensor, or None if not time to update
        """

        # Check if it's time to update
        if not self.should_update(current_time_s):
            return None

        # Calculate dt for this update
        dt = current_time_s - self._prev_update_time_s
        self._prev_update_time_s = current_time_s

        # Gyroscopic terms
        tau_g: float = self._accelerometer_bias_correlation_time

        # Discrete-time standard deviation equivalent to an "integrating" sampler with integration time dt
        sigma_g_d: float = 1 / np.sqrt(dt) * self._gyroscope_noise_density
        sigma_b_g: float = self._gyroscope_random_walk

        # Compute exact covariance of the process after dt [Maybeck 4-114]
        sigma_b_g_d: float = np.sqrt(
            -sigma_b_g * sigma_b_g * tau_g / 2.0 * (np.exp(-2.0 * dt / tau_g) - 1.0)
        )

        # Compute state-transition
        phi_g_d: float = np.exp(-1.0 / tau_g * dt)

        # Get angular velocity in FRD body frame (already converted)
        angular_velocity_frd: np.ndarray = state.angular_velocity_frd_rps.copy()

        # Add noise to angular velocity
        for i in range(3):
            self._gyroscope_bias[i] = (
                phi_g_d * self._gyroscope_bias[i] + sigma_b_g_d * np.random.randn()
            )
            angular_velocity_frd[i] += (
                sigma_g_d * np.random.randn() + self._gyroscope_bias[i]
            )

        # Accelerometer terms
        tau_a: float = self._accelerometer_bias_correlation_time

        # Discrete-time standard deviation equivalent to an "integrating" sampler with integration time dt
        sigma_a_d: float = 1.0 / np.sqrt(dt) * self._accelerometer_noise_density
        sigma_b_a: float = self._accelerometer_random_walk

        # Compute exact covariance of the process after dt [Maybeck 4-114].
        sigma_b_a_d: float = np.sqrt(
            -sigma_b_a * sigma_b_a * tau_a / 2.0 * (np.exp(-2.0 * dt / tau_a) - 1.0)
        )

        # Compute state-transition.
        phi_a_d: float = np.exp(-1.0 / tau_a * dt)

        # Get acceleration in NED frame (already calculated by VehicleState)
        acceleration_ned = state.acceleration_ned_mpss - GRAVITY_VECTOR

        # Convert acceleration from NED inertial to FRD body frame
        attitude_frd_ned = Rotation.from_quat(state.attitude_frd_ned_quat)
        linear_acceleration_frd = attitude_frd_ned.inv().apply(acceleration_ned)

        # Add noise to linear acceleration
        for i in range(3):
            self._accelerometer_bias[i] = (
                phi_a_d * self._accelerometer_bias[i] + sigma_b_a_d * np.random.rand()
            )
            linear_acceleration_frd[i] += (
                sigma_a_d * np.random.randn()
            )  # + self._accelerometer_bias[i]

        # Get attitude directly from state (already in FRD/NED)
        attitude_frd_ned_quat = state.attitude_frd_ned_quat

        self._state = IMUState(
            orientation_quat_frd_ned=attitude_frd_ned_quat.tolist(),
            angular_velocity_frd_body_rps=angular_velocity_frd.tolist(),
            linear_acceleration_frd_body_mpss=linear_acceleration_frd.tolist(),
        )

        self._has_new_data = True
        return self._state  # Return the IMUState object directly
