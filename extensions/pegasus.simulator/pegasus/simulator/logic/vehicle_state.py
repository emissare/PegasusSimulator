"""
| File: state.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Describes the state of a vehicle (or rigidbody).
"""

__all__ = ["VehicleState"]

import carb
import numpy as np
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.rotations import (
    rot_FLU_inertial_to_NED_inertial,
    rot_FLU_body_to_FRD_body,
)


class VehicleState:
    """
    Single source of truth for vehicle state and coordinate conversions.

    Accepts raw data from Isaac Sim (FLU coordinate system) and internally
    stores everything in PX4 convention (NED inertial, FRD body).

    COORDINATE SYSTEMS:
    - FLU: Isaac Sim world/body frame (Front-Left-Up) - INPUT
    - NED: Geographic frame for PX4 (North-East-Down) - STORED
    - FRD: Body frame for PX4 (Front-Right-Down) - STORED
    """

    def __init__(self):
        """
        Initialize the VehicleState with zero values in NED/FRD coordinates.
        """
        # Private storage - everything in NED inertial and FRD body frames
        self._position_ned_m = np.array([0.0, 0.0, 0.0])
        self._velocity_ned_mps = np.array([0.0, 0.0, 0.0])
        self._acceleration_ned_mpss = np.array([0.0, 0.0, 0.0])
        self._attitude_frd_ned_quat = np.array([0.0, 0.0, 0.0, 1.0])  # Identity
        self._angular_velocity_frd_rps = np.array([0.0, 0.0, 0.0])
        self._body_velocity_frd_mps = np.array([0.0, 0.0, 0.0])

        # Store previous velocity for acceleration calculation
        self._prev_velocity_ned_mps = np.array([0.0, 0.0, 0.0])

        # Logging control
        self._last_log_time = 0.0
        self._total_time = 0.0
        self._log_interval = 1.0  # Log every 1 second

    def update_from_isaac(
        self,
        position_flu_m: np.ndarray,
        attitude_flu_quat: np.ndarray,
        linear_velocity_flu_mps: np.ndarray,
        angular_velocity_flu_rps: np.ndarray,
        dt: float,
    ):
        self._position_ned_m = rot_FLU_inertial_to_NED_inertial.apply(position_flu_m)

        self._velocity_ned_mps = rot_FLU_inertial_to_NED_inertial.apply(
            linear_velocity_flu_mps
        )

        if dt > 0:
            self._acceleration_ned_mpss = (
                self._velocity_ned_mps - self._prev_velocity_ned_mps
            ) / dt
        self._prev_velocity_ned_mps = self._velocity_ned_mps.copy()

        # Convert attitude from FLU/FLU to FRD/NED
        attitude_flu_flu = Rotation.from_quat(attitude_flu_quat)
        attitude_frd_ned = (
            rot_FLU_inertial_to_NED_inertial
            * attitude_flu_flu
            * rot_FLU_body_to_FRD_body
        )
        self._attitude_frd_ned_quat = attitude_frd_ned.as_quat()

        # Convert angular velocity from FLU body to FRD body
        self._angular_velocity_frd_rps = rot_FLU_body_to_FRD_body.apply(
            angular_velocity_flu_rps
        )

        # Calculate body velocity in FRD frame
        # First get velocity in FLU body frame, then convert to FRD
        velocity_flu_body_mps = attitude_flu_flu.inv().apply(linear_velocity_flu_mps)
        self._body_velocity_frd_mps = rot_FLU_body_to_FRD_body.apply(
            velocity_flu_body_mps
        )

        # Update total time and log periodically
        self._total_time += dt
        if self._total_time - self._last_log_time >= self._log_interval:
            carb.log_info(
                f"[VehicleState] t={self._total_time:.2f}s | "
                f"pos_ned=[{self._position_ned_m[0]:.2f}, {self._position_ned_m[1]:.2f}, {self._position_ned_m[2]:.2f}] m | "
                f"vel_ned=[{self._velocity_ned_mps[0]:.2f}, {self._velocity_ned_mps[1]:.2f}, {self._velocity_ned_mps[2]:.2f}] m/s"
            )
            self._last_log_time = self._total_time

    @property
    def position_ned_m(self) -> np.ndarray:
        return self._position_ned_m

    @property
    def velocity_ned_mps(self) -> np.ndarray:
        return self._velocity_ned_mps

    @property
    def acceleration_ned_mpss(self) -> np.ndarray:
        return self._acceleration_ned_mpss

    @property
    def attitude_frd_ned_quat(self) -> np.ndarray:
        """Quaternion [qx,qy,qz,qw] for FRD body in NED inertial"""
        return self._attitude_frd_ned_quat

    @property
    def angular_velocity_frd_rps(self) -> np.ndarray:
        """Angular velocity in FRD body frame [p,q,r] in radians per second"""
        return self._angular_velocity_frd_rps

    @property
    def body_velocity_frd_mps(self) -> np.ndarray:
        """Linear velocity in FRD body frame [u,v,w] in meters per second"""
        return self._body_velocity_frd_mps
