"""
| File: vehicle_state.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Describes the state of a vehicle (or rigidbody).

COORDINATE SYSTEMS:
- Isaac World: NWU (North-West-Up) - X points North, Y points West, Z points Up
- PX4 World: NED (North-East-Down) - X points North, Y points East, Z points Down
- Isaac Body: FLU (Front-Left-Up) - X points Front, Y points Left, Z points Up
- PX4 Body: FRD (Front-Right-Down) - X points Front, Y points Right, Z points Down
"""

__all__ = ["VehicleState"]

import carb
import numpy as np
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.rotations import (
    rot_NWU_to_NED,
    rot_FLU_to_FRD,
)


class VehicleState:
    """
    Single source of truth for vehicle state and coordinate conversions.

    Accepts raw data from Isaac Sim (NWU world frame, FLU body frame) and internally
    stores everything in PX4 convention (NED world frame, FRD body frame).

    INPUT FRAMES:
    - NWU: Isaac Sim world frame (North-West-Up)
    - FLU: Isaac Sim body frame (Front-Left-Up)

    STORED FRAMES:
    - NED: PX4 world frame (North-East-Down)
    - FRD: PX4 body frame (Front-Right-Down)
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

    def update_from_isaac(
        self,
        position_nwu_m: np.ndarray,
        attitude_flu_nwu_quat: np.ndarray,
        linear_velocity_nwu_mps: np.ndarray,
        angular_velocity_flu_rps: np.ndarray,
        dt: float,
    ):
        self._position_ned_m = rot_NWU_to_NED.apply(position_nwu_m)

        self._velocity_ned_mps = rot_NWU_to_NED.apply(linear_velocity_nwu_mps)

        if dt > 0:
            self._acceleration_ned_mpss = (
                self._velocity_ned_mps - self._prev_velocity_ned_mps
            ) / dt
        self._prev_velocity_ned_mps = self._velocity_ned_mps.copy()

        # Store the FLU_NWU quaternion for debugging
        self._attitude_flu_nwu_quat_internal = attitude_flu_nwu_quat.copy()

        attitude_flu_nwu = Rotation.from_quat(attitude_flu_nwu_quat)

        # Transform: (NED to NWU) * (NWU to FLU) * (FLU to FRD) = (NED to FRD)
        attitude_frd_ned = (
            rot_NWU_to_NED  # Convert world frame: NWU → NED
            * attitude_flu_nwu  # Original rotation from Isaac
            * rot_FLU_to_FRD  # Convert body frame: FLU → FRD
        )
        self._attitude_frd_ned_quat = attitude_frd_ned.as_quat()

        self._angular_velocity_frd_rps = rot_FLU_to_FRD.apply(angular_velocity_flu_rps)

        # Calculate body velocity in FRD frame
        # First get velocity in FLU body frame, then convert to FRD
        velocity_flu_body_mps = attitude_flu_nwu.inv().apply(linear_velocity_nwu_mps)
        self._body_velocity_frd_mps = rot_FLU_to_FRD.apply(velocity_flu_body_mps)

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
