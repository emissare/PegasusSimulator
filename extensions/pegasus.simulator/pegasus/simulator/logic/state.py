"""
| File: state.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Describes the state of a vehicle (or rigidbody).
"""
__all__ = ["State"]

import numpy as np
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.rotations import (
    rot_FLU_inertial_to_NED_inertial,
    rot_FLU_body_to_FRD_body,
    rot_ENU_to_NED,  # Keep for now, will remove after fixing methods
    rot_FLU_to_FRD   # Keep for now, will remove after fixing methods
)


class State:
    """
    Stores the state of a given vehicle.

    COORDINATE SYSTEMS:
    - FLU: Isaac Sim world/body frame (Front-Left-Up)
    - NED: Geographic frame for PX4 (North-East-Down)
    - FRD: Body frame for PX4 (Front-Right-Down)

    All variables are suffixed with their coordinate system for clarity.
    Legacy code incorrectly assumed Isaac uses ENU, but it actually uses FLU.
    """

    def __init__(self):
        """
        Initialize the State object with clearly labeled coordinate systems
        """

        # Position [x,y,z] in Isaac's FLU world frame
        # X=Forward, Y=Left, Z=Up (no inherent geographic meaning)
        self.position_flu = np.array([0.0, 0.0, 0.0])

        # Quaternion [qx,qy,qz,qw] for FLU body relative to FLU world
        # Identity [0,0,0,1] means vehicle points along +X (Forward)
        self.attitude_flu = np.array([0.0, 0.0, 0.0, 1.0])

        # Linear velocity [u,v,w] in FLU body frame
        self.linear_body_velocity_flu = np.array([0.0, 0.0, 0.0])

        # Linear velocity [vx,vy,vz] in FLU world frame
        self.linear_velocity_flu = np.array([0.0, 0.0, 0.0])

        # Angular velocity [p,q,r] in FLU body frame
        self.angular_velocity_flu_body = np.array([0.0, 0.0, 0.0])

        # Linear acceleration [ax,ay,az] in FLU world frame
        self.linear_acceleration_flu = np.array([0.0, 0.0, 0.0])

        # Keep legacy names as properties for backward compatibility
        # (will remove once all code is updated)

    @property
    def position(self):
        return self.position_flu

    @position.setter
    def position(self, value):
        self.position_flu = value

    @property
    def attitude(self):
        return self.attitude_flu

    @attitude.setter
    def attitude(self, value):
        self.attitude_flu = value

    @property
    def linear_velocity(self):
        return self.linear_velocity_flu

    @linear_velocity.setter
    def linear_velocity(self, value):
        self.linear_velocity_flu = value

    @property
    def linear_body_velocity(self):
        return self.linear_body_velocity_flu

    @linear_body_velocity.setter
    def linear_body_velocity(self, value):
        self.linear_body_velocity_flu = value

    @property
    def angular_velocity(self):
        return self.angular_velocity_flu_body

    @angular_velocity.setter
    def angular_velocity(self, value):
        self.angular_velocity_flu_body = value

    @property
    def linear_acceleration(self):
        return self.linear_acceleration_flu

    @linear_acceleration.setter
    def linear_acceleration(self, value):
        self.linear_acceleration_flu = value

    def get_position_ned(self):
        """
        Convert position from Isaac FLU inertial to NED inertial frame.

        Isaac FLU: X=Front (North), Y=Left (West), Z=Up
        PX4 NED: X=North, Y=East, Z=Down

        Returns:
            np.ndarray: Position [x,y,z] in NED inertial frame
        """
        # Transform from FLU inertial to NED inertial
        position_ned = rot_FLU_inertial_to_NED_inertial.apply(self.position_flu)
        return position_ned

    def get_attitude_ned_frd(self):
        """
        Convert attitude from Isaac FLU to PX4's NED-FRD convention.

        Input: Quaternion for FLU body relative to FLU inertial (from Isaac)
        Output: Quaternion for FRD body relative to NED inertial (for PX4)

        The transformation is:
        q_FRD_NED = q_FLU_to_NED * q_FLU_FLU * q_FLU_to_FRD

        Returns:
            np.ndarray: Quaternion [qx,qy,qz,qw] for FRD body in NED inertial
        """
        # DEBUG: Add logging to understand the transformation
        import carb
        carb.log_warn(f"[DEBUG] get_attitude_ned_frd input (FLU body in FLU world): {self.attitude_flu}")

        # Transform from FLU body in FLU world to FRD body in NED world
        # Step 1: FLU body in FLU world (input quaternion)
        attitude_flu_flu = Rotation.from_quat(self.attitude_flu)

        # Step 2: Apply transformations
        # FLU world → NED world, FLU body → FRD body
        attitude_frd_ned = rot_FLU_inertial_to_NED_inertial * attitude_flu_flu * rot_FLU_body_to_FRD_body
        result = attitude_frd_ned.as_quat()

        carb.log_warn(f"[DEBUG] After FLU→NED * attitude * FLU→FRD: {result}")

        # Calculate Euler angles for debugging
        euler = attitude_frd_ned.as_euler('ZYX', degrees=True)
        carb.log_warn(f"[DEBUG] Euler (ZYX): yaw={euler[0]:.1f}°, pitch={euler[1]:.1f}°, roll={euler[2]:.1f}°")

        return result

    def get_linear_body_velocity_ned_frd(self):
        """
        Convert linear body velocity from Isaac FLU body to PX4 FRD body frame.

        First transforms world velocity to body frame, then converts to FRD.

        Returns:
            np.ndarray: [u,v,w] velocity in FRD body frame
        """
        # Transform world velocity to FLU body frame
        # Use inverse of attitude quaternion to go from world to body
        linear_vel_body_flu = Rotation.from_quat(self.attitude_flu).inv().apply(self.linear_velocity_flu)

        # Convert from FLU body frame to FRD body frame
        return rot_FLU_body_to_FRD_body.apply(linear_vel_body_flu)

    def get_linear_velocity_ned(self):
        """
        Convert linear velocity from Isaac FLU inertial to NED inertial frame.

        Returns:
            np.ndarray: Velocity [vx,vy,vz] in NED inertial frame
        """
        # Transform from FLU inertial to NED inertial
        velocity_ned = rot_FLU_inertial_to_NED_inertial.apply(self.linear_velocity_flu)
        return velocity_ned

    def get_angular_velocity_frd(self):
        """
        Convert angular velocity from Isaac FLU body to PX4 FRD body frame.

        Input: Angular velocity [p,q,r] in FLU body frame (from Isaac)
        Output: Angular velocity [p,q,r] in FRD body frame (for PX4)

        Returns:
            np.ndarray: [p,q,r] angular velocity in FRD body frame
        """
        # Convert from FLU body frame to FRD body frame
        return rot_FLU_body_to_FRD_body.apply(self.angular_velocity_flu_body)

    def get_linear_acceleration_ned(self):
        """
        Convert linear acceleration from Isaac FLU inertial to NED inertial frame.

        Returns:
            np.ndarray: [ax,ay,az] acceleration in NED inertial frame
        """
        # Transform from FLU inertial to NED inertial
        return rot_FLU_inertial_to_NED_inertial.apply(self.linear_acceleration_flu)
