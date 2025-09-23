"""
| File: linear_drag.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: Computes the forces that should actuate on a rigidbody affected by linear drag
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""

import numpy as np
from pegasus.simulator.logic.dynamics.drag import Drag
from pegasus.simulator.logic.vehicle_state import VehicleState


class LinearDrag(Drag):
    """
    Class that implements linear drag computations afftecting a rigid body. It inherits the Drag base class.
    """

    def __init__(self, drag_coefficients=[0.0, 0.0, 0.0]):
        """
        Receives as input the drag coefficients of the vehicle as a 3x1 vector of constants

        Args:
            drag_coefficients (list[float]): The constant linear drag coefficients to used to compute the total drag forces
            affecting the rigid body. The linear drag is given by diag(dx, dy, dz) * [v_x, v_y, v_z] where the velocities
            are expressed in the body frame of the rigid body (using the FRU frame convention).
        """

        # Initialize the base Drag class
        super().__init__()

        # The linear drag coefficients of the vehicle's body frame
        self._drag_coefficients = np.diag(drag_coefficients)

        # The drag force to apply on the vehicle's body frame
        self._drag_force = np.array([0.0, 0.0, 0.0])

    @property
    def drag(self):
        """The drag force to be applied on the body frame of the vehicle

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
            frame, expressed in Newton (N) [dx, dy, dz]
        """
        return self._drag_force

    def update(self, state: VehicleState, dt: float):
        """Method that updates the drag force to be applied on the body frame of the vehicle. The total drag force
        is computed as: F_drag = -diag(dx,dy,dz) * velocity_body_frame

        Args:
            state (VehicleState): The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            np.ndarray: A 3-element array containing the drag force in FRD body frame [N]
        """

        # Get body velocity in FRD frame (standard frame)
        body_vel_frd = state.body_velocity_frd_mps

        # Compute drag force in FRD body frame
        # Note: drag coefficients should be defined in FRD frame
        self._drag_force_frd = -np.dot(self._drag_coefficients, body_vel_frd)
        return self._drag_force_frd
