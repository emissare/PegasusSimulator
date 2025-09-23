"""
Force Generators for Vehicle Physics

This module provides a generalized system for generating forces and torques
in vehicle simulations. It supports two main types of force generators:
1. SpinningBody - Motor-driven rotors that create thrust and reaction torque
2. LiftingSurface - Aerodynamic surfaces that create lift forces only

The system is based on correct physics principles:
- Single rigid body per vehicle (with mass and inertia)
- Forces applied at positions (creates natural moments)
- Newton's third law for motor reaction torques
"""

import numpy as np
import math
from abc import ABC, abstractmethod

# USD imports for visual rotation (avoid repeated imports)
try:
    from omni.usd import get_context
    from pxr import UsdGeom

    USD_AVAILABLE = True
except ImportError:
    USD_AVAILABLE = False


class ForceGenerator(ABC):
    """
    Base class for any component that generates forces and/or torques.

    All force generators have a position relative to the vehicle's center of mass
    and can convert control inputs into force and torque vectors.
    """

    def __init__(self, position: np.ndarray):
        """
        Initialize force generator.

        Args:
            position (np.ndarray): Position relative to body center of mass [x, y, z]
        """
        self.position = np.array(position, dtype=float)

    @abstractmethod
    def get_force_and_torque(self, input_value: float) -> tuple:
        """
        Calculate force and torque vectors for given control input.

        Args:
            input_value (float): Control input (RPM, deflection angle, etc.)

        Returns:
            tuple: (force_vector, torque_vector) where each is np.ndarray([x, y, z])
                   Forces and torques are in the body frame coordinate system.
        """
        pass

    def __repr__(self):
        return f"{self.__class__.__name__}(position={self.position})"


class SpinningBody(ForceGenerator):
    """
    Motor-driven rotor that generates both thrust forces and reaction torques.

    Physics Model:
    - Thrust: T = k_thrust * ω² (aerodynamic)
    - Reaction Torque: τ = k_torque * ω² (Newton's 3rd law)

    The reaction torque occurs because when a motor applies torque to spin a rotor,
    an equal and opposite torque acts on the motor stator (attached to the body).
    This is fundamental physics that occurs even in a vacuum.
    """

    def __init__(
        self,
        position: np.ndarray,
        thrust_coefficient: float,
        torque_coefficient: float,
        spin_direction: int,
    ):
        """
        Initialize spinning body (rotor/propeller).

        Args:
            position (np.ndarray): Position relative to body center [x, y, z]
            thrust_coefficient (float): Thrust per (angular velocity)²
            torque_coefficient (float): Motor torque per (angular velocity)²
            spin_direction (int): +1 for clockwise, -1 for counter-clockwise
        """
        super().__init__(position)
        self.thrust_coefficient = thrust_coefficient
        self.torque_coefficient = torque_coefficient
        self.spin_direction = int(spin_direction)  # Ensure it's +1 or -1

        if self.spin_direction not in [-1, 1]:
            raise ValueError("spin_direction must be +1 (CW) or -1 (CCW)")

        # Visual rotation state (for transform-based animation)
        self.visual_path = None
        self.accumulated_rotation = 0.0  # Radians

    def get_force_and_torque(self, angular_velocity: float) -> tuple:
        """
        Calculate thrust force and motor reaction torque.

        Args:
            angular_velocity (float): Rotor angular velocity (rad/s)

        Returns:
            tuple: (force_vector, torque_vector)
                - force_vector: [0, 0, thrust] (thrust along Z-axis)
                - torque_vector: [0, 0, reaction_torque] (yaw torque)
        """
        # Aerodynamic thrust force (always upward in rotor frame)
        thrust = self.thrust_coefficient * angular_velocity**2
        force = np.array([0.0, 0.0, thrust])

        # Motor reaction torque (Newton's 3rd law)
        # When motor spins rotor in one direction, body experiences opposite torque
        motor_torque = self.torque_coefficient * angular_velocity**2
        reaction_torque = motor_torque * self.spin_direction

        torque = np.array([0.0, 0.0, reaction_torque])

        return force, torque

    def set_visual_path(self, visual_path: str):
        """
        Set the path to the visual mesh for transform-based rotation.

        Args:
            visual_path (str): USD path to the visual mesh
        """
        self.visual_path = visual_path

    def update_visual_rotation(self, angular_velocity: float, dt: float = 1.0 / 60.0):
        """
        Update visual rotation using transform operations.

        Args:
            angular_velocity (float): Rotor angular velocity (rad/s)
            dt (float): Delta time for accumulating rotation
        """
        if not self.visual_path:
            return

        # Three-state visual logic (matches old implementation)
        visual_velocity = 0.0
        if 0.0 < angular_velocity < 100.0:
            # Slow spin when armed but low thrust
            visual_velocity = 5.0 * self.spin_direction
        elif angular_velocity >= 100.0:
            # Fast spin when applying significant thrust
            visual_velocity = 100.0 * self.spin_direction

        # Accumulate rotation
        self.accumulated_rotation += visual_velocity * dt

        # Apply rotation to visual container (not the mesh directly)
        if not USD_AVAILABLE:
            return

        try:
            stage = get_context().get_stage()
            visual_prim = stage.GetPrimAtPath(self.visual_path)

            if visual_prim:
                # Create or get the rotation operation
                rotation_attr = visual_prim.GetAttribute("xformOp:rotateZ")
                if not rotation_attr:
                    # Create the rotation operation if it doesn't exist
                    xformable = UsdGeom.Xformable(visual_prim)
                    rotation_attr = xformable.AddRotateZOp()

                # Set rotation in degrees
                rotation_degrees = math.degrees(self.accumulated_rotation)
                rotation_attr.Set(rotation_degrees)

        except Exception:
            # Fail silently - visual rotation is not critical
            pass

    def __repr__(self):
        direction_str = "CW" if self.spin_direction == 1 else "CCW"
        return (
            f"SpinningBody(pos={self.position}, thrust_k={self.thrust_coefficient}, "
            f"torque_k={self.torque_coefficient}, dir={direction_str})"
        )


class LiftingSurface(ForceGenerator):
    """
    Aerodynamic surface that generates lift forces only (no torques).

    Physics Model:
    - Lift: L = k_lift * α (where α is deflection angle)
    - Torque: 0 (no direct torque generation)

    Examples: Wings, ailerons, elevators, rudders
    Note: While control surfaces create moments about the vehicle's center of mass,
    this happens naturally when the lift force is applied at the surface's position.
    The ForceGenerator doesn't need to calculate these moments directly.
    """

    def __init__(
        self,
        position: np.ndarray,
        lift_coefficient: float,
        lift_direction: np.ndarray = np.array([0, 0, 1]),
    ):
        """
        Initialize lifting surface.

        Args:
            position (np.ndarray): Position relative to body center [x, y, z]
            lift_coefficient (float): Lift force per unit deflection angle
            lift_direction (np.ndarray): Unit vector for lift direction (default: +Z)
        """
        super().__init__(position)
        self.lift_coefficient = lift_coefficient
        self.lift_direction = np.array(lift_direction, dtype=float)

        # Normalize the lift direction vector
        norm = np.linalg.norm(self.lift_direction)
        if norm > 0:
            self.lift_direction = self.lift_direction / norm
        else:
            raise ValueError("lift_direction cannot be zero vector")

    def get_force_and_torque(self, deflection_angle: float) -> tuple:
        """
        Calculate lift force from deflection angle.

        Args:
            deflection_angle (float): Control surface deflection (radians or degrees)

        Returns:
            tuple: (force_vector, torque_vector)
                - force_vector: lift force in specified direction
                - torque_vector: [0, 0, 0] (no direct torque)
        """
        # Calculate lift magnitude
        lift_magnitude = self.lift_coefficient * deflection_angle

        # Apply lift in specified direction
        force = lift_magnitude * self.lift_direction

        # Lifting surfaces don't generate direct torques
        # (moments are created naturally when force is applied at position offset)
        torque = np.array([0.0, 0.0, 0.0])

        return force, torque

    def __repr__(self):
        return (
            f"LiftingSurface(pos={self.position}, lift_k={self.lift_coefficient}, "
            f"direction={self.lift_direction})"
        )
