"""
| File: multirotor.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Modified by: EmissarePegasusSimulator Team
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Programmatic multirotor vehicle creation from YAML configuration
"""

import yaml
import numpy as np
from pathlib import Path

# Isaac Sim imports
import carb
from isaacsim.core.api.objects import DynamicCuboid, DynamicCylinder
from isaacsim.core.prims import RigidPrim
from pxr import UsdGeom, UsdPhysics, Gf

# The vehicle interface
from pegasus.simulator.logic.vehicles.vehicle import Vehicle

# Mavlink interface
from pegasus.simulator.logic.px4_backend import (
    PX4Backend,
    PX4BackendConfig,
)

# Sensors and dynamics setup
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS


# For gimbal support
from pegasus.simulator.logic.graphical_sensors.gimbal_system import GimbalSystem


# VehicleComponents class removed - now using standardized Vehicle base class interface


class Multirotor(Vehicle):
    """
    Creates multirotor vehicles entirely from YAML configuration without USD files.
    Supports symmetric quadrotor_x configuration with configurable parameters.
    """

    def __init__(
        self,
        # Simulation specific configurations
        stage_prefix: str = "quadrotor",
        config_file: str = "",  # YAML config file path (required)
        vehicle_id: int = 0,
        # Spawning pose of the vehicle
        init_pos=[0.0, 0.0, 0.07],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
    ):
        """Initializes the multirotor object from YAML configuration

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "quadrotor".
            config_file (str): Path to vehicle YAML configuration file (required).
            vehicle_id (int): The id to be used for the vehicle. Defaults to 0.
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.07].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
        """

        # Store stage prefix immediately (needed for __del__ if initialization fails) - BUILD v2
        self._stage_prefix = stage_prefix
        carb.log_info(f"Multirotor BUILD v2 - Initializing with config: {config_file}")

        # Validate config file parameter
        if not config_file:
            raise ValueError(
                "config_file parameter is required - must specify path to YAML vehicle configuration"
            )

        # Load vehicle configuration from YAML
        config_path = Path(config_file)
        if not config_path.exists():
            raise FileNotFoundError(f"Vehicle config file not found: {config_file}")

        with open(config_path, "r") as f:
            self.vehicle_config = yaml.safe_load(f)

        # Load motor database using path utilities
        from pegasus.simulator.utils.paths import (
            get_motor_db_path,
            ensure_config_file_exists,
        )

        motor_db_path = ensure_config_file_exists(get_motor_db_path(), "motor database")
        with open(motor_db_path, "r") as f:
            motor_database = yaml.safe_load(f)

        # Get motor parameters
        motor_name = self.vehicle_config["vehicle"]["motor"]
        if motor_name not in motor_database["motors"]:
            raise ValueError(f"Motor '{motor_name}' not found in motor database")
        self.motor_params = motor_database["motors"][motor_name]

        # Extract vehicle parameters
        self.vehicle_type = self.vehicle_config["vehicle"]["type"]
        self.rotor_separation = self.vehicle_config["vehicle"]["rotor_separation"]
        self.vehicle_mass = self.vehicle_config["vehicle"]["mass"]

        # Setup sensors and backend first
        sensors, graphical_sensors, graphs, backend = self._setup_vehicle_components()

        # Initialize parent Vehicle class WITHOUT USD file (empty string)
        super().__init__(
            stage_prefix,
            "",
            init_pos,
            init_orientation,
            sensors,
            graphical_sensors,
            graphs,
            backend,
        )

        # Manual motor control attributes
        self._manual_control_enabled = False
        self._manual_motor_speeds = [0.0, 0.0, 0.0, 0.0]  # Angular velocities in rad/s

        # Create vehicle structure programmatically AFTER parent class initialization
        self._create_vehicle_structure(stage_prefix)

        # Setup force generators for new physics system
        self._setup_force_generators()

        # Register all components for standardized access
        self._register_components()

    def _create_vehicle_structure(self, stage_prefix: str):
        """
        Create the vehicle structure programmatically based on vehicle type.
        """
        if self.vehicle_type == "quadrotor_x":
            self._create_quadrotor_x_structure(stage_prefix)
        else:
            raise ValueError(f"Unsupported vehicle type: {self.vehicle_type}")

    def _create_quadrotor_x_structure(self, stage_prefix: str):
        """
        Create quadrotor X configuration structure with rotors at 45° angles.
        Matches the current Iris motor rotation pattern.
        """

        # PX4 X configuration rotor positions (above body) - Isaac Sim coords: +X forward, +Y left, +Z up
        separation = self.rotor_separation
        rotor_height = 0.1  # 10cm above body
        rotor_positions = [
            [
                separation / 2,
                -separation / 2,
                rotor_height,
            ],  # Motor 0: Front-right (+X, -Y)
            [
                -separation / 2,
                separation / 2,
                rotor_height,
            ],  # Motor 1: Rear-left (-X, +Y)
            [
                separation / 2,
                separation / 2,
                rotor_height,
            ],  # Motor 2: Front-left (+X, +Y)
            [
                -separation / 2,
                -separation / 2,
                rotor_height,
            ],  # Motor 3: Rear-right (-X, -Y)
        ]

        # Standard rotation directions (matching current Iris: [-1, -1, 1, 1])
        rotor_directions = [-1, -1, 1, 1]  # CCW, CCW, CW, CW

        # Build the multirotor structure
        self._build_multirotor_structure(
            stage_prefix, rotor_positions, rotor_directions
        )

    def _build_multirotor_structure(
        self, stage_prefix: str, rotor_positions: list, rotor_directions: list
    ):
        """
        Build the physical multirotor structure with body and rotors.
        """

        # Create main body
        self._create_body(stage_prefix)

        # Create rotors
        for i, (pos, direction) in enumerate(zip(rotor_positions, rotor_directions)):
            self._create_rotor(stage_prefix, i, pos, direction)

        # Create gimbal mount if enabled
        if self.vehicle_config["vehicle"].get("gimbal", {}).get("enabled", False):
            self._create_gimbal_mount(stage_prefix)

        # Setup articulation root after all structure is created
        self._setup_articulation(stage_prefix)

    def _create_body(self, stage_prefix: str):
        """
        Create the body using proper Xform hierarchy with ONE rigid body.
        Matches the old Pegasus structure: body Xform container with mesh child.
        Only the mesh has physics - this prevents nested rigid body errors.
        """

        body_config = self.vehicle_config["vehicle"]["body"]
        dimensions = body_config["dimensions"]
        color = body_config["color"]

        # Get USD stage for creating geometry
        from omni.usd import get_context

        stage = get_context().get_stage()

        # Create body Xform container (no physics)
        body_xform_path = f"{stage_prefix}/body"
        body_xform = UsdGeom.Xform.Define(stage, body_xform_path)

        # Create body mesh as child with physics (the ONE rigid body)
        body_mesh_path = f"{body_xform_path}/body_mesh"
        body_geom = UsdGeom.Cube.Define(stage, body_mesh_path)

        # Set geometry properties
        body_geom.CreateSizeAttr(max(dimensions))

        # Apply scaling to achieve desired dimensions
        scale_x = dimensions[0] / max(dimensions)
        scale_y = dimensions[1] / max(dimensions)
        scale_z = dimensions[2] / max(dimensions)
        body_geom.AddScaleOp().Set(Gf.Vec3d(scale_x, scale_y, scale_z))

        # Apply color (using USD material - simplified approach)
        if hasattr(body_geom, "CreateDisplayColorAttr"):
            normalized_color = [c / 255.0 for c in color]  # Convert to 0-1 range
            body_geom.CreateDisplayColorAttr(
                [(normalized_color[0], normalized_color[1], normalized_color[2])]
            )

        # Apply physics ONLY to the mesh (the ONE rigid body)
        body_prim = stage.GetPrimAtPath(body_mesh_path)
        UsdPhysics.RigidBodyAPI.Apply(body_prim)
        UsdPhysics.CollisionAPI.Apply(body_prim)

        # Set mass
        mass_api = UsdPhysics.MassAPI.Apply(body_prim)
        mass_api.CreateMassAttr(self.vehicle_mass)

        # Set inertia matrix if specified in configuration
        inertia_config = self.vehicle_config["vehicle"].get("inertia", {})
        if inertia_config:
            # Create diagonal inertia tensor from configuration
            diagonal_inertia = Gf.Vec3f(
                inertia_config.get("ixx", 0.029125),
                inertia_config.get("iyy", 0.029125),
                inertia_config.get("izz", 0.055225),
            )
            mass_api.CreateDiagonalInertiaAttr(diagonal_inertia)

            # Set center of mass if needed (default to origin)
            center_of_mass = Gf.Vec3f(0.0, 0.0, 0.0)
            mass_api.CreateCenterOfMassAttr(center_of_mass)

    def _create_rotor(
        self, stage_prefix: str, rotor_index: int, position: list, direction: int
    ):
        """
        Create a rotor with separate physics and visual components.
        Physics: Small rigid body for force application (connected via FixedJoint)
        Visual: Pure mesh for animation (no physics)
        """

        rotor_radius = self.motor_params["rotor_radius"]

        # Get USD stage for creating geometry
        from omni.usd import get_context

        stage = get_context().get_stage()

        # Create rotor Xform container as child of body (for automatic transforms)
        rotor_xform_path = f"{stage_prefix}/body/rotor{rotor_index}"
        rotor_xform = UsdGeom.Xform.Define(stage, rotor_xform_path)

        # Set rotor position relative to body
        rotor_xform.AddTranslateOp().Set(
            Gf.Vec3d(position[0], position[1], position[2])
        )

        # 1. Create PHYSICS component - small rigid body for force application
        rotor_physics_path = f"{rotor_xform_path}/rotor_physics"
        rotor_physics = UsdGeom.Sphere.Define(stage, rotor_physics_path)

        # Make physics component very small and invisible
        rotor_physics.CreateRadiusAttr(0.005)  # 5mm sphere
        rotor_physics.CreateDisplayColorAttr(
            [(1.0, 0.0, 0.0)]
        )  # Red for debugging (will be invisible)

        # Apply physics to the physics component
        physics_prim = stage.GetPrimAtPath(rotor_physics_path)
        UsdPhysics.RigidBodyAPI.Apply(physics_prim)
        UsdPhysics.CollisionAPI.Apply(physics_prim)

        # Very small mass (0.01kg) for force application point
        mass_api = UsdPhysics.MassAPI.Apply(physics_prim)
        mass_api.CreateMassAttr(0.01)

        # 2. Create ROTATION container - handles rotation without affecting scale
        rotor_rotation_path = f"{rotor_physics_path}/rotor_rotation"
        rotor_rotation = UsdGeom.Xform.Define(stage, rotor_rotation_path)

        # 3. Create VISUAL component - pure mesh as child of rotation container
        rotor_visual_path = f"{rotor_rotation_path}/rotor_visual"
        rotor_geom = UsdGeom.Cube.Define(stage, rotor_visual_path)

        # Set visual geometry properties
        rotor_geom.CreateSizeAttr(rotor_radius * 2)  # Blade span

        # Apply blade-like scaling (long, narrow, thin)
        rotor_geom.AddScaleOp().Set(Gf.Vec3d(1.0, 0.15, 0.02))

        # Apply dark gray color
        if hasattr(rotor_geom, "CreateDisplayColorAttr"):
            rotor_geom.CreateDisplayColorAttr([(0.2, 0.2, 0.2)])  # Dark gray

        # NO physics applied to visual - pure mesh only!
        # Rotation applied to container, scale applied to mesh - no interference!

        # 3. Add fixed joint to connect physics component to main body
        fixed_joint = self._add_fixed_joint(stage_prefix, rotor_index)

    def _add_fixed_joint(self, stage_prefix: str, rotor_index: int):
        """
        Add a fixed joint to rigidly connect rotor physics component to main body.
        This allows forces applied to rotor to propagate through the articulation.
        """

        # Get USD stage
        from omni.usd import get_context

        stage = get_context().get_stage()

        # Create fixed joint as child of rotor
        joint_path = f"{stage_prefix}/body/rotor{rotor_index}/joint{rotor_index}"
        joint = UsdPhysics.FixedJoint.Define(stage, joint_path)

        # Connect main body rigid body to rotor physics rigid body
        joint.CreateBody0Rel().SetTargets(
            [f"{stage_prefix}/body/body_mesh"]
        )  # Parent: main body
        joint.CreateBody1Rel().SetTargets(
            [f"{stage_prefix}/body/rotor{rotor_index}/rotor_physics"]
        )  # Child: rotor physics

        return joint

    def _create_gimbal_mount(self, stage_prefix: str):
        """
        Create gimbal mount point if gimbal is enabled.
        """

        gimbal_config = self.vehicle_config["vehicle"]["gimbal"]
        mount_position = gimbal_config.get("mount_position", [0.0, 0.0, -0.05])

        # Create mount point as a small cube
        mount = DynamicCuboid(
            prim_path=f"{stage_prefix}/gimbal_mount",
            name="gimbal_mount",
            position=np.array(mount_position),
            size=0.03,  # Small mount
            color=np.array([100, 100, 100]),
            mass=0.1,
        )

    def _setup_articulation(self, stage_prefix: str):
        """
        Apply ArticulationRootAPI to the vehicle root to create proper articulation.
        This must be called AFTER all the vehicle structure is created.
        """
        from omni.usd import get_context

        stage = get_context().get_stage()
        root_prim = stage.GetPrimAtPath(stage_prefix)
        UsdPhysics.ArticulationRootAPI.Apply(root_prim)

    def _setup_vehicle_components(self):
        """
        Setup sensors, dynamics, and backends based on configuration.
        """

        # Setup backends (always use PX4 MAVLink)
        backend = PX4Backend(config=PX4BackendConfig())

        # Setup sensors
        sensors = []
        sensor_config = self.vehicle_config["vehicle"]["sensors"]

        if sensor_config.get("imu", {}).get("enabled", False):
            carb.log_warn("Add IMU")
            imu_config = sensor_config["imu"]
            sensors.append(
                IMU(
                    {
                        "update_rate": imu_config.get("update_rate", 250),
                    }
                )
            )

        if sensor_config.get("gps", {}).get("enabled", False):
            carb.log_warn("Add GPS")
            gps_config = sensor_config["gps"]
            sensors.append(
                GPS(
                    {
                        "update_rate": gps_config.get("update_rate", 10),
                    }
                )
            )

        if sensor_config.get("barometer", {}).get("enabled", False):
            carb.log_warn("Add Baro")
            baro_config = sensor_config["barometer"]
            sensors.append(
                Barometer(
                    {
                        "update_rate": baro_config.get("update_rate", 50),
                    }
                )
            )

        if sensor_config.get("magnetometer", {}).get("enabled", False):
            carb.log_warn("Add Mag")
            mag_config = sensor_config["magnetometer"]
            sensors.append(
                Magnetometer(
                    {
                        "update_rate": mag_config.get("update_rate", 100),
                    }
                )
            )

        # Setup graphical sensors (including gimbal)
        graphical_sensors = []

        # Add gimbal if configured
        if self.vehicle_config["vehicle"].get("gimbal", {}).get("enabled", False):
            gimbal_config_file = self.vehicle_config["vehicle"]["gimbal"]["config_file"]
            gimbal_system = GimbalSystem(gimbal_config_file, "gimbal_mount")
            graphical_sensors.append(gimbal_system)

        # Setup drag
        drag = LinearDrag([0.50, 0.30, 0.0])

        # Setup graphs (empty for now)
        graphs = []

        # Store thrust curve and drag for vehicle dynamics
        self._drag = drag

        return sensors, graphical_sensors, graphs, backend

    def _setup_force_generators(self):
        """
        Setup force generators with explicit indices.
        Registers rotors as SpinningBody force generators.
        Joint handles will be connected later when simulation starts.
        """
        # Get motor parameters
        thrust_coeff = self.motor_params["rotor_constant"]
        # Note: Removed 2x boost - Iris motor should provide sufficient power
        torque_coeff = self.motor_params["torque_coefficient"]

        # Calculate rotor positions (same as in _create_quadrotor_x_structure)
        separation = self.rotor_separation
        rotor_height = 0.1  # 10cm above body center
        arm_length = separation / 2

        # PX4 X configuration positions - Isaac Sim coords: +X forward, +Y left, +Z up
        positions = [
            [+arm_length, -arm_length, rotor_height],  # Motor 0: Front-right (+X, -Y)
            [-arm_length, +arm_length, rotor_height],  # Motor 1: Rear-left (-X, +Y)
            [+arm_length, +arm_length, rotor_height],  # Motor 2: Front-left (+X, +Y)
            [-arm_length, -arm_length, rotor_height],  # Motor 3: Rear-right (-X, -Y)
        ]

        # Standard rotation directions (alternating for stability)
        directions = [-1, -1, 1, 1]  # CCW, CCW, CW, CW

        # Register each rotor with explicit indices 0-3
        for i, (pos, direction) in enumerate(zip(positions, directions)):
            self.add_spinning_body(
                index=i,  # Explicit index
                position=pos,
                thrust_coefficient=thrust_coeff,
                torque_coefficient=torque_coeff,
                spin_direction=direction,
            )

            # Set visual path for animation (target the rotation container, not the mesh)
            visual_path = (
                f"{self._stage_prefix}/body/rotor{i}/rotor_physics/rotor_rotation"
            )
            self.components[i].set_visual_path(visual_path)

        carb.log_info(
            f"Registered 4 rotor force generators with indices 0-3 (joint handles will be connected when simulation starts)"
        )

    def _connect_joint_handles(self):
        """
        Connect joint handles to spinning bodies.
        Called when simulation starts and dynamic control interface is available.
        """
        # Get articulation - now that simulation is running
        articulation = self.get_dc_interface().get_articulation(self._stage_prefix)

        if not articulation:
            carb.log_warn(
                "Could not find articulation - joint handles will not be connected"
            )
            return

        # Using fixed joints for physics and transform-based visual animation
        carb.log_info(
            "Using fixed joints for physics and transform-based visual rotation"
        )

    def start(self):
        """Called when simulation starts - connect joint handles for visual rotation."""
        self._connect_joint_handles()

    def stop(self):
        """In this case we do not need to do anything extra when the simulation stops"""
        pass

    def enable_manual_control(self):
        """Enable manual motor control mode (overrides PX4 commands)"""
        self._manual_control_enabled = True
        carb.log_info("Manual motor control enabled - overriding PX4 commands")

    def disable_manual_control(self):
        """Disable manual motor control mode (returns to PX4 control)"""
        self._manual_control_enabled = False
        self._manual_motor_speeds = [0.0, 0.0, 0.0, 0.0]  # Reset to zero
        carb.log_info("Manual motor control disabled - returning to PX4 control")

    def set_manual_motor_speeds(self, motor_speeds):
        """
        Set manual motor speeds (0-100% converted to rad/s)

        Args:
            motor_speeds (list): List of 4 motor speeds as percentages (0-100)
        """
        if len(motor_speeds) != 4:
            carb.log_error("Motor speeds must be a list of 4 values")
            return

        # Convert percentage to angular velocity (assume max 1000 rad/s)
        max_speed = 1000.0  # rad/s
        self._manual_motor_speeds = [
            (speed / 100.0) * max_speed for speed in motor_speeds
        ]

    def is_manual_control_enabled(self):
        """Check if manual control is currently enabled"""
        return self._manual_control_enabled

    def update(self, dt: float):
        """
        Update method called at each physics step.
        Apply forces and torques to the vehicle based on motor commands.
        """

        # Get desired angular velocities from backends or manual control
        if self._manual_control_enabled:
            # Use manual motor speeds from UI
            desired_rotor_velocities = self._manual_motor_speeds.copy()
        elif self._backend is not None:
            # Normal PX4 control path
            desired_rotor_velocities = self._backend.input_reference()
        else:
            desired_rotor_velocities = [0.0 for i in range(4)]

        # Convert list to indexed dictionary for new physics system
        inputs = {i: vel for i, vel in enumerate(desired_rotor_velocities)}

        # Apply forces using the new physics system
        # Components handle their own physics and visual rotation!
        self.apply_forces(inputs, dt)

        # Apply drag forces to body using standardized body access
        drag = self._drag.update(self._state, dt)
        self.apply_force(drag, body_part=self.relative_body_path)

        # Update backend
        if self._backend:
            self._backend.update(dt)

    # ===============================================================
    # ---- Component Registration and Access Methods ----
    # ===============================================================

    def _register_components(self):
        """Register all multirotor components for standardized access."""
        # Register rotors - note: rotors are visual-only, but we register paths for force application
        for i in range(4):
            # Register rotor visual path (rotation container for animation, actual mesh is child)
            self.register_component(
                f"rotor{i}_visual",
                f"{self._stage_prefix}/body/rotor{i}/rotor_physics/rotor_rotation/rotor_visual",
            )
            # Register rotor physics path (for force application)
            self.register_component(
                f"rotor{i}_physics", f"{self._stage_prefix}/body/rotor{i}/rotor_physics"
            )
            # Register joint path (for diagnostics)
            self.register_component(
                f"joint{i}", f"{self._stage_prefix}/body/rotor{i}/joint{i}"
            )

        # Register gimbal mount if present
        if self.vehicle_config["vehicle"].get("gimbal", {}).get("enabled", False):
            self.register_component(
                "gimbal_mount", f"{self._stage_prefix}/body/gimbal_mount"
            )

    @property
    def rotor_count(self) -> int:
        """Get number of rotors."""
        return 4  # Standard quadrotor

    def get_rotor_path(self, index: int) -> str:
        """Get path to specific rotor.

        Args:
            index (int): Rotor index (0-3)

        Returns:
            str: Path to rotor or None if invalid index
        """
        if 0 <= index < self.rotor_count:
            return self.get_component_path(f"rotor{index}")
        return None

    def get_rotor_joint_path(self, index: int) -> str:
        """Get path to rotor joint.

        Args:
            index (int): Rotor index (0-3)

        Returns:
            str: Path to rotor joint or None if invalid index
        """
        if 0 <= index < self.rotor_count:
            return self.get_component_path(f"joint{index}")
        return None

    def get_all_rotor_paths(self) -> list:
        """Get all rotor paths.

        Returns:
            list: List of all rotor paths
        """
        return [self.get_rotor_path(i) for i in range(self.rotor_count)]

    def get_gimbal_mount_path(self) -> str:
        """Get gimbal mount path if gimbal is enabled.

        Returns:
            str: Path to gimbal mount or None if no gimbal
        """
        return self.get_component_path("gimbal_mount")
