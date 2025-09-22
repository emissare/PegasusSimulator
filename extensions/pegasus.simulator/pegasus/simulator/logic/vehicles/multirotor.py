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
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig

# Sensors and dynamics setup
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS

# For gimbal support
from pegasus.simulator.logic.graphical_sensors.gimbal_system import GimbalSystem




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
            raise ValueError("config_file parameter is required - must specify path to YAML vehicle configuration")

        # Load vehicle configuration from YAML
        config_path = Path(config_file)
        if not config_path.exists():
            raise FileNotFoundError(f"Vehicle config file not found: {config_file}")

        with open(config_path, 'r') as f:
            self.vehicle_config = yaml.safe_load(f)

        # Load motor database using path utilities
        from pegasus.simulator.utils.paths import get_motor_db_path, ensure_config_file_exists

        motor_db_path = ensure_config_file_exists(get_motor_db_path(), "motor database")
        with open(motor_db_path, 'r') as f:
            motor_database = yaml.safe_load(f)

        # Get motor parameters
        motor_name = self.vehicle_config['vehicle']['motor']
        if motor_name not in motor_database['motors']:
            raise ValueError(f"Motor '{motor_name}' not found in motor database")
        self.motor_params = motor_database['motors'][motor_name]

        # Extract vehicle parameters
        self.vehicle_type = self.vehicle_config['vehicle']['type']
        self.rotor_separation = self.vehicle_config['vehicle']['rotor_separation']
        self.vehicle_mass = self.vehicle_config['vehicle']['mass']

        # Setup sensors, thrusters, and backends first
        sensors, graphical_sensors, graphs, backends = self._setup_vehicle_components()

        # Initialize parent Vehicle class WITHOUT USD file (empty string)
        super().__init__(
            stage_prefix,
            "",
            init_pos,
            init_orientation,
            sensors,
            graphical_sensors,
            graphs,
            backends
        )

        # Manual motor control attributes
        self._manual_control_enabled = False
        self._manual_motor_speeds = [0.0, 0.0, 0.0, 0.0]  # Angular velocities in rad/s

        # Create vehicle structure programmatically AFTER parent class initialization
        self._create_vehicle_structure(stage_prefix)

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

        # Standard X configuration rotor positions (above body)
        separation = self.rotor_separation
        rotor_height = 0.1  # 10cm above body
        rotor_positions = [
            [separation/2, separation/2, rotor_height],    # Front-right
            [-separation/2, separation/2, rotor_height],   # Front-left
            [-separation/2, -separation/2, rotor_height],  # Rear-left
            [separation/2, -separation/2, rotor_height]    # Rear-right
        ]

        # Standard rotation directions (matching current Iris: [-1, -1, 1, 1])
        rotor_directions = [-1, -1, 1, 1]  # CCW, CCW, CW, CW

        # Build the multirotor structure
        self._build_multirotor_structure(stage_prefix, rotor_positions, rotor_directions)

    def _build_multirotor_structure(self, stage_prefix: str, rotor_positions: list, rotor_directions: list):
        """
        Build the physical multirotor structure with body and rotors.
        """

        # Create main body
        self._create_body(stage_prefix)

        # Create rotors
        for i, (pos, direction) in enumerate(zip(rotor_positions, rotor_directions)):
            self._create_rotor(stage_prefix, i, pos, direction)

        # Create gimbal mount if enabled
        if self.vehicle_config['vehicle'].get('gimbal', {}).get('enabled', False):
            self._create_gimbal_mount(stage_prefix)

        # Setup articulation root after all structure is created
        self._setup_articulation(stage_prefix)

    def _create_body(self, stage_prefix: str):
        """
        Create the main body of the vehicle as a DynamicCuboid.
        """

        body_config = self.vehicle_config['vehicle']['body']
        dimensions = body_config['dimensions']
        color = body_config['color']

        # Create dynamic cuboid for the body
        self.body = DynamicCuboid(
            prim_path=f"{stage_prefix}/body",
            name="body",
            position=np.array([0.0, 0.0, 0.0]),
            size=max(dimensions),  # DynamicCuboid uses single size parameter
            scale=np.array([
                dimensions[0] / max(dimensions),
                dimensions[1] / max(dimensions),
                dimensions[2] / max(dimensions)
            ]),
            color=np.array(color),
            mass=self.vehicle_mass
        )

    def _create_rotor(self, stage_prefix: str, rotor_index: int, position: list, direction: int):
        """
        Create a single rotor disk at the specified position with the given rotation direction.
        """

        rotor_radius = self.motor_params['rotor_radius']

        # Create single rotor rigid body that can receive forces
        rotor = DynamicCylinder(
            prim_path=f"{stage_prefix}/rotor{rotor_index}",
            name=f"rotor{rotor_index}",
            position=np.array(position),
            radius=rotor_radius,
            height=0.01,  # Thin disk to represent rotor
            color=np.array([50, 50, 50]),  # Dark gray
            mass=0.05  # Total rotor mass
        )

        # Add revolute joint for visual spinning effect
        self._add_rotor_joint(stage_prefix, rotor_index)

    def _add_rotor_joint(self, stage_prefix: str, rotor_index: int):
        """
        Add a revolute joint to a rotor for visual rotation effects.
        """

        # Get USD stage
        from omni.usd import get_context
        stage = get_context().get_stage()

        # Create revolute joint for the rotor
        joint_path = f"{stage_prefix}/joint{rotor_index}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

        # Set joint properties
        joint.CreateAxisAttr("Z")  # Rotation around Z-axis
        joint.CreateBody0Rel().SetTargets([f"{stage_prefix}/body"])
        joint.CreateBody1Rel().SetTargets([f"{stage_prefix}/rotor{rotor_index}"])

        # Set joint limits (free rotation)
        joint.CreateLowerLimitAttr(-3.14159)
        joint.CreateUpperLimitAttr(3.14159)

    def _create_gimbal_mount(self, stage_prefix: str):
        """
        Create gimbal mount point if gimbal is enabled.
        """

        gimbal_config = self.vehicle_config['vehicle']['gimbal']
        mount_position = gimbal_config.get('mount_position', [0.0, 0.0, -0.05])

        # Create mount point as a small cube
        mount = DynamicCuboid(
            prim_path=f"{stage_prefix}/gimbal_mount",
            name="gimbal_mount",
            position=np.array(mount_position),
            size=0.03,  # Small mount
            color=np.array([100, 100, 100]),
            mass=0.1
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
        Setup sensors, thrusters, dynamics, and backends based on configuration.
        """

        # Setup backends (always use PX4 MAVLink)
        backends = [PX4MavlinkBackend(config=PX4MavlinkBackendConfig())]

        # Setup sensors
        sensors = []
        sensor_config = self.vehicle_config['vehicle']['sensors']

        if sensor_config.get('imu', {}).get('enabled', False):
            imu_config = sensor_config['imu']
            sensors.append(IMU({
                "frequency": imu_config.get('frequency', 250),
                "pos": imu_config.get('position', [0.0, 0.0, 0.0])
            }))

        if sensor_config.get('gps', {}).get('enabled', False):
            gps_config = sensor_config['gps']
            sensors.append(GPS({
                "frequency": gps_config.get('frequency', 10),
                "pos": gps_config.get('position', [0.0, 0.0, 0.02])
            }))

        if sensor_config.get('barometer', {}).get('enabled', False):
            baro_config = sensor_config['barometer']
            sensors.append(Barometer({
                "frequency": baro_config.get('frequency', 50),
                "pos": baro_config.get('position', [0.0, 0.0, 0.01])
            }))

        # Setup graphical sensors (including gimbal)
        graphical_sensors = []

        # Add gimbal if configured
        if self.vehicle_config['vehicle'].get('gimbal', {}).get('enabled', False):
            gimbal_config_file = self.vehicle_config['vehicle']['gimbal']['config_file']
            gimbal_system = GimbalSystem(gimbal_config_file, "gimbal_mount")
            graphical_sensors.append(gimbal_system)

        # Setup thrust curve using motor parameters
        thrust_curve = self._setup_thrust_curve()

        # Setup drag
        drag = LinearDrag([0.50, 0.30, 0.0])

        # Setup graphs (empty for now)
        graphs = []

        # Store thrust curve and drag for vehicle dynamics
        self._thrusters = thrust_curve
        self._drag = drag

        return sensors, graphical_sensors, graphs, backends

    def _setup_thrust_curve(self):
        """
        Setup quadratic thrust curve using motor parameters from database.
        """

        # Extract motor parameters
        rotor_constant = self.motor_params['rotor_constant']
        rolling_moment_coeff = self.motor_params['rolling_moment_coefficient']
        max_velocity = self.motor_params['max_rotor_velocity']

        # Create thrust curve configuration
        thrust_config = {
            "num_rotors": 4,
            "rotor_constant": [rotor_constant] * 4,
            "rolling_moment_coefficient": [rolling_moment_coeff] * 4,
            "rot_dir": [-1, -1, 1, 1],  # Match Iris configuration
            "max_rotor_velocity": [max_velocity] * 4,
            "min_rotor_velocity": [0.0] * 4,
        }

        return QuadraticThrustCurve(thrust_config)

    def start(self):
        """In this case we do not need to do anything extra when the simulation starts"""
        pass

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

        # Get the articulation root of the vehicle
        articulation = self.get_dc_interface().get_articulation(self._stage_prefix)

        # Get desired angular velocities from backends or manual control
        if self._manual_control_enabled:
            # Use manual motor speeds from UI
            desired_rotor_velocities = self._manual_motor_speeds.copy()
        elif len(self._backends) != 0:
            # Normal PX4 control path
            desired_rotor_velocities = self._backends[0].input_reference()
        else:
            desired_rotor_velocities = [0.0 for i in range(4)]

        # Update thruster model
        self._thrusters.set_input_reference(desired_rotor_velocities)
        forces_z, _, rolling_moment = self._thrusters.update(self._state, dt)

        # Apply forces to rotors
        for i in range(4):
            self.apply_force([0.0, 0.0, forces_z[i]], body_part=f"/rotor{i}")

            # Handle propeller visual effect
            if forces_z[i] > 0.1:
                # Get rotor joint and set velocity for visual effect
                joint = self.get_dc_interface().find_articulation_dof(articulation, f"joint{i}")
                if joint is not None:
                    self.get_dc_interface().set_dof_velocity(
                        joint, 100 * self._thrusters.rot_dir[i]
                    )

        # Apply rolling moment to body
        self.apply_torque([0.0, 0.0, rolling_moment], "/body")

        # Apply drag forces
        drag = self._drag.update(self._state, dt)
        self.apply_force(drag, body_part="/body")

        # Update all backends
        for backend in self._backends:
            backend.update(dt)