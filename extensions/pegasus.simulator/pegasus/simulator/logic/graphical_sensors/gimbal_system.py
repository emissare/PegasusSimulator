"""
| File: gimbal_system.py
| Author: EmissarePegasusSimulator Team
| License: BSD-3-Clause. Copyright (c) 2024. All rights reserved.
| Description: Multi-sensor gimbal system with 3-axis articulation
"""
__all__ = ["GimbalSystem"]

import yaml
import time
import numpy as np
from pathlib import Path

# Isaac Sim imports
from isaacsim.core.api.objects import DynamicCuboid
from pxr import UsdGeom, UsdPhysics, Gf, Usd
from omni.usd import get_context

# Pegasus imports
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.graphical_sensors import GraphicalSensor
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.graphical_sensors.laser_rangefinder import LaserRangefinder
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Scipy for rotations
from scipy.spatial.transform import Rotation


class GimbalSystem(GraphicalSensor):
    """
    Multi-sensor gimbal system with 3-axis stabilization.
    Supports color camera, monochrome camera, and laser rangefinder.
    """

    def __init__(self, gimbal_config_file: str, mount_point: str):
        """
        Initialize the GimbalSystem from YAML configuration.

        Args:
            gimbal_config_file (str): Path to gimbal configuration YAML file
            mount_point (str): Mount point prim name on vehicle (e.g., "gimbal_mount")
        """

        # Load gimbal configuration using path utilities
        from pegasus.simulator.utils.paths import resolve_config_path, ensure_config_file_exists

        config_path = resolve_config_path(gimbal_config_file)
        config_path = ensure_config_file_exists(config_path, "gimbal config")

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Initialize the GraphicalSensor base class
        super().__init__(
            sensor_type="GimbalSystem",
            update_rate=self.config['gimbal'].get('control', {}).get('status_rate', 10.0)
        )

        self.mount_point = mount_point
        self.gimbal_config = self.config['gimbal']

        # Gimbal state
        self._gimbal_angles = {
            'yaw': 0.0,
            'pitch': self.gimbal_config['joints']['pitch']['default_position'],
            'roll': 0.0
        }

        # Joint drives for control
        self._joint_drives = {}

        # Sensors attached to gimbal
        self._sensors = {}

        # Gimbal structure prims
        self._gimbal_prims = {}

        # Control state
        self._control_mode = self.gimbal_config.get('control', {}).get('mode', 'position')
        self._stabilization_enabled = self.gimbal_config.get('control', {}).get('stabilization', {}).get('enabled', True)

        # Output state
        self._state = {
            'timestamp': 0.0,
            'angles': self._gimbal_angles.copy(),
            'sensors': {}
        }

    def initialize(self, vehicle):
        """
        Initialize the gimbal system when attached to a vehicle.
        """
        super().initialize(vehicle)

        # Create gimbal structure
        self._create_gimbal_structure()

        # Attach sensors
        self._attach_sensors()

    def _create_gimbal_structure(self):
        """
        Create the 3-axis gimbal structure with revolute joints.
        """

        vehicle_path = self._vehicle.prim_path
        gimbal_base_path = f"{vehicle_path}/body/{self.mount_point}/gimbal"

        # Get USD stage
        stage = get_context().get_stage()

        # Create gimbal mount base
        self._create_gimbal_mount(gimbal_base_path)

        # Create yaw assembly (first rotation - Z axis)
        yaw_path = f"{gimbal_base_path}/yaw_assembly"
        self._create_gimbal_link(yaw_path, "yaw_link")
        self._create_revolute_joint(stage, "yaw_joint", gimbal_base_path, yaw_path, "Z")

        # Create roll assembly (second rotation - X axis)
        roll_path = f"{yaw_path}/roll_assembly"
        self._create_gimbal_link(roll_path, "roll_link")
        self._create_revolute_joint(stage, "roll_joint", yaw_path, roll_path, "X")

        # Create pitch assembly (third rotation - Y axis)
        pitch_path = f"{roll_path}/pitch_assembly"
        self._create_gimbal_link(pitch_path, "pitch_link")
        self._create_revolute_joint(stage, "pitch_joint", roll_path, pitch_path, "Y")

        # Store sensor mount point
        self._sensor_mount_path = f"{pitch_path}/sensor_mount"
        self._create_sensor_mount(self._sensor_mount_path)

    def _create_gimbal_mount(self, base_path: str):
        """Create the main gimbal mount structure."""

        structure_config = self.gimbal_config.get('structure', {})
        mount_size = structure_config.get('mount_size', [0.08, 0.08, 0.04])
        mount_color = structure_config.get('mount_color', [80, 80, 80])

        mount = DynamicCuboid(
            prim_path=base_path,
            name="gimbal_base",
            position=np.array([0.0, 0.0, 0.0]),
            size=max(mount_size),
            scale=np.array([
                mount_size[0] / max(mount_size),
                mount_size[1] / max(mount_size),
                mount_size[2] / max(mount_size)
            ]),
            color=np.array(mount_color),
            mass=0.2
        )

        self._gimbal_prims['base'] = mount

    def _create_gimbal_link(self, link_path: str, link_name: str):
        """Create a gimbal link/assembly."""

        link = DynamicCuboid(
            prim_path=link_path,
            name=link_name,
            position=np.array([0.0, 0.0, 0.0]),
            size=0.03,  # Small link
            color=np.array([120, 120, 120]),
            mass=0.05
        )

        self._gimbal_prims[link_name] = link

    def _create_sensor_mount(self, mount_path: str):
        """Create the sensor mounting platform."""

        mount = DynamicCuboid(
            prim_path=mount_path,
            name="sensor_mount",
            position=np.array([0.0, 0.0, 0.0]),
            size=0.04,  # Platform for sensors
            color=np.array([150, 150, 150]),
            mass=0.1
        )

        self._gimbal_prims['sensor_mount'] = mount

    def _create_revolute_joint(self, stage, joint_name: str, parent_path: str, child_path: str, axis: str):
        """Create a revolute joint between two bodies."""

        joint_path = f"{parent_path}/{joint_name}"
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)

        # Set joint axis
        joint.CreateAxisAttr(axis)

        # Set body relationships
        joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([child_path])

        # Set joint limits based on configuration
        axis_lower = axis.lower()
        if axis_lower in self.gimbal_config['joints']:
            joint_config = self.gimbal_config['joints'][axis_lower]
            joint_range = joint_config['range']

            # Convert degrees to radians
            lower_limit = np.radians(joint_range[0])
            upper_limit = np.radians(joint_range[1])

            joint.CreateLowerLimitAttr(lower_limit)
            joint.CreateUpperLimitAttr(upper_limit)

        # Add joint drive for position control
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateStiffnessAttr(1000.0)  # High stiffness for position control
        drive.CreateDampingAttr(100.0)    # Damping for stability

        # Store joint reference
        self._joint_drives[axis_lower] = {
            'joint': joint,
            'drive': drive,
            'path': joint_path
        }

    def _attach_sensors(self):
        """Attach sensors to the gimbal based on configuration."""

        sensor_configs = self.gimbal_config.get('sensors', {})

        for sensor_name, sensor_config in sensor_configs.items():
            if not sensor_config.get('enabled', True):
                continue

            sensor_type = sensor_config['type']

            if sensor_type == 'monocular_camera':
                sensor = self._create_camera_sensor(sensor_name, sensor_config)
            elif sensor_type == 'laser_rangefinder':
                sensor = self._create_rangefinder_sensor(sensor_name, sensor_config)
            else:
                print(f"Warning: Unsupported sensor type: {sensor_type}")
                continue

            self._sensors[sensor_name] = sensor

    def _create_camera_sensor(self, sensor_name: str, sensor_config: dict):
        """Create a camera sensor attached to the gimbal."""

        # Convert config to MonocularCamera format
        camera_config = {
            'position': np.array(sensor_config['position']),
            'orientation': np.array(sensor_config['orientation']),
            'resolution': sensor_config['resolution'],
            'frequency': sensor_config['frequency'],
            'diagonal_fov': sensor_config['fov'],
            'depth': sensor_config.get('depth', False)
        }

        # Create camera relative to sensor mount
        camera = MonocularCamera(f"{self._sensor_mount_path}/{sensor_name}", camera_config)
        camera.initialize(self._vehicle)

        return camera

    def _create_rangefinder_sensor(self, sensor_name: str, sensor_config: dict):
        """Create a laser rangefinder sensor attached to the gimbal."""

        # Convert config to LaserRangefinder format
        rangefinder_config = {
            'position': np.array(sensor_config['position']),
            'orientation': np.array(sensor_config['orientation']),
            'max_range': sensor_config['max_range'],
            'min_range': sensor_config['min_range'],
            'frequency': sensor_config['frequency'],
            'beam_width': sensor_config.get('beam_width', 0.01)
        }

        # Create rangefinder relative to sensor mount
        rangefinder = LaserRangefinder(f"{self._sensor_mount_path}/{sensor_name}", rangefinder_config)
        rangefinder.initialize(self._vehicle)

        return rangefinder

    def set_angles(self, pitch: float, roll: float, yaw: float):
        """
        Set gimbal angles manually.

        Args:
            pitch (float): Pitch angle in degrees
            roll (float): Roll angle in degrees
            yaw (float): Yaw angle in degrees
        """

        # Clamp angles to joint limits
        pitch = self._clamp_angle('pitch', pitch)
        roll = self._clamp_angle('roll', roll)
        yaw = self._clamp_angle('yaw', yaw)

        # Update internal state
        self._gimbal_angles['pitch'] = pitch
        self._gimbal_angles['roll'] = roll
        self._gimbal_angles['yaw'] = yaw

        # Apply to joint drives
        self._apply_joint_targets()

    def _clamp_angle(self, axis: str, angle: float) -> float:
        """Clamp angle to joint limits."""

        joint_config = self.gimbal_config['joints'][axis]
        joint_range = joint_config['range']

        return np.clip(angle, joint_range[0], joint_range[1])

    def _apply_joint_targets(self):
        """Apply current angle targets to joint drives."""

        for axis, angle_deg in self._gimbal_angles.items():
            if axis in self._joint_drives:
                angle_rad = np.radians(angle_deg)
                drive = self._joint_drives[axis]['drive']
                drive.GetTargetPositionAttr().Set(angle_rad)

    def start(self):
        """Start the gimbal system."""
        # Initialize sensors
        for sensor in self._sensors.values():
            if hasattr(sensor, 'start'):
                sensor.start()

        # Set default position
        self._apply_joint_targets()

    def stop(self):
        """Stop the gimbal system."""
        # Stop sensors
        for sensor in self._sensors.values():
            if hasattr(sensor, 'stop'):
                sensor.stop()

    @GraphicalSensor.update_at_rate
    def update(self, state: State, dt: float):
        """
        Update gimbal system and sensors.
        """

        # Update stabilization if enabled
        if self._stabilization_enabled:
            self._update_stabilization(state)

        # Update sensors
        sensor_data = {}
        for name, sensor in self._sensors.items():
            if hasattr(sensor, 'update'):
                data = sensor.update(state, dt)
                if data is not None:
                    sensor_data[name] = data

        # Update state
        self._state = {
            'timestamp': time.time(),
            'angles': self._gimbal_angles.copy(),
            'sensors': sensor_data,
            'control_mode': self._control_mode,
            'stabilization_enabled': self._stabilization_enabled
        }

        return self._state

    def _update_stabilization(self, state: State):
        """
        Update gimbal stabilization to compensate for vehicle motion.
        """

        # Get vehicle attitude
        vehicle_quat = state.attitude_inertial_frame  # [qw, qx, qy, qz]

        # Convert to Euler angles
        vehicle_rot = Rotation.from_quat([vehicle_quat[1], vehicle_quat[2], vehicle_quat[3], vehicle_quat[0]])
        vehicle_euler = vehicle_rot.as_euler('xyz', degrees=True)

        # Compensate for vehicle roll and pitch (keep yaw following vehicle)
        compensation_rate = self.gimbal_config.get('control', {}).get('stabilization', {}).get('compensation_rate', 0.8)

        # Apply compensation (simplified stabilization)
        compensated_roll = -vehicle_euler[0] * compensation_rate
        compensated_pitch = self._gimbal_angles['pitch'] - vehicle_euler[1] * compensation_rate

        # Clamp compensated angles
        compensated_roll = self._clamp_angle('roll', compensated_roll)
        compensated_pitch = self._clamp_angle('pitch', compensated_pitch)

        # Apply compensation
        self._gimbal_angles['roll'] = compensated_roll
        self._gimbal_angles['pitch'] = compensated_pitch

        # Update joint targets
        self._apply_joint_targets()

    @property
    def state(self):
        """Get current gimbal state."""
        return self._state

    @property
    def sensors(self):
        """Get attached sensors."""
        return self._sensors

    def get_sensor_data(self):
        """Get data from all sensors."""
        return self._state.get('sensors', {})