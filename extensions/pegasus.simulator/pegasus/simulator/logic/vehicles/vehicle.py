"""
| File: vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Definition of the Vehicle class which is used as the base for all the vehicles.
"""

# Numerical computations
import numpy as np
from scipy.spatial.transform import Rotation

# Low level APIs
import carb
from pxr import Usd, Gf

# High level Isaac sim APIs
import omni.usd
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from omni.usd import get_stage_next_free_path
from isaacsim.core.api.robots.robot import Robot
from omni.isaac.dynamic_control import _dynamic_control

# Extension APIs
from pegasus.simulator.logic.vehicle_state import VehicleState
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicle_manager import VehicleManager
from pegasus.simulator.logic.force_generators import (
    ForceGenerator,
    SpinningBody,
    LiftingSurface,
)


def get_world_transform_xform(prim: Usd.Prim):
    """
    Get the local transformation of a prim using omni.usd.get_world_transform_matrix().
    See https://docs.omniverse.nvidia.com/kit/docs/omni.usd/latest/omni.usd/omni.usd.get_world_transform_matrix.html
    Args:
        prim (Usd.Prim): The prim to calculate the world transformation.
    Returns:
        A tuple of:
        - Translation vector.
        - Rotation quaternion, i.e. 3d vector plus angle.
        - Scale vector.
    """
    world_transform: Gf.Matrix4d = omni.usd.get_world_transform_matrix(prim)
    rotation: Gf.Rotation = world_transform.ExtractRotation()
    return rotation


class Vehicle(Robot):

    def __init__(
        self,
        stage_prefix: str,
        usd_path: str = None,
        init_pos_enu_m=[0.0, 0.0, 0.0],
        init_orientation_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        sensors=[],
        graphical_sensors=[],
        graphs=[],
        backend=None,
    ):
        """
        Class that initializes a vehicle in the isaac sim's curent stage

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "quadrotor".
            usd_path (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            init_pos_enu_m (list): The initial position of the vehicle in the ENU inertial frame in meters. Defaults to [0.0, 0.0, 0.0].
            init_orientation_quat_xyzw (list): The initial orientation of the vehicle as quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
        """

        # Get the current world at which we want to spawn the vehicle
        self._world = PegasusInterface().world
        self._current_stage = self._world.stage

        # Save the name with which the vehicle will appear in the stage
        # and the name of the .usd file that contains its description
        self._stage_prefix = get_stage_next_free_path(
            self._current_stage, stage_prefix, False
        )
        self._usd_file = usd_path

        # Get the vehicle name by taking the last part of vehicle stage prefix
        self._vehicle_name = self._stage_prefix.rpartition("/")[-1]

        # Spawn the vehicle primitive in the world's stage
        self._prim = define_prim(self._stage_prefix, "Xform")
        self._prim = get_prim_at_path(self._stage_prefix)

        # Only add USD reference if a file is provided (for programmatic vehicles, usd_path is empty)
        if self._usd_file and self._usd_file.strip():
            self._prim.GetReferences().AddReference(self._usd_file)

        # Initialize the "Robot" class
        # Note: we need to change the rotation to have qw first, because NVidia
        # does not keep a standard of quaternions inside its own libraries (not good, but okay)
        super().__init__(
            prim_path=self._stage_prefix,
            name=self._stage_prefix,
            position=init_pos_enu_m,
            orientation=[
                init_orientation_quat_xyzw[3],
                init_orientation_quat_xyzw[0],
                init_orientation_quat_xyzw[1],
                init_orientation_quat_xyzw[2],
            ],
            articulation_controller=None,
        )

        self._vehicle_dc_interface = None

        # Add this object for the world to track, so that if we clear the world, this object is deleted from memory and
        # as a consequence, from the VehicleManager as well
        try:
            self._world.scene.add(self)
        except Exception as e:
            if "name is not unique" in str(e):
                carb.log_warn(
                    f"Vehicle {self._stage_prefix} already exists in scene, reusing existing entry"
                )
                # Remove the existing entry and try again
                try:
                    # Try to find and remove the existing object
                    for obj in self._world.scene._scene_registry:
                        if hasattr(obj, "name") and obj.name == self._stage_prefix:
                            self._world.scene._scene_registry.remove(obj)
                            break
                    # Now add this new one
                    self._world.scene.add(self)
                except:
                    carb.log_error(
                        f"Failed to resolve scene conflict for {self._stage_prefix}, continuing anyway"
                    )
            else:
                raise e

        VehicleManager.get_vehicle_manager().add_vehicle(self._stage_prefix, self)
        self._state = VehicleState()

        # Single monolithic physics callback for all updates
        self._world.add_physics_callback(
            self._stage_prefix + "/main_physics_update", self._main_physics_update
        )

        self._sim_running = False

        self._world.add_timeline_callback(
            self._stage_prefix + "/start_stop_sim", self.sim_start_stop
        )

        self._sensors = sensors
        for sensor in self._sensors:
            sensor.initialize(
                self,
                PegasusInterface().latitude_deg,
                PegasusInterface().longitude_deg,
                PegasusInterface().altitude_msl_m,
            )

        # Sensors now updated in main physics callback
        self._graphical_sensors = graphical_sensors

        for graphical_sensor in self._graphical_sensors:
            graphical_sensor.initialize(self)

        self._world.add_render_callback(
            self._stage_prefix + "/GraphicalSensors", self.update_graphical_sensors
        )

        self._graphs = graphs

        for graph in self._graphs:
            graph.initialize(self)

        self._backend = backend

        if self._backend:
            self._backend.initialize(self)

        self._component_paths = {}

        self.components = {}

    def __del__(self):
        """
        Method that is invoked when a vehicle object gets destroyed. When this happens, we also invoke the
        'remove_vehicle' from the VehicleManager in order to remove the vehicle from the list of active vehicles.
        """

        # Remove this object from the vehicleHandler (only if _stage_prefix exists)
        if hasattr(self, "_stage_prefix"):
            VehicleManager.get_vehicle_manager().remove_vehicle(self._stage_prefix)

    """
    Properties
    """

    @property
    def state(self):
        """The state of the vehicle.

        Returns:
            VehicleState: The current state of the vehicle, i.e., position, orientation, linear and angular velocities...
        """
        return self._state

    @property
    def vehicle_name(self) -> str:
        """Vehicle name.

        Returns:
            Vehicle name (str): last prim name in vehicle prim path
        """
        return self._vehicle_name

    # ===============================================================
    # ---- Component Access Interface ----
    # ===============================================================

    @property
    def body_path(self) -> str:
        """Get path to primary vehicle body (rigid body).

        Standard: All vehicles MUST have their primary rigid body at /[vehicle]/body/body_mesh.
        This follows the Pegasus structure where body is an Xform container with a mesh child that has physics.

        Returns:
            str: Path to primary vehicle body rigid body (e.g., "/quadrotor/body/body_mesh")
        """
        return f"{self._stage_prefix}/body/body_mesh"

    @property
    def relative_body_path(self) -> str:
        """Get relative path to primary vehicle body from stage prefix.

        This is the path expected by apply_force() and apply_torque() methods,
        which prefix it with self._stage_prefix.

        Returns:
            str: Relative path to body (e.g., "/body/body_mesh")
        """
        return "/body/body_mesh"

    @property
    def root_path(self) -> str:
        """Get path to vehicle root.

        Returns:
            str: Path to vehicle root (e.g., "/quadrotor")
        """
        return self._stage_prefix

    def register_component(self, name: str, path: str):
        """Register a component path for standardized access.

        This allows subclasses to register their specific components
        (rotors, wings, control surfaces, etc.) for type-safe access.

        Args:
            name (str): Component name (e.g., "rotor0", "left_wing")
            path (str): Full path to component (e.g., "/quadrotor/body/rotor0")
        """
        self._component_paths[name] = path

    def get_component_path(self, component_name: str) -> str:
        """Get path to named component.

        Args:
            component_name (str): Name of component to look up

        Returns:
            str: Path to component or None if not found
        """
        return self._component_paths.get(component_name)

    def get_registered_components(self) -> dict:
        """Get all registered components.

        Returns:
            dict: Dictionary of component_name -> path
        """
        return self._component_paths.copy()

    def get_body_rigid(self):
        """Get the primary body rigid body handle.

        Returns:
            Rigid body handle for the primary vehicle body
        """
        return self.get_dc_interface().get_rigid_body(self.body_path)

    def get_component_rigid(self, component_name: str):
        """Get rigid body handle for named component.

        Args:
            component_name (str): Name of component

        Returns:
            Rigid body handle or None if component not found
        """
        path = self.get_component_path(component_name)
        if path:
            return self.get_dc_interface().get_rigid_body(path)
        return None

    # ===============================================================
    # ---- Force Generator System ----
    # ===============================================================

    def add_spinning_body(
        self,
        index: int,
        position: list,
        thrust_coefficient: float,
        torque_coefficient: float,
        spin_direction: int,
    ) -> int:
        """
        Add a motor-driven rotor that generates thrust force and reaction torque.

        Args:
            index (int): Explicit index for this component
            position (list): Position relative to body center [x, y, z]
            thrust_coefficient (float): Thrust per (angular velocity)²
            torque_coefficient (float): Motor torque per (angular velocity)²
            spin_direction (int): +1 for clockwise, -1 for counter-clockwise

        Returns:
            int: Index of the added generator (for reference in control inputs)
        """
        if index in self.components:
            raise ValueError(f"Component index {index} already in use")

        import numpy as np

        generator = SpinningBody(
            position=np.array(position),
            thrust_coefficient=thrust_coefficient,
            torque_coefficient=torque_coefficient,
            spin_direction=spin_direction,
        )
        self.components[index] = generator
        return index

    def add_lifting_surface(
        self,
        index: int,
        position: list,
        lift_coefficient: float,
        lift_direction: list = [0, 0, 1],
    ) -> int:
        """
        Add a lifting surface that generates only aerodynamic forces.

        Args:
            index (int): Explicit index for this component
            position (list): Position relative to body center [x, y, z]
            lift_coefficient (float): Lift force per unit deflection angle
            lift_direction (list): Unit vector for lift direction (default: +Z)

        Returns:
            int: Index of the added generator (for reference in control inputs)
        """
        if index in self.components:
            raise ValueError(f"Component index {index} already in use")

        import numpy as np

        generator = LiftingSurface(
            position=np.array(position),
            lift_coefficient=lift_coefficient,
            lift_direction=np.array(lift_direction),
        )
        self.components[index] = generator
        return index

    def apply_forces(self, inputs: dict, dt: float = 1.0 / 60.0):
        """
        Apply forces and torques from components based on input dictionary.

        This is the core physics method that applies forces at specific positions
        to create natural moments. Isaac Sim automatically calculates the resulting
        pitch and roll moments from force position offsets.

        Args:
            inputs (dict): Control inputs keyed by component index
                          {index: input_value} where input_value is RPM, deflection, etc.
            dt (float): Delta time for visual rotation animation
        """
        import numpy as np
        import carb
        from pegasus.simulator.logic.force_generators import (
            SpinningBody,
            LiftingSurface,
        )

        for index, input_value in inputs.items():
            if index not in self.components:
                continue

            component = self.components[index]

            # Type-specific behavior
            if isinstance(component, SpinningBody):
                # Calculate and apply spinning body forces/torques
                force, torque = component.get_force_and_torque(input_value)

                # Apply forces to the rotor's physics rigid body (not the main body)
                rotor_physics_path = f"/body/rotor{index}/rotor_physics"

                if np.any(force):
                    self.apply_force(
                        force.tolist(),
                        pos=[0.0, 0.0, 0.0],  # Apply at center of rotor physics body
                        body_part=rotor_physics_path,
                    )

                if np.any(torque):
                    self.apply_torque(
                        torque.tolist(), body_part=self.relative_body_path
                    )

                # Update visual rotation
                component.update_visual_rotation(input_value, dt)

            elif isinstance(component, LiftingSurface):
                # Calculate and apply lift forces
                force, _ = component.get_force_and_torque(input_value)

                if np.any(force):
                    # TODO: Consider orientation/airspeed for realistic lift
                    self.apply_force(
                        force.tolist(),
                        pos=component.position.tolist(),
                        body_part=self.relative_body_path,
                    )

    def get_components(self) -> dict:
        """
        Get dictionary of all registered components.

        Returns:
            dict: Dictionary of {index: component} pairs
        """
        return self.components.copy()

    def get_component_count(self) -> int:
        """
        Get the number of registered components.

        Returns:
            int: Number of components
        """
        return len(self.components)

    """
    Operations
    """

    def sim_start_stop(self, event):
        """
        Callback that is called every time there is a timeline event such as starting/stoping the simulation.

        Args:
            event: A timeline event generated from Isaac Sim, such as starting or stoping the simulation.
        """

        # If the start/stop button was pressed, then call the start and stop methods accordingly
        if self._world.is_playing() and self._sim_running == False:
            self._sim_running = True

            # Initialize the sensors
            for sensor in self._sensors:
                sensor.start()

            # Initialize the graphical sensors
            for graphical_sensor in self._graphical_sensors:
                graphical_sensor.start()

            # Intializes the communication with the backend. This method is invoked automatically when the simulation starts
            if self._backend:
                self._backend.start()

            # Invoke the start method of the vehicle (if it exists)
            self.start()

        if self._world.is_stopped() and self._sim_running == True:
            self._sim_running = False

            # Reset the DC interface
            self._vehicle_dc_interface = None

            # Stop the sensors
            for sensor in self._sensors:
                sensor.stop()

            # Stop the graphical sensors
            for graphical_sensor in self._graphical_sensors:
                graphical_sensor.stop()

            # Signal the backend that the simulation has stoped. This method is invoked automatically when the simulation stops
            if self._backend:
                self._backend.stop()

            self.stop()

    def apply_force(self, force_frd, pos_frd=[0.0, 0.0, 0.0], body_part="/body"):
        """
        Method that will apply a force on the rigidbody, on the part specified in the 'body_part' at its relative position.
        Forces are provided in FRD body frame and converted to FLU at the interface.

        Args:
            force_frd (array-like): A 3-dimensional vector with the force [Fx, Fy, Fz] in FRD body frame [N].
            pos_frd (list): Position where force is applied in FRD body frame [m]. Defaults to [0.0, 0.0, 0.0].
            body_part (str): The body part to apply force to. Defaults to "/body".
        """

        # Convert force from FRD to FLU for Isaac Sim interface
        # FRD to FLU: X->X, Y->-Y, Z->-Z
        force_flu = [force_frd[0], -force_frd[1], -force_frd[2]]
        pos_flu = [pos_frd[0], -pos_frd[1], -pos_frd[2]]

        # Get the handle of the rigidbody that we will apply the force to
        rb = self.get_dc_interface().get_rigid_body(self._stage_prefix + body_part)

        if rb:
            # Apply the force to the rigidbody (Isaac Sim expects FLU)
            self.get_dc_interface().apply_body_force(
                rb, carb._carb.Float3(force_flu), carb._carb.Float3(pos_flu), False
            )

    def apply_torque(self, torque_frd, body_part="/body"):
        """
        Method that applies a torque vector to the rigidbody.
        Torques are provided in FRD body frame and converted to FLU at the interface.

        Args:
            torque_frd (array-like): A 3-dimensional vector with the torque [Tx, Ty, Tz] in FRD body frame [N⋅m].
            body_part (str): The body part to apply torque to. Defaults to "/body".
        """

        # Convert torque from FRD to FLU for Isaac Sim interface
        # FRD to FLU: X->X, Y->-Y, Z->-Z
        torque_flu = [torque_frd[0], -torque_frd[1], -torque_frd[2]]

        # Get the handle of the rigidbody that we will apply a torque to
        rb = self.get_dc_interface().get_rigid_body(self._stage_prefix + body_part)

        if rb:
            # Apply the torque to the rigidbody (Isaac Sim expects FLU)
            self.get_dc_interface().apply_body_torque(
                rb, carb._carb.Float3(torque_flu), False
            )

    def _main_physics_update(self, dt: float):
        """
        Main monolithic physics callback that handles all physics updates in a clear, sequential order.
        This is the only callback registered with the physics engine for this vehicle.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        # Get current simulation time
        current_time_s = self._world.current_time

        # Step 1: Update the vehicle state from Isaac Sim
        self.update_state(dt)

        # Step 2: Apply control inputs and forces to the vehicle
        self.update(dt)

        # Step 3: Update all sensors with the new state
        for sensor in self._sensors:
            sensor_data = sensor.update(self._state, current_time_s)

            # If sensor has new data and we have a backend, send it
            if sensor_data is not None and self._backend:
                self._backend.update_sensor(sensor.sensor_type, sensor_data)

        # Step 4: Send current state to backend
        if self._backend:
            self._backend.update_state(self._state)
            self._backend.update(dt)

    def update_state(self, dt: float):
        """
        Method that is called at every physics step to retrieve and update the current state of the vehicle, i.e., get
        the current position, orientation, linear and angular velocities and acceleration of the vehicle.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the body frame interface of the vehicle (this will be the frame used to get the position, orientation, etc.)
        body = self.get_dc_interface().get_rigid_body(self.body_path)

        # Get the current position and orientation in the inertial frame
        pose = self.get_dc_interface().get_rigid_body_pose(body)

        # Get the attitude according to the convention [w, x, y, z]
        prim = self._world.stage.GetPrimAtPath(self.body_path)
        rotation_quat = get_world_transform_xform(prim).GetQuaternion()
        rotation_quat_real = rotation_quat.GetReal()
        rotation_quat_img = rotation_quat.GetImaginary()

        # Get the angular velocity of the vehicle expressed in the body frame of reference
        ang_vel = self.get_dc_interface().get_rigid_body_angular_velocity(body)

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        linear_vel = self.get_dc_interface().get_rigid_body_linear_velocity(body)

        # Prepare raw Isaac data
        position_flu_m = np.array(pose.p)
        attitude_flu_quat = np.array(
            [
                rotation_quat_img[0],
                rotation_quat_img[1],
                rotation_quat_img[2],
                rotation_quat_real,
            ]
        )
        linear_velocity_flu_mps = np.array(linear_vel)

        # Convert angular velocity from world to body frame
        angular_velocity_world = np.array(ang_vel)
        angular_velocity_flu_body_rps = (
            Rotation.from_quat(attitude_flu_quat).inv().apply(angular_velocity_world)
        )

        # Update state using the single source of truth method
        self._state.update_from_isaac(
            position_flu_m=position_flu_m,
            attitude_flu_quat=attitude_flu_quat,
            linear_velocity_flu_mps=linear_velocity_flu_mps,
            angular_velocity_flu_rps=angular_velocity_flu_body_rps,
            dt=dt,
        )

    def start(self):
        """
        Method that should be implemented by the class that inherits the vehicle object.
        """
        pass

    def stop(self):
        """
        Method that should be implemented by the class that inherits the vehicle object.
        """
        pass

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in
        simulation based on the motor speed. This method must be implemented
        by a class that inherits this type and it's called periodically by the physics engine.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        pass

    # NOTE: update_sensors is now handled in _main_physics_update
    # Keeping empty method for backward compatibility if needed

    def update_graphical_sensors(self, event):
        """Callback that is called at every rendering steps and will call the graphical_sensor.update method to generate new
        sensor data. For each data that the sensor generates, the backend.update_graphical_sensor method will also be called for
        the backend. For example, if new data is generated for a monocular camera and we have a backend, then the update_graphical_sensor
        method will be called for that backend so that this data can latter be sent through a ROS2 topic.

        Args:
            event (float): The timer event that contains the time elapsed between the previous and current function calls (s).
        """

        # Call the update method for the sensor to update its values internally (if applicable)
        for sensor in self._graphical_sensors:
            sensor_data = sensor.update(self._state, event.payload["dt"])

            # If some data was updated and we have a backend, then just update it
            if sensor_data is not None and self._backend:
                self._backend.update_graphical_sensor(sensor.sensor_type, sensor_data)

    # NOTE: update_sim_state is now handled in _main_physics_update
    # Keeping empty method for backward compatibility if needed

    def get_dc_interface(self):

        if self._vehicle_dc_interface is None:
            self._vehicle_dc_interface = (
                _dynamic_control.acquire_dynamic_control_interface()
            )

        return self._vehicle_dc_interface
