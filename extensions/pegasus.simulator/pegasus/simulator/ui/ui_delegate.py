"""
| File: ui_delegate.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Definition of the UiDelegate which is an abstraction layer betweeen the extension UI and code logic features
"""

# External packages
import os
import asyncio
import tempfile
from scipy.spatial.transform import Rotation

# Omniverse extensions
import carb
import omni.ui as ui
import omni.usd
import omni.kit.window.file_exporter
from pxr import Usd, Sdf

# Extension Configurations
from pegasus.simulator.params import VEHICLES, SIMULATION_ENVIRONMENTS, WORLD_SETTINGS, ASSET_PATH
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Vehicle Manager to spawn Vehicles
from pegasus.simulator.logic import PX4Backend, PX4BackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor
from pegasus.simulator.logic.vehicle_manager import VehicleManager


class UIDelegate:
    """
    Object that will interface between the logic/dynamic simulation part of the extension and the Widget UI
    """

    def __init__(self):

        # The window that will be bound to this delegate
        self._window = None

        # Flag to prevent duplicate cleanup calls
        self._cleaned_up = False

        # Get an instance of the pegasus simulator
        self._pegasus_sim: PegasusInterface = PegasusInterface()

        # Attribute that holds the currently selected scene from the dropdown menu
        self._scene_dropdown: ui.AbstractItemModel = None
        self._scene_names = list(SIMULATION_ENVIRONMENTS.keys())

        # Selected latitude, longitude and altitude
        self._latitude_field: ui.AbstractValueModel = None
        self._latitude = PegasusInterface().latitude_deg
        self._longitude_field: ui.AbstractValueModel = None
        self._longitude = PegasusInterface().longitude_deg
        self._altitude_field: ui.AbstractValueModel = None
        self._altitude = PegasusInterface().altitude_msl_m

        # Attribute that hold the currently selected vehicle from the dropdown menu
        self._vehicle_dropdown: ui.AbstractItemModel = None
        self._vehicles_names = list(VEHICLES.keys())

        # Get an instance of the vehicle manager
        self._vehicle_manager = VehicleManager()

        # Selected value for the the id of the vehicle
        self._vehicle_id_field: ui.AbstractValueModel = None
        self._vehicle_id: int = 0

        # Attribute that will save the model for the px4-autostart checkbox
        self._px4_autostart_checkbox: ui.AbstractValueModel = None
        self._autostart_px4: bool = True

        # Atributes to store the path for the Px4 directory
        self._px4_directory_field: ui.AbstractValueModel = None
        self._px4_dir: str = PegasusInterface().px4_path

        # Atributes to store the PX4 airframe
        self._px4_airframe_field: ui.AbstractValueModel = None
        self._px4_airframe: str = self._pegasus_sim.px4_default_airframe

    def set_window_bind(self, window):
        self._window = window

    def set_scene_dropdown(self, scene_dropdown_model: ui.AbstractItemModel):
        self._scene_dropdown = scene_dropdown_model
    
    def set_latitude_field(self, latitude_model: ui.AbstractValueModel):
        self._latitude_field = latitude_model
    
    def set_longitude_field(self, longitude_model: ui.AbstractValueModel):
        self._longitude_field = longitude_model

    def set_altitude_field(self, altitude_model: ui.AbstractValueModel):
        self._altitude_field = altitude_model

    def set_vehicle_dropdown(self, vehicle_dropdown_model: ui.AbstractItemModel):
        self._vehicle_dropdown = vehicle_dropdown_model

    def set_vehicle_id_field(self, vehicle_id_field: ui.AbstractValueModel):
        self._vehicle_id_field = vehicle_id_field

    def set_px4_autostart_checkbox(self, checkbox_model:ui.AbstractValueModel):
        self._px4_autostart_checkbox = checkbox_model

    def set_px4_directory_field(self, directory_field_model: ui.AbstractValueModel):
        self._px4_directory_field = directory_field_model

    def set_px4_airframe_field(self, airframe_field_model: ui.AbstractValueModel):
        self._px4_airframe_field = airframe_field_model

    """
    ---------------------------------------------------------------------
    Callbacks to handle user interaction with the extension widget window
    ---------------------------------------------------------------------
    """

    def on_load_environment(self):
        """
        Method that should be invoked when the button to load the selected environment is pressed
        """

        # Check if an environment is selected in the drop-down menu
        if self._scene_dropdown is not None:

            # Get the id of the selected environment from the list
            environment_index = self._scene_dropdown.get_item_value_model().as_int

            # Get the name of the selected world
            selected_world = self._scene_names[environment_index]

            # Try to spawn the selected world
            self._pegasus_sim.set_world_settings(**WORLD_SETTINGS['px4'])
            asyncio.ensure_future(self._pegasus_sim.load_environment_async(SIMULATION_ENVIRONMENTS[selected_world], force_clear=True))

    def on_set_new_global_coordinates(self):
        """
        Method that gets invoked to set new global coordinates for this simulation
        """
        self._pegasus_sim.set_global_coordinates(
            self._latitude_field.get_value_as_float(),
            self._longitude_field.get_value_as_float(),
            self._altitude_field.get_value_as_float())
        
    def on_reset_global_coordinates(self):
        """
        Method that gets invoked to set the global coordinates to the defaults saved in the extension configuration file
        """
        self._pegasus_sim.set_default_global_coordinates()

        self._latitude_field.set_value(self._pegasus_sim.latitude_deg)
        self._longitude_field.set_value(self._pegasus_sim.longitude_deg)
        self._altitude_field.set_value(self._pegasus_sim.altitude_msl_m)

    def on_set_new_default_global_coordinates(self):
        """
        Method that gets invoked to set new defualt global coordinates for this simulation. This will attempt
        to save the current coordinates as new defaults for the extension itself
        """
        self._pegasus_sim.set_new_default_global_coordinates(
            self._latitude_field.get_value_as_float(),
            self._longitude_field.get_value_as_float(),
            self._altitude_field.get_value_as_float()
        )

    def on_clear_scene(self):
        """
        Method that should be invoked when the clear world button is pressed
        """
        self._pegasus_sim.clear_scene()

    def on_load_vehicle(self):
        """
        Method that should be invoked when the button to load the selected vehicle is pressed
        """

        async def async_load_vehicle():
            # Check if we already have a physics environment activated. If not, then activate it
            # and only after spawn the vehicle. This is to avoid trying to spawn a vehicle without a physics
            # environment setup. This way we can even spawn a vehicle in an empty world and it won't care
            if hasattr(self._pegasus_sim.world, "_physics_context") == False:
                await self._pegasus_sim.world.initialize_simulation_context_async()

            # Check if a vehicle is selected in the drop-down menu
            if self._vehicle_dropdown is not None and self._window is not None:

                # Get the id of the selected vehicle from the list
                vehicle_index = self._vehicle_dropdown.get_item_value_model().as_int

                # Get the name of the selected vehicle
                selected_vehicle = self._vehicles_names[vehicle_index]

                # Get the id of the selected vehicle
                self._vehicle_id = self._vehicle_id_field.get_value_as_int()

                # Get the desired position and orientation of the vehicle from the UI transform
                pos, euler_angles = self._window.get_selected_vehicle_attitude()

                # The Multirotor class now handles PX4 backend configuration internally from YAML

                # Clean up any existing vehicle before loading a new one (BUILD: v2)
                vehicle_path = "/World/quadrotor"
                carb.log_info(f"YAML Vehicle Loading - Build v2 - Cleaning up {vehicle_path}")

                # Check if vehicle exists in VehicleManager and clean it up
                existing_vehicle = self._vehicle_manager.get_vehicle(vehicle_path)
                if existing_vehicle:
                    carb.log_info(f"Removing existing vehicle from VehicleManager: {vehicle_path}")
                    # Remove from Isaac Sim scene first
                    try:
                        self._pegasus_sim.world.scene.remove_object(existing_vehicle)
                    except:
                        pass  # Object might not be in scene
                    # Remove from VehicleManager
                    self._vehicle_manager.remove_vehicle(vehicle_path)

                # Check if prim exists in stage and remove it
                stage = omni.usd.get_context().get_stage()
                if stage:
                    existing_prim = stage.GetPrimAtPath(vehicle_path)
                    if existing_prim and existing_prim.IsValid():
                        carb.log_info(f"Removing existing prim from stage: {vehicle_path}")
                        stage.RemovePrim(vehicle_path)

                # Try to spawn the selected vehicle in the world to the specified namespace
                # VEHICLES[selected_vehicle] now contains path to YAML config file
                Multirotor(
                    stage_prefix="/World/quadrotor",
                    config_file=VEHICLES[selected_vehicle],
                    vehicle_id=self._vehicle_id,
                    init_pos=pos,
                    init_orientation=Rotation.from_euler("XYZ", euler_angles, degrees=True).as_quat(),
                )

            # Log that a vehicle of the type multirotor was spawned in the world via the extension UI
                carb.log_info("Spawned the vehicle: " + selected_vehicle + " using the Pegasus Simulator UI")
            else:
                # Log that it was not possible to spawn the vehicle in the world using the Pegasus Simulator UI
                carb.log_error("Could not spawn the vehicle using the Pegasus Simulator UI")

        # Run the actual vehicle spawn async so that the UI does not freeze
        asyncio.ensure_future(async_load_vehicle())        

    def on_set_viewport_camera(self):
        """
        Method that should be invoked when the button to set the viewport camera pose is pressed
        """
        carb.log_warn("The viewport camera pose has been adjusted")

        if self._window:

            # Get the current camera position value
            camera_position, camera_target = self._window.get_selected_camera_pos()

            if camera_position is not None and camera_target is not None:

                # Set the camera view to a fixed value
                self._pegasus_sim.set_viewport_camera(eye=camera_position, target=camera_target)
    
    def on_set_new_default_px4_path(self):
        """
        Method that will try to update the new PX4 autopilot path with whatever is passed on the string field
        """
        carb.log_warn("A new default PX4 Path will be set for the extension.")

        # Read the current path from the field
        path = self._px4_directory_field.get_value_as_string()

        # Set the path using the pegasus interface
        self._pegasus_sim.set_px4_path(path)

    def on_reset_px4_path(self):
        """
        Method that will reset the string field to the default PX4 path
        """
        carb.log_warn("Reseting the path to the default one")
        self._px4_directory_field.set_value(self._pegasus_sim.px4_path)

    def on_save_environment(self):
        """
        Method to save the current environment (everything except vehicle at /World/quadrotor)
        """
        # Get the current stage
        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_error("No stage available to save")
            return

        # Get save path from user
        file_exporter = omni.kit.window.file_exporter.get_file_exporter()

        def save_environment_callback(filename: str, dirname: str, extension: str = "", selections: list = []) -> None:
            if not filename:
                return

            filepath = os.path.join(dirname, filename)
            if not filepath.endswith('.usd'):
                filepath += '.usd'

            try:
                # Solution B1: Use stage.Export() method for proper USD handling

                # Create a temporary file for exporting the flattened stage
                with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as temp_file:
                    temp_filepath = temp_file.name

                carb.log_info(f"Creating temporary flattened stage at: {temp_filepath}")

                # Export the entire flattened stage to temp file
                stage.Export(temp_filepath, addSourceFileComment=False)

                # Open the temp file as a new stage
                temp_stage = Usd.Stage.Open(temp_filepath)
                if not temp_stage:
                    raise Exception("Failed to open temporary flattened stage")

                # Create a new stage for export
                export_stage = Usd.Stage.CreateNew(filepath)

                # Copy all prims except the vehicle at /World/quadrotor
                VEHICLE_PATH = "/World/quadrotor"

                for prim in temp_stage.Traverse():
                    prim_path = str(prim.GetPath())

                    # Skip vehicle and its children
                    if prim_path.startswith(VEHICLE_PATH):
                        continue

                    # Create the prim in export stage
                    export_prim = export_stage.DefinePrim(prim.GetPath(), prim.GetTypeName())

                    # Copy all attributes (which are now flattened/composed)
                    for attr in prim.GetAttributes():
                        if attr.HasAuthoredValue():
                            export_attr = export_prim.CreateAttribute(attr.GetName(), attr.GetTypeName())
                            export_attr.Set(attr.Get())

                    # Copy relationships
                    for rel in prim.GetRelationships():
                        if rel.HasAuthoredTargets():
                            export_rel = export_prim.CreateRelationship(rel.GetName())
                            export_rel.SetTargets(rel.GetTargets())

                export_stage.Save()
                carb.log_info(f"Environment saved to: {filepath}")

                # Clean up temporary file
                try:
                    os.unlink(temp_filepath)
                    carb.log_info(f"Cleaned up temporary file: {temp_filepath}")
                except Exception as cleanup_error:
                    carb.log_warn(f"Failed to clean up temporary file {temp_filepath}: {cleanup_error}")

            except Exception as e:
                carb.log_error(f"Failed to save environment: {str(e)}")

        file_exporter.show_window(
            title="Save Environment",
            export_button_label="Save",
            export_handler=save_environment_callback,
            filename_url=os.path.join(ASSET_PATH, "Worlds", "environment.usd")
        )

    def on_save_vehicle(self):
        """
        Method to save only the vehicle at /World/quadrotor
        """
        # Get the current stage
        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_error("No stage available to save")
            return

        # Check if vehicle exists
        VEHICLE_PATH = "/World/quadrotor"
        vehicle_prim = stage.GetPrimAtPath(VEHICLE_PATH)

        if not vehicle_prim or not vehicle_prim.IsValid():
            carb.log_error(f"No vehicle found at {VEHICLE_PATH}")
            return

        # Get save path from user
        file_exporter = omni.kit.window.file_exporter.get_file_exporter()

        def save_vehicle_callback(filename: str, dirname: str, extension: str = "", selections: list = []) -> None:
            if not filename:
                return

            filepath = os.path.join(dirname, filename)
            if not filepath.endswith('.usd'):
                filepath += '.usd'

            try:
                # Solution B1: Use stage.Export() method for proper USD handling

                # Create a temporary file for exporting the flattened stage
                with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as temp_file:
                    temp_filepath = temp_file.name

                carb.log_info(f"Creating temporary flattened stage at: {temp_filepath}")

                # Export the entire flattened stage to temp file
                stage.Export(temp_filepath, addSourceFileComment=False)

                # Open the temp file as a new stage
                temp_stage = Usd.Stage.Open(temp_filepath)
                if not temp_stage:
                    raise Exception("Failed to open temporary flattened stage")

                # Create a new stage for the final export
                export_stage = Usd.Stage.CreateNew(filepath)

                # Get the vehicle prim from the flattened temp stage
                flattened_vehicle = temp_stage.GetPrimAtPath(VEHICLE_PATH)

                if flattened_vehicle and flattened_vehicle.IsValid():
                    carb.log_info(f"Copying vehicle subtree from {VEHICLE_PATH}")
                    # Copy the entire flattened vehicle subtree
                    for prim in Usd.PrimRange(flattened_vehicle):
                        prim_path = prim.GetPath()

                        # Create the prim in export stage
                        export_prim = export_stage.DefinePrim(prim_path, prim.GetTypeName())

                        # Copy all attributes (which are now flattened/composed)
                        for attr in prim.GetAttributes():
                            if attr.HasAuthoredValue():
                                export_attr = export_prim.CreateAttribute(attr.GetName(), attr.GetTypeName())
                                export_attr.Set(attr.Get())

                        # Copy relationships
                        for rel in prim.GetRelationships():
                            if rel.HasAuthoredTargets():
                                export_rel = export_prim.CreateRelationship(rel.GetName())
                                export_rel.SetTargets(rel.GetTargets())
                else:
                    raise Exception(f"Vehicle prim not found at {VEHICLE_PATH} in flattened stage")

                export_stage.Save()
                carb.log_info(f"Vehicle saved to: {filepath}")

                # Clean up temporary file
                try:
                    os.unlink(temp_filepath)
                    carb.log_info(f"Cleaned up temporary file: {temp_filepath}")
                except Exception as cleanup_error:
                    carb.log_warn(f"Failed to clean up temporary file {temp_filepath}: {cleanup_error}")

            except Exception as e:
                carb.log_error(f"Failed to save vehicle: {str(e)}")

        file_exporter.show_window(
            title="Save Vehicle",
            export_button_label="Save",
            export_handler=save_vehicle_callback,
            filename_url=os.path.join(ASSET_PATH, "Robots", "vehicle.usd")
        )

    # ========================================
    # Gimbal Control Methods
    # ========================================

    def on_gimbal_enabled_changed(self, enabled: bool):
        """
        Called when gimbal control is enabled/disabled
        """
        carb.log_info(f"Gimbal control {'enabled' if enabled else 'disabled'}")
        # TODO: Enable/disable gimbal control in the vehicle
        self._gimbal_enabled = enabled

    def on_gimbal_pitch_changed(self, pitch: float):
        """
        Called when gimbal pitch slider changes
        """
        if hasattr(self, '_gimbal_enabled') and self._gimbal_enabled:
            carb.log_info(f"Gimbal pitch changed to: {pitch}°")
            self._send_gimbal_command(pitch=pitch)

    def on_gimbal_roll_changed(self, roll: float):
        """
        Called when gimbal roll slider changes
        """
        if hasattr(self, '_gimbal_enabled') and self._gimbal_enabled:
            carb.log_info(f"Gimbal roll changed to: {roll}°")
            self._send_gimbal_command(roll=roll)

    def on_gimbal_yaw_changed(self, yaw: float):
        """
        Called when gimbal yaw slider changes
        """
        if hasattr(self, '_gimbal_enabled') and self._gimbal_enabled:
            carb.log_info(f"Gimbal yaw changed to: {yaw}°")
            self._send_gimbal_command(yaw=yaw)

    def on_gimbal_preset_clicked(self, pitch: float, roll: float, yaw: float):
        """
        Called when a gimbal preset button is clicked
        """
        if hasattr(self, '_gimbal_enabled') and self._gimbal_enabled:
            carb.log_info(f"Gimbal preset: pitch={pitch}°, roll={roll}°, yaw={yaw}°")
            self._send_gimbal_command(pitch=pitch, roll=roll, yaw=yaw)

    def on_gimbal_stabilize_clicked(self):
        """
        Called when stabilize mode button is clicked
        """
        carb.log_info("Gimbal stabilize mode activated")
        # TODO: Enable gimbal stabilization
        self._gimbal_mode = "stabilize"

    def on_gimbal_manual_clicked(self):
        """
        Called when manual mode button is clicked
        """
        carb.log_info("Gimbal manual mode activated")
        # TODO: Disable gimbal stabilization
        self._gimbal_mode = "manual"

    def _send_gimbal_command(self, pitch=None, roll=None, yaw=None):
        """
        Send gimbal command to the active vehicle via MAVLink
        """
        try:
            # Get the current active vehicle
            vehicle_manager = VehicleManager.get_vehicle_manager()
            vehicles = vehicle_manager.vehicles

            if not vehicles:
                carb.log_warn("No vehicles available for gimbal control")
                return

            # Get the first vehicle (or implement vehicle selection)
            vehicle_id = list(vehicles.keys())[0]
            vehicle = vehicles[vehicle_id]

            # Find gimbal system in vehicle's graphical sensors
            gimbal_system = None
            for sensor in vehicle._graphical_sensors:
                if hasattr(sensor, 'set_angles'):  # Check if it's a gimbal system
                    gimbal_system = sensor
                    break

            if gimbal_system is None:
                carb.log_warn("No gimbal system found in vehicle")
                return

            # Get current angles if not all specified
            current_angles = gimbal_system._gimbal_angles
            new_pitch = pitch if pitch is not None else current_angles.get('pitch', 0)
            new_roll = roll if roll is not None else current_angles.get('roll', 0)
            new_yaw = yaw if yaw is not None else current_angles.get('yaw', 0)

            # Send command to gimbal system
            gimbal_system.set_angles(new_pitch, new_roll, new_yaw)

            # Also send MAVLink command if vehicle has PX4 backend
            self._send_mavlink_gimbal_command(new_pitch, new_roll, new_yaw, vehicle)

        except Exception as e:
            carb.log_error(f"Failed to send gimbal command: {str(e)}")

    def _send_mavlink_gimbal_command(self, pitch: float, roll: float, yaw: float, vehicle):
        """
        Send MAVLink gimbal command to PX4 backend
        """
        try:
            # Find PX4 backend
            px4_backend = None
            if vehicle._backend and isinstance(vehicle._backend, PX4Backend):
                px4_backend = vehicle._backend

            if px4_backend is None:
                carb.log_warn("No PX4 MAVLink backend found for gimbal control")
                return

            # TODO: Implement MAVLink gimbal command sending
            # This would involve creating a gimbal_device_set_attitude message
            # and sending it through the MAVLink connection

            carb.log_info(f"Sending MAVLink gimbal command: pitch={pitch}°, roll={roll}°, yaw={yaw}°")

            # For now, just log the command
            # In a full implementation, this would create and send a MAVLink message:
            # - MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
            # - or GIMBAL_DEVICE_SET_ATTITUDE message

        except Exception as e:
            carb.log_error(f"Failed to send MAVLink gimbal command: {str(e)}")

    def on_manual_motor_enabled_changed(self, enabled: bool):
        """
        Handle manual motor control enable/disable checkbox change.

        Args:
            enabled (bool): True if manual control is enabled
        """
        try:
            # Get the current active vehicle
            vehicle_manager = VehicleManager.get_vehicle_manager()
            vehicles = vehicle_manager.vehicles

            if not vehicles:
                carb.log_warn("No vehicles loaded for motor control")
                return

            # Get the first vehicle (assuming single vehicle for now)
            vehicle = next(iter(vehicles.values()))

            if enabled:
                vehicle.enable_manual_control()
                carb.log_info("Manual motor control enabled")
            else:
                vehicle.disable_manual_control()
                carb.log_info("Manual motor control disabled")

        except Exception as e:
            carb.log_error(f"Failed to toggle manual motor control: {str(e)}")

    def on_motor_speed_changed(self, motor_index: int, speed: float):
        """
        Handle individual motor speed change.

        Args:
            motor_index (int): Motor index (0-3)
            speed (float): Motor speed percentage (0-100)
        """
        try:
            # Get the current active vehicle
            vehicle_manager = VehicleManager.get_vehicle_manager()
            vehicles = vehicle_manager.vehicles

            if not vehicles:
                return

            # Get the first vehicle (assuming single vehicle for now)
            vehicle = next(iter(vehicles.values()))

            if vehicle.is_manual_control_enabled():
                # Get current motor speeds and update the specified motor
                current_speeds = vehicle._manual_motor_speeds

                # Convert rad/s back to percentage to get current values
                current_percentages = [(s / 1000.0) * 100.0 for s in current_speeds]

                # Update the specific motor
                if 0 <= motor_index < 4:
                    current_percentages[motor_index] = speed
                    vehicle.set_manual_motor_speeds(current_percentages)

        except Exception as e:
            carb.log_error(f"Failed to set motor {motor_index} speed: {str(e)}")

    def on_master_throttle_changed(self, throttle: float):
        """
        Handle master throttle change (sets all motors to same value).

        Args:
            throttle (float): Throttle percentage (0-100)
        """
        try:
            # Get the current active vehicle
            vehicle_manager = VehicleManager.get_vehicle_manager()
            vehicles = vehicle_manager.vehicles

            if not vehicles:
                return

            # Get the first vehicle (assuming single vehicle for now)
            vehicle = next(iter(vehicles.values()))

            if vehicle.is_manual_control_enabled():
                # Set all motors to the same throttle value
                motor_speeds = [throttle, throttle, throttle, throttle]
                vehicle.set_manual_motor_speeds(motor_speeds)

                # Update UI sliders to reflect the master throttle
                if self._window:
                    for i in range(4):
                        if len(self._window._motor_sliders) > i:
                            self._window._motor_sliders[i].model.set_value(throttle)
                            self._window._motor_values[i].model.set_value(throttle)

        except Exception as e:
            carb.log_error(f"Failed to set master throttle: {str(e)}")

    def cleanup(self):
        """
        Clean up all references to enable proper extension reload.
        """
        if self._cleaned_up:
            carb.log_info("UIDelegate cleanup already completed, skipping")
            return

        carb.log_info("UIDelegate cleanup started")
        self._cleaned_up = True

        # Clear window reference to break circular reference
        self._window = None

        # Clear all UI model references
        self._scene_dropdown = None
        self._latitude_field = None
        self._longitude_field = None
        self._altitude_field = None
        self._vehicle_dropdown = None
        self._vehicle_id_field = None
        self._px4_autostart_checkbox = None
        self._px4_directory_field = None
        self._px4_airframe_field = None

        # Clear scene and vehicle name lists
        self._scene_names = None
        self._vehicles_names = None

        # Clear manager and interface references
        self._vehicle_manager = None

        # Clear PegasusInterface reference and reset singleton
        if self._pegasus_sim:
            # Clear our reference
            self._pegasus_sim = None

            # Reset the singleton instance to ensure clean reload
            try:
                from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
                PegasusInterface.reset_singleton()
            except Exception as e:
                carb.log_warn(f"Could not reset PegasusInterface singleton: {str(e)}")

        carb.log_info("UIDelegate cleanup completed")
