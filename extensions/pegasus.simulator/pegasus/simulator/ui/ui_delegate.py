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
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, WORLD_SETTINGS, ASSET_PATH
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Vehicle Manager to spawn Vehicles
from pegasus.simulator.logic.backends import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.vehicle_manager import VehicleManager
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera


class UIDelegate:
    """
    Object that will interface between the logic/dynamic simulation part of the extension and the Widget UI
    """

    def __init__(self):

        # The window that will be bound to this delegate
        self._window = None

        # Get an instance of the pegasus simulator
        self._pegasus_sim: PegasusInterface = PegasusInterface()

        # Attribute that holds the currently selected scene from the dropdown menu
        self._scene_dropdown: ui.AbstractItemModel = None
        self._scene_names = list(SIMULATION_ENVIRONMENTS.keys())

        # Selected latitude, longitude and altitude
        self._latitude_field: ui.AbstractValueModel = None
        self._latitude = PegasusInterface().latitude
        self._longitude_field: ui.AbstractValueModel = None
        self._longitude = PegasusInterface().longitude
        self._altitude_field: ui.AbstractValueModel = None
        self._altitude = PegasusInterface().altitude

        # Attribute that hold the currently selected vehicle from the dropdown menu
        self._vehicle_dropdown: ui.AbstractItemModel = None
        self._vehicles_names = list(ROBOTS.keys())

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

        self._latitude_field.set_value(self._pegasus_sim.latitude)
        self._longitude_field.set_value(self._pegasus_sim.longitude)
        self._altitude_field.set_value(self._pegasus_sim.altitude)

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
                selected_robot = self._vehicles_names[vehicle_index]

                # Get the id of the selected vehicle
                self._vehicle_id = self._vehicle_id_field.get_value_as_int()

                # Get the desired position and orientation of the vehicle from the UI transform
                pos, euler_angles = self._window.get_selected_vehicle_attitude()

                # Create PX4 backend (only backend supported)
                # Read if we should auto-start px4 from the checkbox
                px4_autostart = self._px4_autostart_checkbox.get_value_as_bool()

                # Read the PX4 path from the field
                px4_path = os.path.expanduser(self._px4_directory_field.get_value_as_string())

                # Read the PX4 airframe from the field
                px4_airframe = self._px4_airframe_field.get_value_as_string()

                backend_config = PX4MavlinkBackendConfig({
                    "vehicle_id": self._vehicle_id,
                    "px4_autolaunch": px4_autostart,
                    "px4_dir": px4_path,
                    "px4_vehicle_model": px4_airframe
                })
                backend = PX4MavlinkBackend(config=backend_config)
                   
                # Create the multirotor configuration
                config_multirotor = MultirotorConfig()
                config_multirotor.backends = [backend]
                config_multirotor.graphical_sensors = [MonocularCamera("camera", config={"update_rate": 60.0})]

                # Clean up any existing vehicle before loading a new one
                vehicle_path = "/World/quadrotor"

                # Clear all callbacks for the vehicle path to prevent conflicts
                callback_paths = [
                    f"{vehicle_path}/state",
                    f"{vehicle_path}/update",
                    f"{vehicle_path}/start_stop_sim",
                    f"{vehicle_path}/Sensors",
                    f"{vehicle_path}/GraphicalSensors",
                    f"{vehicle_path}/mav_state"
                ]

                carb.log_info(f"Clearing callbacks for {vehicle_path}")
                for callback_path in callback_paths:
                    try:
                        self._pegasus_sim.world.remove_physics_callback(callback_path)
                    except Exception as e:
                        carb.log_info(f"Physics callback {callback_path} not found or already removed: {e}")

                    try:
                        self._pegasus_sim.world.remove_render_callback(callback_path)
                    except Exception as e:
                        carb.log_info(f"Render callback {callback_path} not found or already removed: {e}")

                    try:
                        self._pegasus_sim.world.remove_timeline_callback(callback_path)
                    except Exception as e:
                        carb.log_info(f"Timeline callback {callback_path} not found or already removed: {e}")

                # Check if vehicle exists in VehicleManager and clean it up
                existing_vehicle = self._vehicle_manager.get_vehicle(vehicle_path)
                if existing_vehicle:
                    carb.log_info(f"Removing existing vehicle at {vehicle_path}")
                    # Remove from Isaac Sim scene first
                    self._pegasus_sim.world.scene.remove_object(existing_vehicle)
                    # Remove from VehicleManager
                    self._vehicle_manager.remove_vehicle(vehicle_path)
                    # Delete the vehicle object
                    del existing_vehicle

                # Check if prim exists in stage and remove it
                stage = omni.usd.get_context().get_stage()
                if stage:
                    existing_prim = stage.GetPrimAtPath(vehicle_path)
                    if existing_prim and existing_prim.IsValid():
                        carb.log_info(f"Removing existing prim at {vehicle_path}")
                        stage.RemovePrim(vehicle_path)

                # Try to spawn the selected robot in the world to the specified namespace
                Multirotor(
                    "/World/quadrotor",
                    ROBOTS[selected_robot],
                    self._vehicle_id,
                    pos,
                    Rotation.from_euler("XYZ", euler_angles, degrees=True).as_quat(),
                    config=config_multirotor,
                )

            # Log that a vehicle of the type multirotor was spawned in the world via the extension UI
                carb.log_info("Spawned the robot: " + selected_robot + " using the Pegasus Simulator UI")
            else:
                # Log that it was not possible to spawn the vehicle in the world using the Pegasus Simulator UI
                carb.log_error("Could not spawn the robot using the Pegasus Simulator UI")

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
