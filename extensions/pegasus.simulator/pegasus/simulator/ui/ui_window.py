"""
| File: ui_window.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Definition of WidgetWindow which contains all the UI code that defines the extension GUI
"""

__all__ = ["WidgetWindow"]

import numpy as np

import carb
import omni.ui as ui
from omni.ui import color as cl

from pegasus.simulator.ui.ui_delegate import UIDelegate
from pegasus.simulator.params import (
    VEHICLES,
    SIMULATION_ENVIRONMENTS,
    THUMBNAIL,
    WORLD_THUMBNAIL,
    WINDOW_TITLE,
)


class WidgetWindow(ui.Window):
    LABEL_PADDING = 120
    BUTTON_HEIGHT = 50
    GENERAL_SPACING = 5

    WINDOW_WIDTH = 325
    WINDOW_HEIGHT = 850

    BUTTON_SELECTED_STYLE = {
        "Button": {
            "background_color": cl("#3780ae"),
            "border_color": cl("#29587c"),
            "border_width": 2,
            "border_radius": 5,
            "padding": 2,
        }
    }

    BUTTON_BASE_STYLE = {
        "Button": {
            "background_color": cl("#292929"),
            "border_color": cl("#292929"),
            "border_width": 2,
            "border_radius": 5,
            "padding": 5,
        }
    }

    def __init__(self, delegate: UIDelegate, **kwargs):
        """
        Constructor for the Window UI widget of the extension. Receives as input a UIDelegate that implements
        all the callbacks to handle button clicks, drop-down menu actions, etc. (abstracting the interface between
        the logic of the code and the ui)
        """

        # Setup the base widget window
        super().__init__(
            WINDOW_TITLE,
            width=WidgetWindow.WINDOW_WIDTH,
            height=WidgetWindow.WINDOW_HEIGHT,
            visible=True,
            **kwargs,
        )
        self.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

        # Setup the delegate that will bridge between the logic and the UI
        self._delegate = delegate

        # Bind the UI delegate to this window
        self._delegate.set_window_bind(self)

        # Auxiliar attributes for getting the transforms of the vehicle and the camera from the UI
        self._camera_transform_models = []
        self._vehicle_transform_models = []

        # Track UI callbacks and elements for cleanup
        self._ui_subscriptions = []
        self._gimbal_buttons = []
        self._gimbal_enabled_checkbox = None
        self._gimbal_pitch_slider = None
        self._gimbal_pitch_value = None
        self._gimbal_roll_slider = None
        self._gimbal_roll_value = None
        self._gimbal_yaw_slider = None
        self._gimbal_yaw_value = None

        # Motor control UI elements
        self._motor_control_enabled_checkbox = None
        self._motor_sliders = []
        self._motor_values = []
        self._master_throttle_slider = None
        self._master_throttle_value = None

        # Build the actual window UI
        self._build_window()

    def destroy(self):

        # Only destroy the UI window - do NOT clear the scene during shutdown
        # (on_clear_scene() should only be called when user explicitly clicks "Clear Scene")

        carb.log_info("WidgetWindow cleanup started")

        # Clean up UI subscriptions - they auto-unsubscribe when destroyed
        carb.log_info(f"Cleaning up {len(self._ui_subscriptions)} UI subscriptions")
        self._ui_subscriptions.clear()
        carb.log_info("UI subscriptions cleared - auto-unsubscribed")

        # Clean up model references
        self._camera_transform_models.clear()
        self._vehicle_transform_models.clear()

        # Clear gimbal UI element references to break potential circular references
        self._gimbal_buttons.clear()
        self._gimbal_enabled_checkbox = None
        self._gimbal_pitch_slider = None
        self._gimbal_pitch_value = None
        self._gimbal_roll_slider = None
        self._gimbal_roll_value = None
        self._gimbal_yaw_slider = None
        self._gimbal_yaw_value = None

        # Clean up delegate reference and call its cleanup
        if self._delegate:
            try:
                carb.log_info("Calling UIDelegate cleanup")
                self._delegate.cleanup()
                carb.log_info("UIDelegate cleanup completed successfully")
            except Exception as e:
                carb.log_error(f"Failed to cleanup UIDelegate: {str(e)}")
            self._delegate = None
        else:
            carb.log_info("UIDelegate already None, skipping cleanup")

        carb.log_info("WidgetWindow cleanup completed")

        # It will destroy all the children
        super().destroy()

    def _build_window(self):

        # Define the UI of the widget window
        with self.frame:

            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):

                # Vertical Stack of menus
                with ui.VStack():
                    # Add build number for debugging
                    with ui.HStack(height=20):
                        ui.Spacer()
                        ui.Label(
                            "Build: 2025.09.23-v3",
                            style={"color": 0x808080FF, "font_size": 12},
                        )
                        ui.Spacer()
                    ui.Spacer(height=5)

                    # Create a frame for configuring PX4 settings
                    self._px4_configuration_frame()
                    ui.Spacer(height=5)

                    # Create a frame for selecting which scene to load
                    self._scene_selection_frame()
                    ui.Spacer(height=5)

                    # Create a frame for selecting which vehicle to load in the simulation environment
                    self._robot_selection_frame()
                    ui.Spacer(height=5)

                    # Create a frame for selecting the camera position, and what it should point torwards to
                    self._viewport_camera_frame()
                    ui.Spacer(height=5)

                    # Create a frame for manual gimbal control
                    self._gimbal_control_frame()
                    ui.Spacer(height=5)

                    # Create a frame for manual motor control
                    self._motor_control_frame()
                    ui.Spacer()

    def _scene_selection_frame(self):
        """
        Method that implements a dropdown menu with the list of available simulation environemts for the vehicle
        """

        # Frame for selecting the simulation environment to load
        with ui.CollapsableFrame("Environment Selection"):
            with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                ui.Spacer(height=WidgetWindow.GENERAL_SPACING)

                # Iterate over all existing pre-made worlds bundled with this extension
                with ui.HStack():
                    ui.Label(
                        "World Assets", width=WidgetWindow.LABEL_PADDING, height=10.0
                    )

                    # Combo box with the available environments to select from
                    dropdown_menu = ui.ComboBox(0, height=10, name="environments")
                    for environment in SIMULATION_ENVIRONMENTS:
                        dropdown_menu.model.append_child_item(
                            None, ui.SimpleStringModel(environment)
                        )

                    # Allow the delegate to know which option was selected in the dropdown menu
                    self._delegate.set_scene_dropdown(dropdown_menu.model)

                ui.Spacer(height=0)

                # UI to configure the default latitude, longitude and altitude coordinates
                with ui.CollapsableFrame("Geographic Coordinates", collapsed=False):
                    with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                        with ui.HStack():

                            # Latitude
                            ui.Label(
                                "Latitude",
                                name="label",
                                width=WidgetWindow.LABEL_PADDING - 50,
                            )
                            latitude_field = ui.FloatField(name="latitude", precision=6)
                            latitude_field.model.set_value(self._delegate._latitude)
                            self._delegate.set_latitude_field(latitude_field.model)
                            ui.Circle(
                                name="transform",
                                width=20,
                                height=20,
                                radius=3.5,
                                size_policy=ui.CircleSizePolicy.FIXED,
                            )

                            # Longitude
                            ui.Label(
                                "Longitude",
                                name="label",
                                width=WidgetWindow.LABEL_PADDING - 50,
                            )
                            longitude_field = ui.FloatField(
                                name="longitude", precision=6
                            )
                            longitude_field.model.set_value(self._delegate._longitude)
                            self._delegate.set_longitude_field(longitude_field.model)
                            ui.Circle(
                                name="transform",
                                width=20,
                                height=20,
                                radius=3.5,
                                size_policy=ui.CircleSizePolicy.FIXED,
                            )

                            # Altitude
                            ui.Label(
                                "Altitude",
                                name="label",
                                width=WidgetWindow.LABEL_PADDING - 50,
                            )
                            altitude_field = ui.FloatField(name="altitude", precision=6)
                            altitude_field.model.set_value(self._delegate._altitude)
                            self._delegate.set_altitude_field(altitude_field.model)
                            ui.Circle(
                                name="transform",
                                width=20,
                                height=20,
                                radius=3.5,
                                size_policy=ui.CircleSizePolicy.FIXED,
                            )

                        with ui.HStack():
                            ui.Button(
                                "Set",
                                enabled=True,
                                clicked_fn=self._delegate.on_set_new_global_coordinates,
                            )
                            ui.Button(
                                "Reset",
                                enabled=True,
                                clicked_fn=self._delegate.on_reset_global_coordinates,
                            )
                            ui.Button(
                                "Make Default",
                                enabled=True,
                                clicked_fn=self._delegate.on_set_new_default_global_coordinates,
                            )

                ui.Spacer(height=0)

                with ui.HStack():
                    # Add a thumbnail image to have a preview of the world that is about to be loaded
                    with ui.ZStack(
                        width=WidgetWindow.LABEL_PADDING,
                        height=WidgetWindow.BUTTON_HEIGHT * 2,
                    ):
                        ui.Rectangle()
                        ui.Image(
                            WORLD_THUMBNAIL,
                            fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                            alignment=ui.Alignment.LEFT_CENTER,
                        )

                    ui.Spacer(width=WidgetWindow.GENERAL_SPACING)

                    with ui.VStack():
                        # Button for loading environment only
                        ui.Button(
                            "Load Environment",
                            height=WidgetWindow.BUTTON_HEIGHT,
                            clicked_fn=self._delegate.on_load_environment,
                            style=WidgetWindow.BUTTON_BASE_STYLE,
                        )

                        # Button to save environment only
                        ui.Button(
                            "Save Environment",
                            height=WidgetWindow.BUTTON_HEIGHT,
                            clicked_fn=self._delegate.on_save_environment,
                            style=WidgetWindow.BUTTON_BASE_STYLE,
                        )

    def _robot_selection_frame(self):
        """
        Method that implements a frame that allows the user to choose which robot that is about to be spawned
        """

        # --------------------------
        # Function UI starts here
        # --------------------------

        # Frame for selecting the vehicle to load
        with ui.CollapsableFrame(title="Vehicle Selection"):
            with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                with ui.HStack():
                    # Add a thumbnail image to have a preview of the world that is about to be loaded
                    with ui.ZStack(
                        width=WidgetWindow.LABEL_PADDING,
                        height=WidgetWindow.BUTTON_HEIGHT * 2,
                    ):
                        ui.Rectangle()
                        ui.Image(
                            THUMBNAIL,
                            fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                            alignment=ui.Alignment.CENTER,
                        )
                    ui.Spacer(width=10)

                    with ui.VStack():
                        with ui.HStack():
                            # Iterate over all existing robots in the extension
                            ui.Label(
                                "Vehicle Model",
                                name="label",
                                width=WidgetWindow.LABEL_PADDING,
                                alignment=ui.Alignment.TOP,
                            )

                            # Combo box with the available vehicles to select from
                            dropdown_menu = ui.ComboBox(0, name="vehicles")
                            for vehicle in VEHICLES:
                                dropdown_menu.model.append_child_item(
                                    None, ui.SimpleStringModel(vehicle)
                                )
                            self._delegate.set_vehicle_dropdown(dropdown_menu.model)

                            # Store reference to the dropdown for refreshing
                            self._vehicle_dropdown_menu = dropdown_menu

                            # Add refresh button
                            ui.Button(
                                "↻",
                                width=25,
                                height=25,
                                clicked_fn=self._refresh_vehicles,
                                tooltip="Refresh vehicle list",
                            )

                        with ui.HStack():
                            ui.Label(
                                "Vehicle ID",
                                name="label",
                                width=WidgetWindow.LABEL_PADDING,
                                alignment=ui.Alignment.TOP,
                            )
                            vehicle_id_field = ui.IntField()
                            self._delegate.set_vehicle_id_field(vehicle_id_field.model)

                # Buttons to load and save the vehicle
                with ui.HStack():
                    ui.Button(
                        "Load Vehicle",
                        height=WidgetWindow.BUTTON_HEIGHT,
                        clicked_fn=self._delegate.on_load_vehicle,
                        style=WidgetWindow.BUTTON_BASE_STYLE,
                    )
                    ui.Button(
                        "Save Vehicle",
                        height=WidgetWindow.BUTTON_HEIGHT,
                        clicked_fn=self._delegate.on_save_vehicle,
                        style=WidgetWindow.BUTTON_BASE_STYLE,
                    )

    def _px4_configuration_frame(self):
        """
        A helper function to create a frame for configuring PX4 settings.
        """
        with ui.CollapsableFrame(title="PX4 Configuration"):
            with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                ui.Spacer(height=WidgetWindow.GENERAL_SPACING)
                with ui.HStack():
                    ui.Label(
                        "Auto-launch PX4",
                        name="label",
                        width=WidgetWindow.LABEL_PADDING - 20,
                    )
                    px4_checkbox = ui.CheckBox()
                    px4_checkbox.model.set_value(self._delegate._autostart_px4)
                    self._delegate.set_px4_autostart_checkbox(px4_checkbox.model)

                with ui.HStack():
                    ui.Label(
                        "PX4 Path", name="label", width=WidgetWindow.LABEL_PADDING - 20
                    )
                    px4_path_field = ui.StringField(name="px4_path", width=300)
                    px4_path_field.model.set_value(self._delegate._px4_dir)
                    self._delegate.set_px4_directory_field(px4_path_field.model)

                    ui.Button(
                        "Reset",
                        enabled=True,
                        clicked_fn=self._delegate.on_reset_px4_path,
                    )
                    ui.Button(
                        "Make Default",
                        enabled=True,
                        clicked_fn=self._delegate.on_set_new_default_px4_path,
                    )

                with ui.HStack():
                    ui.Label(
                        "PX4 airframe",
                        name="label",
                        width=WidgetWindow.LABEL_PADDING - 20,
                    )
                    px4_airframe_field = ui.StringField(name="px4_model")
                    px4_airframe_field.model.set_value(self._delegate._px4_airframe)
                    self._delegate.set_px4_airframe_field(px4_airframe_field.model)

    def _viewport_camera_frame(self):
        """
        Method that implements a frame that allows the user to choose what is the viewport camera pose easily
        """

        all_axis = ["X", "Y", "Z"]
        colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
        default_values = [5.0, 5.0, 5.0]
        target_default_values = [0.0, 0.0, 0.0]

        # Frame for setting the camera to visualize the vehicle in the simulator viewport
        with ui.CollapsableFrame("Viewport Camera"):
            with ui.VStack(spacing=8):
                ui.Spacer(height=0)

                # Iterate over the position and rotation menus
                with ui.HStack():
                    with ui.HStack():
                        ui.Label("Position", name="transform", width=50, height=20)
                        ui.Spacer()
                    # Fields X, Y and Z
                    for axis, default_value in zip(all_axis, default_values):
                        with ui.HStack():
                            with ui.ZStack(width=15):
                                ui.Rectangle(
                                    width=15,
                                    height=20,
                                    style={
                                        "background_color": colors[axis],
                                        "border_radius": 3,
                                        "corner_flag": ui.CornerFlag.LEFT,
                                    },
                                )
                                ui.Label(
                                    axis,
                                    height=20,
                                    name="transform_label",
                                    alignment=ui.Alignment.CENTER,
                                )
                            float_drag = ui.FloatDrag(
                                name="transform", min=-1000000, max=1000000, step=0.01
                            )
                            float_drag.model.set_value(default_value)
                            # Save the model of each FloatDrag such that we can access its values later on
                            self._camera_transform_models.append(float_drag.model)
                            ui.Circle(
                                name="transform",
                                width=20,
                                height=20,
                                radius=3.5,
                                size_policy=ui.CircleSizePolicy.FIXED,
                            )

                # Iterate over the position and rotation menus
                with ui.HStack():
                    with ui.HStack():
                        ui.Label("Target", name="transform", width=50, height=20)
                        ui.Spacer()
                    # Fields X, Y and Z
                    for axis, default_value in zip(all_axis, target_default_values):
                        with ui.HStack():
                            with ui.ZStack(width=15):
                                ui.Rectangle(
                                    width=15,
                                    height=20,
                                    style={
                                        "background_color": colors[axis],
                                        "border_radius": 3,
                                        "corner_flag": ui.CornerFlag.LEFT,
                                    },
                                )
                                ui.Label(
                                    axis,
                                    height=20,
                                    name="transform_label",
                                    alignment=ui.Alignment.CENTER,
                                )
                            float_drag = ui.FloatDrag(
                                name="transform", min=-1000000, max=1000000, step=0.01
                            )
                            float_drag.model.set_value(default_value)
                            # Save the model of each FloatDrag such that we can access its values later on
                            self._camera_transform_models.append(float_drag.model)
                            ui.Circle(
                                name="transform",
                                width=20,
                                height=20,
                                radius=3.5,
                                size_policy=ui.CircleSizePolicy.FIXED,
                            )

                # Button to set the camera view
                ui.Button(
                    "Set Camera Pose",
                    height=WidgetWindow.BUTTON_HEIGHT,
                    clicked_fn=self._delegate.on_set_viewport_camera,
                    style=WidgetWindow.BUTTON_BASE_STYLE,
                )
                ui.Spacer()

    # ------------------------------------------------------------------------------------------------
    # TODO - optimize the reading of values from the transform widget. This could be one function only
    # ------------------------------------------------------------------------------------------------

    def get_selected_vehicle_attitude(self):
        """
        Return fixed vehicle spawn position and orientation (always at origin).
        """
        # Always spawn at origin with no rotation
        vehicle_pos = np.array([0.0, 0.0, 0.0])
        vehicle_orientation = np.array([0.0, 0.0, 0.0])
        return vehicle_pos, vehicle_orientation

    def get_selected_camera_pos(self):
        """
        Method that returns the currently selected camera position in the camera transform widget
        """

        # Extract the camera desired position and the target it is pointing to
        if len(self._camera_transform_models) == 6:
            camera_pos = np.array(
                [
                    self._camera_transform_models[i].get_value_as_float()
                    for i in range(3)
                ]
            )
            camera_target = np.array(
                [
                    self._camera_transform_models[i].get_value_as_float()
                    for i in range(3, 6)
                ]
            )
            return camera_pos, camera_target

        return None, None

    def _refresh_vehicles(self):
        """
        Method to refresh the vehicle dropdown by rescanning the assets directory
        """
        # Import here to avoid circular imports
        from pegasus.simulator.params import refresh_vehicles

        # Refresh the vehicles dictionary
        updated_vehicles = refresh_vehicles()

        # Clear the current dropdown items
        self._vehicle_dropdown_menu.model.clear()

        # Repopulate with new items
        for vehicle in updated_vehicles:
            self._vehicle_dropdown_menu.model.append_child_item(
                None, ui.SimpleStringModel(vehicle)
            )

        # Update the delegate's vehicles names list
        self._delegate._vehicles_names = list(updated_vehicles.keys())

        carb.log_info(f"Vehicle list refreshed: found {len(updated_vehicles)} vehicles")

    def _gimbal_control_frame(self):
        """
        Method that implements manual gimbal control interface
        """

        with ui.CollapsableFrame("Gimbal Control", collapsed=False):
            with ui.VStack(height=0, spacing=5, name="gimbal_frame_v_stack"):
                ui.Spacer(height=WidgetWindow.GENERAL_SPACING)

                # Gimbal control enable/disable
                with ui.HStack():
                    ui.Label("Gimbal Control", width=WidgetWindow.LABEL_PADDING)
                    self._gimbal_enabled_checkbox = ui.CheckBox(width=20)
                    self._ui_subscriptions.append(
                        self._gimbal_enabled_checkbox.model.subscribe_value_changed_fn(
                            lambda m: self._delegate.on_gimbal_enabled_changed(
                                m.get_value_as_bool()
                            )
                        )
                    )

                ui.Spacer(height=5)

                # Pitch control (-135° to +45°)
                with ui.HStack():
                    ui.Label("Pitch", width=80)
                    self._gimbal_pitch_slider = ui.FloatSlider(
                        min=-135.0, max=45.0, default=-90.0, width=150, height=20
                    )
                    self._gimbal_pitch_value = ui.FloatField(width=60, height=20)
                    self._gimbal_pitch_value.model.set_value(-90.0)

                # Wire up pitch controls
                self._ui_subscriptions.append(
                    self._gimbal_pitch_slider.model.subscribe_value_changed_fn(
                        lambda m: [
                            self._gimbal_pitch_value.model.set_value(
                                m.get_value_as_float()
                            ),
                            self._delegate.on_gimbal_pitch_changed(
                                m.get_value_as_float()
                            ),
                        ]
                    )
                )

                self._ui_subscriptions.append(
                    self._gimbal_pitch_value.model.subscribe_value_changed_fn(
                        lambda m: [
                            self._gimbal_pitch_slider.model.set_value(
                                m.get_value_as_float()
                            ),
                            self._delegate.on_gimbal_pitch_changed(
                                m.get_value_as_float()
                            ),
                        ]
                    )
                )

                # Roll control (-45° to +45°)
                with ui.HStack():
                    ui.Label("Roll", width=80)
                    self._gimbal_roll_slider = ui.FloatSlider(
                        min=-45.0, max=45.0, default=0.0, width=150, height=20
                    )
                    self._gimbal_roll_value = ui.FloatField(width=60, height=20)
                    self._gimbal_roll_value.model.set_value(0.0)

                # Wire up roll controls
                self._ui_subscriptions.append(
                    self._gimbal_roll_slider.model.subscribe_value_changed_fn(
                        lambda m: [
                            self._gimbal_roll_value.model.set_value(
                                m.get_value_as_float()
                            ),
                            self._delegate.on_gimbal_roll_changed(
                                m.get_value_as_float()
                            ),
                        ]
                    )
                )

                self._ui_subscriptions.append(
                    self._gimbal_roll_value.model.subscribe_value_changed_fn(
                        lambda m: [
                            self._gimbal_roll_slider.model.set_value(
                                m.get_value_as_float()
                            ),
                            self._delegate.on_gimbal_roll_changed(
                                m.get_value_as_float()
                            ),
                        ]
                    )
                )

                # Yaw control (-180° to +180°)
                with ui.HStack():
                    ui.Label("Yaw", width=80)
                    self._gimbal_yaw_slider = ui.FloatSlider(
                        min=-180.0, max=180.0, default=0.0, width=150, height=20
                    )
                    self._gimbal_yaw_value = ui.FloatField(width=60, height=20)
                    self._gimbal_yaw_value.model.set_value(0.0)

                # Wire up yaw controls
                self._ui_subscriptions.append(
                    self._gimbal_yaw_slider.model.subscribe_value_changed_fn(
                        lambda m: [
                            self._gimbal_yaw_value.model.set_value(
                                m.get_value_as_float()
                            ),
                            self._delegate.on_gimbal_yaw_changed(
                                m.get_value_as_float()
                            ),
                        ]
                    )
                )

                self._ui_subscriptions.append(
                    self._gimbal_yaw_value.model.subscribe_value_changed_fn(
                        lambda m: [
                            self._gimbal_yaw_slider.model.set_value(
                                m.get_value_as_float()
                            ),
                            self._delegate.on_gimbal_yaw_changed(
                                m.get_value_as_float()
                            ),
                        ]
                    )
                )

                ui.Spacer(height=5)

                # Preset buttons
                with ui.HStack():
                    button1 = ui.Button(
                        "Point Down",
                        clicked_fn=self._on_gimbal_point_down_clicked,
                        height=WidgetWindow.BUTTON_HEIGHT // 2,
                        style=WidgetWindow.BUTTON_BASE_STYLE,
                    )
                    self._gimbal_buttons.append(button1)

                    button2 = ui.Button(
                        "Point Forward",
                        clicked_fn=self._on_gimbal_center_clicked,
                        height=WidgetWindow.BUTTON_HEIGHT // 2,
                        style=WidgetWindow.BUTTON_BASE_STYLE,
                    )
                    self._gimbal_buttons.append(button2)

                with ui.HStack():
                    button3 = ui.Button(
                        "Stabilize Mode",
                        clicked_fn=self._on_gimbal_stabilize_clicked,
                        height=WidgetWindow.BUTTON_HEIGHT // 2,
                        style=WidgetWindow.BUTTON_BASE_STYLE,
                    )
                    self._gimbal_buttons.append(button3)

                    button4 = ui.Button(
                        "Manual Mode",
                        clicked_fn=self._on_gimbal_manual_clicked,
                        height=WidgetWindow.BUTTON_HEIGHT // 2,
                        style=WidgetWindow.BUTTON_BASE_STYLE,
                    )
                    self._gimbal_buttons.append(button4)

                ui.Spacer(height=WidgetWindow.GENERAL_SPACING)

    def _set_gimbal_preset(self, pitch: float, roll: float, yaw: float):
        """
        Set gimbal to a preset position and update UI controls.
        """
        # Update sliders
        self._gimbal_pitch_slider.model.set_value(pitch)
        self._gimbal_roll_slider.model.set_value(roll)
        self._gimbal_yaw_slider.model.set_value(yaw)

        # Update value fields
        self._gimbal_pitch_value.model.set_value(pitch)
        self._gimbal_roll_value.model.set_value(roll)
        self._gimbal_yaw_value.model.set_value(yaw)

        # Notify delegate
        self._delegate.on_gimbal_preset_clicked(pitch, roll, yaw)

    def _on_gimbal_point_down_clicked(self):
        """Handle Point Down preset button click"""
        self._set_gimbal_preset(-90, 0, 0)

    def _on_gimbal_center_clicked(self):
        """Handle Center preset button click"""
        self._set_gimbal_preset(0, 0, 0)

    def _on_gimbal_stabilize_clicked(self):
        """Handle Stabilize mode button click"""
        if self._delegate:
            self._delegate.on_gimbal_stabilize_clicked()

    def _on_gimbal_manual_clicked(self):
        """Handle Manual mode button click"""
        if self._delegate:
            self._delegate.on_gimbal_manual_clicked()

    def _motor_control_frame(self):
        """
        Method that implements manual motor control interface
        """

        with ui.CollapsableFrame("Motor Control (Manual Override)", collapsed=False):
            with ui.VStack(height=0, spacing=5, name="motor_frame_v_stack"):
                ui.Spacer(height=WidgetWindow.GENERAL_SPACING)

                # Motor control enable/disable with warning
                with ui.HStack():
                    ui.Label("Enable Manual Control", width=WidgetWindow.LABEL_PADDING)
                    self._motor_control_enabled_checkbox = ui.CheckBox(width=20)
                    self._ui_subscriptions.append(
                        self._motor_control_enabled_checkbox.model.subscribe_value_changed_fn(
                            lambda m: self._delegate.on_manual_motor_enabled_changed(
                                m.get_value_as_bool()
                            )
                        )
                    )

                # Warning label (initially hidden)
                with ui.HStack():
                    ui.Spacer()
                    ui.Label(
                        "⚠️  WARNING: Overrides PX4 Commands",
                        style={"color": 0xFF4444FF, "font_size": 12},
                        alignment=ui.Alignment.CENTER,
                    )
                    ui.Spacer()

                ui.Spacer(height=5)

                # Individual motor controls
                self._motor_sliders = []
                self._motor_values = []

                # Motor position labels for clarity (PX4 X-configuration)
                motor_labels = [
                    "M0 (FR)",  # Motor 0: Front-Right
                    "M1 (RL)",  # Motor 1: Rear-Left
                    "M2 (FL)",  # Motor 2: Front-Left
                    "M3 (RR)",  # Motor 3: Rear-Right
                ]

                for i in range(4):
                    with ui.HStack():
                        ui.Label(motor_labels[i], width=60)
                        slider = ui.FloatSlider(
                            min=0.0, max=100.0, default=0.0, width=120, height=20
                        )
                        value = ui.FloatField(width=50, height=20)
                        value.model.set_value(0.0)
                        ui.Label("%", width=15)

                        # Store references
                        self._motor_sliders.append(slider)
                        self._motor_values.append(value)

                        # Wire up controls
                        self._ui_subscriptions.append(
                            slider.model.subscribe_value_changed_fn(
                                lambda m, motor_idx=i: [
                                    self._motor_values[motor_idx].model.set_value(
                                        m.get_value_as_float()
                                    ),
                                    self._delegate.on_motor_speed_changed(
                                        motor_idx, m.get_value_as_float()
                                    ),
                                ]
                            )
                        )

                        self._ui_subscriptions.append(
                            value.model.subscribe_value_changed_fn(
                                lambda m, motor_idx=i: [
                                    self._motor_sliders[motor_idx].model.set_value(
                                        m.get_value_as_float()
                                    ),
                                    self._delegate.on_motor_speed_changed(
                                        motor_idx, m.get_value_as_float()
                                    ),
                                ]
                            )
                        )

                ui.Spacer(height=5)

                # Master throttle control
                with ui.HStack():
                    ui.Label("Master", width=60)
                    self._master_throttle_slider = ui.FloatSlider(
                        min=0.0, max=100.0, default=0.0, width=120, height=20
                    )
                    self._master_throttle_value = ui.FloatField(width=50, height=20)
                    self._master_throttle_value.model.set_value(0.0)
                    ui.Label("%", width=15)

                # Wire up master throttle
                self._ui_subscriptions.append(
                    self._master_throttle_slider.model.subscribe_value_changed_fn(
                        lambda m: [
                            self._master_throttle_value.model.set_value(
                                m.get_value_as_float()
                            ),
                            self._delegate.on_master_throttle_changed(
                                m.get_value_as_float()
                            ),
                        ]
                    )
                )

                self._ui_subscriptions.append(
                    self._master_throttle_value.model.subscribe_value_changed_fn(
                        lambda m: [
                            self._master_throttle_slider.model.set_value(
                                m.get_value_as_float()
                            ),
                            self._delegate.on_master_throttle_changed(
                                m.get_value_as_float()
                            ),
                        ]
                    )
                )

                ui.Spacer(height=WidgetWindow.GENERAL_SPACING)
