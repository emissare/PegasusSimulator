"""
| File: extension.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Implements the Pegasus_SimulatorExtension which omni.ext.IExt that is created when this class is enabled. In turn, this class initializes the extension widget.
"""
__all__ = ["Pegasus_SimulatorExtension"]

# Python garbage collenction and asyncronous API
import gc
import asyncio
import weakref
from functools import partial
from threading import Timer

# Omniverse general API
import pxr
import carb
import omni.ext
import omni.usd
import omni.kit.ui
import omni.kit.app
import omni.ui as ui
import omni.timeline

from omni.kit.viewport.utility import get_active_viewport

# Pegasus Extension Files and API
from pegasus.simulator.params import MENU_PATH, WINDOW_TITLE
#from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Setting up the UI for the extension's Widget
from pegasus.simulator.ui.ui_window import WidgetWindow
from pegasus.simulator.ui.ui_delegate import UIDelegate


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class Pegasus_SimulatorExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):

        carb.log_info("Pegasus Simulator is starting up")

        # Save the extension id
        self._ext_id = ext_id

        # Initialize state tracking for clean lifecycle management
        self._menu_created = False
        self._window_created = False

        # Create the UI of the app and its manager
        self.ui_delegate = None
        self.ui_window = None

        # Add the extension to the editor menu inside isaac sim
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            self._menu = editor_menu.add_item(MENU_PATH, self.show_window, toggle=True, value=True)
            self._menu_created = True
            carb.log_info(f"Menu item created at path: {MENU_PATH}")
        else:
            carb.log_warn("Editor menu not available, skipping menu creation")

        # Show the window directly (no workspace callback needed)
        carb.log_info("Creating window directly")
        self.show_window(None, True)
        carb.log_info("Window creation completed")


    def show_window(self, menu, show):
        """
        Method that controls whether a widget window is created or not
        """
        carb.log_info(f"show_window called with menu={menu}, show={show}")

        if show == True:
            if not self._window_created:
                # Create a window and its delegate
                carb.log_info("Creating UIDelegate...")
                self.ui_delegate = UIDelegate()
                carb.log_info("Creating WidgetWindow...")
                self.ui_window = WidgetWindow(self.ui_delegate)
                carb.log_info(f"Window created successfully: {self.ui_window}")
                self.ui_window.set_visibility_changed_fn(self._visibility_changed_fn)
                self._window_created = True
            else:
                # Window already exists, just make it visible
                if self.ui_window:
                    self.ui_window.visible = True
                    carb.log_info("Window already exists, made visible")

        # If we have a window and we are not supposed to show it, then change its visibility
        elif self.ui_window:
            self.ui_window.visible = False
            carb.log_info("Window hidden")

    def _visibility_changed_fn(self, visible):
        """
        This method is invoked when the user pressed the "X" to close the extension window
        """

        # Update the Isaac sim menu visibility
        self._set_menu(visible)

        if not visible:
            # Destroy the window, because we create a new one in the show window method
            asyncio.ensure_future(self._destroy_window_async())

    def _set_menu(self, visible):
        """
        Method that updates the isaac sim ui menu to create the Widget window on and off
        """
        editor_menu = omni.kit.ui.get_editor_menu()
        if editor_menu:
            editor_menu.set_value(MENU_PATH, visible)

    async def _destroy_window_async(self):
        """
        Async window destruction - only handles window and delegate cleanup
        Menu and extension-level cleanup is handled in on_shutdown()
        """
        carb.log_info("Starting async window destruction")

        # Wait one frame before it gets destructed (from NVidia example)
        await omni.kit.app.get_app().next_update_async()

        # Only clean up window and delegate - not extension-level resources
        if self._window_created and self.ui_window:
            carb.log_info("Destroying window")
            self.ui_window.destroy()
            self.ui_window = None
            self._window_created = False
            carb.log_info("Window destroyed successfully")

        # Clear delegate reference after window cleanup
        if self.ui_delegate:
            try:
                carb.log_info("Calling UIDelegate cleanup from async destroy")
                self.ui_delegate.cleanup()
                carb.log_info("UIDelegate cleanup from async destroy completed")
            except Exception as e:
                carb.log_error(f"Failed UIDelegate cleanup from async destroy: {str(e)}")
            self.ui_delegate = None
        else:
            carb.log_info("UIDelegate already None in async destroy")

        carb.log_info("Async window destruction completed")

    def on_shutdown(self):
        """
        Callback called when the extension is shutdown
        Handles only extension-level cleanup (menu, workspace registration)
        Window cleanup is handled separately in _destroy_window_async()
        """
        carb.log_info("Pegasus Isaac extension shutdown started")

        # Clean up extension-level resources in proper order

        # 1. Remove editor menu item (if it was created)
        if self._menu_created:
            editor_menu = omni.kit.ui.get_editor_menu()
            if editor_menu:
                try:
                    carb.log_info(f"Removing editor menu item: {MENU_PATH}")
                    editor_menu.remove_item(MENU_PATH)
                    carb.log_info("Editor menu item removed successfully")
                except Exception as e:
                    carb.log_info(f"Menu item {MENU_PATH} already removed or not found: {str(e)}")
            else:
                carb.log_warn("Editor menu not available for cleanup")
            self._menu_created = False
        else:
            carb.log_info("Menu was not created, skipping menu cleanup")

        # Clear menu reference
        self._menu = None

        # 2. Clean up window if it still exists (shouldn't normally happen)
        if self._window_created:
            carb.log_info("Window still exists during shutdown, cleaning up")
            if self.ui_window:
                self.ui_window.destroy()
                self.ui_window = None
            if self.ui_delegate:
                try:
                    carb.log_info("Calling UIDelegate cleanup from on_shutdown")
                    self.ui_delegate.cleanup()
                    carb.log_info("UIDelegate cleanup from on_shutdown completed")
                except Exception as e:
                    carb.log_error(f"Failed UIDelegate cleanup from on_shutdown: {str(e)}")
                self.ui_delegate = None
            self._window_created = False
        else:
            carb.log_info("Window already cleaned up, skipping window cleanup")

        # 3. Reset all state flags
        self._menu_created = False
        self._window_created = False

        carb.log_info("Pegasus Isaac extension shutdown completed")

        # Call the garbage collector
        gc.collect()
