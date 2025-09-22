"""
| File: params.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: File that defines the base configurations for the Pegasus Simulator.
"""
import os
from pathlib import Path
import glob

import isaacsim.storage.native as nucleus

# Extension configuration
EXTENSION_NAME = "Pegasus Simulator"
WINDOW_TITLE = "Pegasus Simulator"
MENU_PATH = "Window/" + WINDOW_TITLE
DOC_LINK = "https://docs.omniverse.nvidia.com"
EXTENSION_OVERVIEW = "This extension shows how to incorporate drones into Isaac Sim"

# Get the current directory of where this extension is located
EXTENSION_FOLDER_PATH = Path(os.path.dirname(os.path.realpath(__file__)))
ROOT = str(EXTENSION_FOLDER_PATH.parent.parent.parent.resolve())

# Get the configurations file path
CONFIG_FILE = ROOT + "/pegasus.simulator/config/configs.yaml"

# Define the Extension Assets Path
ASSET_PATH = ROOT + "/pegasus.simulator/pegasus/simulator/assets"

# Define the vehicles config path
VEHICLES_CONFIG_PATH = ROOT + "/pegasus.simulator/config/vehicles"

# Function to refresh vehicles list from YAML configs
def refresh_vehicles():
    """Refresh the VEHICLES dictionary by scanning for YAML vehicle configurations"""
    global VEHICLES
    VEHICLES = {}
    if os.path.exists(VEHICLES_CONFIG_PATH):
        # Find all .yaml files in the vehicles config directory
        for file_path in glob.glob(os.path.join(VEHICLES_CONFIG_PATH, "*.yaml")):
            # Get the filename without extension as the display name
            filename = os.path.basename(file_path)
            name = os.path.splitext(filename)[0]
            # Make the name more user-friendly by replacing underscores with spaces and capitalizing
            display_name = name.replace("_", " ").title()
            VEHICLES[display_name] = file_path
    return VEHICLES

# Dynamically scan for YAML vehicle configurations
VEHICLES = refresh_vehicles()

# Setup the default simulation environments path
NVIDIA_ASSETS_PATH = str(nucleus.get_assets_root_path())
ISAAC_SIM_ENVIRONMENTS = "/Isaac/Environments"
NVIDIA_SIMULATION_ENVIRONMENTS = {
    "Default Environment": "Grid/default_environment.usd",
    "Black Gridroom": "Grid/gridroom_black.usd",
    "Curved Gridroom": "Grid/gridroom_curved.usd",
    "Hospital": "Hospital/hospital.usd",
    "Office": "Office/office.usd",
    "Simple Room": "Simple_Room/simple_room.usd",
    "Warehouse": "Simple_Warehouse/warehouse.usd",
    "Warehouse with Forklifts": "Simple_Warehouse/warehouse_with_forklifts.usd",
    "Warehouse with Shelves": "Simple_Warehouse/warehouse_multiple_shelves.usd",
    "Full Warehouse": "Simple_Warehouse/full_warehouse.usd",
    "Flat Plane": "Terrains/flat_plane.usd",
    "Rough Plane": "Terrains/rough_plane.usd",
    "Slope Plane": "Terrains/slope.usd",
    "Stairs Plane": "Terrains/stairs.usd",
}

OMNIVERSE_ENVIRONMENTS = {
    "Exhibition Hall": "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/NVIDIA/Assets/Scenes/Templates/Interior/ZetCG_ExhibitionHall.usd"
}

# Dynamically scan for local environments
LOCAL_ENVIRONMENTS = {}
worlds_path = ASSET_PATH + "/Worlds"
if os.path.exists(worlds_path):
    # Find all .usd and .usda files
    for file_path in glob.glob(os.path.join(worlds_path, "**/*.usd*"), recursive=True):
        # Get the filename without extension as the display name
        filename = os.path.basename(file_path)
        name = os.path.splitext(filename)[0]
        LOCAL_ENVIRONMENTS[name] = file_path

SIMULATION_ENVIRONMENTS = {}

# Add the Isaac Sim assets to the list
for asset in NVIDIA_SIMULATION_ENVIRONMENTS:
    SIMULATION_ENVIRONMENTS[asset] = (
        NVIDIA_ASSETS_PATH + ISAAC_SIM_ENVIRONMENTS + "/" + NVIDIA_SIMULATION_ENVIRONMENTS[asset]
    )

# Add the omniverse assets to the list
for asset in OMNIVERSE_ENVIRONMENTS:
    SIMULATION_ENVIRONMENTS[asset] = OMNIVERSE_ENVIRONMENTS[asset]

# Add the local assets to the list
for asset in LOCAL_ENVIRONMENTS:
    SIMULATION_ENVIRONMENTS[asset] = LOCAL_ENVIRONMENTS[asset]

BACKENDS = {
    "px4": "px4"
}

# Define the default settings for the simulation environment
WORLD_SETTINGS = {
    'px4': {
        "physics_dt": 1.0 / 250.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
        "device": "cpu"
    }
}
DEFAULT_WORLD_SETTINGS = WORLD_SETTINGS['px4']

# Define where the thumbnail of the vehicle is located (generic vehicle thumbnail)
THUMBNAIL = ROOT + "/pegasus.simulator/data/preview.png"

# Define where the thumbail of the world is located
WORLD_THUMBNAIL = ASSET_PATH + "/Worlds/Empty_thumbnail.png"

