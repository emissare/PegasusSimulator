# Camera View and Following Guide for PegasusSimulator

This guide describes different methods to set up third-person camera views and following systems for quadrotors in Isaac Sim using the PegasusSimulator framework.

## Quick Start: Simple Camera Following

### Method 1: Camera Attachment (Recommended for Beginners)

The simplest way to create a following camera is to attach it directly to your quadrotor:

1. **Create a Camera**:
   - In Isaac Sim: `Create > Camera` from the menu bar
   - Or: Position your viewport to desired view and select `Camera > Create Camera from View`

2. **Attach Camera to Quadrotor**:
   - In the **Stage** panel, find your camera prim (e.g., `/World/Camera`)
   - **Drag and drop** the camera prim under your quadrotor prim (e.g., `/World/quadrotor`)
   - The camera is now a child of the quadrotor and will move with it

3. **Position the Camera**:
   - Select the camera in the Stage panel
   - In the **Property** panel, adjust the `Transform > Translate` values:
     - For third-person view: `X: -5.0, Y: 0.0, Z: 3.0` (5m behind, 3m above)
     - For side view: `X: 0.0, Y: -5.0, Z: 2.0` (5m to the side, 2m above)

4. **Set Camera as Active**:
   - Click the **camera button** in the viewport (top-left corner)
   - Select your camera from the dropdown menu

### Method 2: Viewport Camera Controls

For manual camera control during simulation:

- **Focus on Object**: Select your quadrotor and press `F` to center and zoom
- **Orbit Camera**: Hold `Alt + Left Mouse Button` and drag to orbit around the quadrotor
- **Zoom**: Use mouse scroll wheel or `Alt + Right Mouse Button`
- **Pan**: Use `Middle Mouse Button` to pan the view

## Advanced Camera Following

### Method 3: Isaac Lab ViewportCameraController

For programmatic camera tracking with smooth following:

```python
from omni.isaac.lab.envs.ui import ViewportCameraController

# Create camera controller
camera_controller = ViewportCameraController(
    viewport_api=viewport_api,
    origin_type="asset_root",  # Track the root of an asset
    asset_name="quadrotor"     # Name of your quadrotor
)

# The camera will automatically track the quadrotor's movement
```

**Origin Types**:
- `"world"`: Static camera at world center
- `"env"`: Static camera at environment center
- `"asset_root"`: Dynamic tracking of specified asset (recommended for following)

### Method 4: Programmatic Camera Control

Using the PegasusInterface for dynamic camera positioning:

```python
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Get vehicle position
vehicle_position = vehicle.get_world_pose()[0]  # [x, y, z]

# Calculate camera position (5m behind, 3m above)
camera_offset = [-5.0, 0.0, 3.0]
camera_position = vehicle_position + camera_offset

# Set viewport camera
pg = PegasusInterface()
pg.set_viewport_camera(
    camera_position=camera_position.tolist(),
    camera_target=vehicle_position.tolist()
)
```

## Camera Inspector Extension

Access advanced camera management tools:

1. **Open Extension**: `Tools > Robotics > Camera Inspector`
2. **Features**:
   - Copy camera position/orientation to clipboard
   - Create new viewports for cameras
   - Manage multiple camera views simultaneously

## Example Integration

### Modifying Existing Examples

To add camera following to any Pegasus example:

```python
# In your PegasusApp.__init__() method, after creating the vehicle:

# Method 1: Set static third-person view
self.pg.set_viewport_camera([5.0, 5.0, 3.0], [0.0, 0.0, 0.0])

# Method 2: In your run() loop for dynamic following
while simulation_app.is_running():
    # Get vehicle position
    vehicle_pos = self.vehicle.get_world_pose()[0]

    # Update camera to follow
    camera_pos = vehicle_pos + np.array([-5.0, 0.0, 3.0])
    self.pg.set_viewport_camera(camera_pos.tolist(), vehicle_pos.tolist())

    # Continue simulation
    self.world.step(render=True)
```

## Camera Positioning Reference

### Common Third-Person Positions

| View Type | X Offset | Y Offset | Z Offset | Description |
|-----------|----------|----------|----------|-------------|
| Chase Cam | -5.0 | 0.0 | 3.0 | Behind and above |
| Side View | 0.0 | -5.0 | 2.0 | Side view |
| High Angle | -3.0 | -3.0 | 5.0 | Diagonal high view |
| Close Follow | -2.0 | 0.0 | 1.0 | Close behind |

### Camera Orientation Tips

- **Look-at Target**: Always point camera toward vehicle center
- **Smooth Following**: Use interpolation for gradual camera movement
- **Field of View**: Adjust camera `fov` property for wider/narrower view
- **Clipping Planes**: Set appropriate `near` and `far` clipping distances

## Troubleshooting

### Camera Not Following
- **Check Hierarchy**: Ensure camera is child of vehicle in Stage panel
- **Transform Values**: Verify camera position relative to vehicle
- **Active Camera**: Make sure correct camera is selected in viewport

### Performance Issues
- **Update Rate**: Limit camera updates to rendering framerate
- **Distance**: Avoid extreme camera distances that require high precision
- **Multiple Cameras**: Use Camera Inspector to manage multiple viewports

### Camera Attachment Issues
- **TiledCamera**: Some camera types may not follow properly - use standard Camera prim
- **Vehicle Movement**: Ensure vehicle physics are working correctly
- **Coordinate Frames**: Check that camera offsets are in correct coordinate system

## Best Practices

1. **Start Simple**: Use camera attachment method first, then advance to programmatic control
2. **Test Positions**: Use viewport controls to find good camera angles before implementing
3. **Smooth Movement**: Always interpolate camera position changes for better visual experience
4. **Multiple Views**: Set up multiple cameras for different perspectives
5. **Performance**: Update camera position only when necessary, not every frame

## Related Examples

- `examples/8_camera_vehicle.py`: Basic camera setup with vehicle
- `examples/5_python_multi_vehicle.py`: Multi-vehicle camera positioning
- `examples/6_paper_results.py`: Advanced camera positioning for research

# Gimbal Camera Control

This section covers how to implement gimbal-controlled cameras that can be controlled from companion computers using MAVLink protocols, providing realistic camera stabilization and directional control.

## Overview

A gimbal camera system provides:
- **3-axis stabilization** (roll, pitch, yaw)
- **Companion computer control** via MAVLink
- **Real-world compatibility** with standard gimbal protocols
- **Advanced features** like object tracking and directional constraints

## Gimbal Control Architecture

### MAVLink Gimbal Protocol v2

The industry-standard approach uses MAVLink Gimbal Protocol v2, where:

- **Companion Computer** acts as Gimbal Manager
- **PX4** handles gimbal device interface
- **Isaac Sim** simulates the physical gimbal
- **Same code** works on real and simulated vehicles

### Control Flow

```
Companion Computer -> MAVLink -> PX4 -> uORB Topics -> Isaac Sim Gimbal
```

## Implementation Methods

### Method 1: Direct MAVLink Control (Recommended)

Send gimbal commands directly from your companion computer:

```python
# Example using MAVSDK or pymavlink
from mavsdk import System
from mavsdk.gimbal import GimbalMode, ControlMode

async def control_gimbal():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Point camera downward
    await drone.gimbal.set_pitch_and_yaw(-90, 0)  # -90° pitch = downward

    # Continuous directional control
    await drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)  # Follow vehicle heading
```

### Method 2: PX4 Mount Control

Use PX4's mount control system:

```python
# Send MAV_CMD_DO_MOUNT_CONTROL command
def send_mount_control(mavlink_connection, pitch, roll, yaw):
    mavlink_connection.mav.command_long_send(
        target_system=1, target_component=1,
        command=mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        confirmation=0,
        param1=pitch,   # Pitch angle in degrees
        param2=roll,    # Roll angle in degrees
        param3=yaw,     # Yaw angle in degrees
        param4=0, param5=0, param6=0, param7=0
    )

# Point camera downward
send_mount_control(connection, pitch=-90, roll=0, yaw=0)
```

### Method 3: uORB Direct Control (PX4 SITL)

For direct integration with PX4 SITL:

```python
# Publish to gimbal_device_set_attitude topic
import px4_msgs.msg as px4_msgs

def publish_gimbal_attitude(node, pitch, roll, yaw):
    msg = px4_msgs.GimbalDeviceSetAttitude()
    msg.timestamp = int(time.time() * 1e6)
    msg.flags = px4_msgs.GimbalDeviceSetAttitude.FLAG_PITCH_VALID | \
                px4_msgs.GimbalDeviceSetAttitude.FLAG_ROLL_VALID | \
                px4_msgs.GimbalDeviceSetAttitude.FLAG_YAW_VALID

    # Convert to quaternion
    from scipy.spatial.transform import Rotation
    r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    q = r.as_quat()  # [x, y, z, w]

    msg.q = [q[3], q[0], q[1], q[2]]  # PX4 uses [w, x, y, z]

    gimbal_pub.publish(msg)
```

## Creating a Gimbal in Isaac Sim

### Option 1: Import PX4 Gazebo Gimbal Model

1. **Copy the gimbal model** from PX4:
   ```bash
   cp -r /path/to/PX4-Autopilot/Tools/simulation/gz/models/gimbal /path/to/isaac_sim_assets/
   ```

2. **Convert SDF to USD**: Use Isaac Sim's SDF importer or manual conversion

3. **Attach to quadrotor**:
   ```python
   # In your vehicle configuration
   gimbal_path = "/World/quadrotor/gimbal"
   isaac_sim.import_model(gimbal_model_path, gimbal_path)
   ```

### Option 2: Create Custom USD Gimbal

```python
# Create gimbal structure in USD
def create_gimbal(stage, parent_path):
    # Mount point
    mount_prim = stage.DefinePrim(f"{parent_path}/gimbal_mount", "Xform")

    # Yaw joint (vertical axis)
    yaw_joint = stage.DefinePrim(f"{parent_path}/gimbal_mount/yaw_joint", "RevoluteJoint")
    yaw_joint.CreateAttribute("physics:axis", "Z")
    yaw_joint.CreateAttribute("physics:lowerLimit", -180)
    yaw_joint.CreateAttribute("physics:upperLimit", 180)

    # Roll joint (horizontal axis)
    roll_joint = stage.DefinePrim(f"{parent_path}/gimbal_mount/yaw_joint/roll_joint", "RevoluteJoint")
    roll_joint.CreateAttribute("physics:axis", "X")
    roll_joint.CreateAttribute("physics:lowerLimit", -45)
    roll_joint.CreateAttribute("physics:upperLimit", 45)

    # Pitch joint and camera
    pitch_joint = stage.DefinePrim(f"{parent_path}/gimbal_mount/yaw_joint/roll_joint/pitch_joint", "RevoluteJoint")
    pitch_joint.CreateAttribute("physics:axis", "Y")
    pitch_joint.CreateAttribute("physics:lowerLimit", -135)
    pitch_joint.CreateAttribute("physics:upperLimit", 45)

    # Camera attached to pitch joint
    camera_prim = stage.DefinePrim(f"{parent_path}/gimbal_mount/yaw_joint/roll_joint/pitch_joint/camera", "Camera")

    return camera_prim
```

## Directional Constraints

### Always Point Downward

```python
class DownwardGimbalController:
    def __init__(self, vehicle, gimbal):
        self.vehicle = vehicle
        self.gimbal = gimbal

    def update(self):
        # Get vehicle attitude
        vehicle_pose = self.vehicle.get_world_pose()
        vehicle_quat = vehicle_pose[1]  # [x, y, z, w]

        # Calculate gimbal angles to maintain downward pointing
        # regardless of vehicle attitude
        from scipy.spatial.transform import Rotation
        vehicle_rot = Rotation.from_quat(vehicle_quat)

        # Target: camera pointing down in world frame
        target_direction = np.array([0, 0, -1])  # Down

        # Convert to vehicle frame
        vehicle_down = vehicle_rot.inv().apply(target_direction)

        # Convert to gimbal angles
        pitch = np.arctan2(vehicle_down[0], -vehicle_down[2])
        roll = np.arctan2(vehicle_down[1], -vehicle_down[2])
        yaw = 0  # Or follow vehicle heading

        # Send to gimbal
        self.send_gimbal_command(pitch, roll, yaw)
```

### Horizon Lock Stabilization

```python
class HorizonLockController:
    def __init__(self, vehicle, gimbal):
        self.vehicle = vehicle
        self.gimbal = gimbal

    def update(self):
        # Get vehicle IMU data
        vehicle_rotation = self.vehicle.get_angular_velocity()
        vehicle_attitude = self.vehicle.get_world_pose()[1]

        # Calculate gimbal compensation
        # Roll: Compensate vehicle roll to keep horizon level
        # Pitch: Maintain desired angle (e.g., downward)
        # Yaw: Follow vehicle heading or maintain fixed direction

        vehicle_euler = self.quat_to_euler(vehicle_attitude)

        gimbal_roll = -vehicle_euler[0]  # Compensate vehicle roll
        gimbal_pitch = -90  # Fixed downward
        gimbal_yaw = 0  # Follow vehicle or fixed

        self.send_gimbal_command(gimbal_pitch, gimbal_roll, gimbal_yaw)
```

## PX4 Configuration

### Parameters

Set these PX4 parameters for gimbal control:

```bash
# Enable MAVLink gimbal protocol v2
param set MNT_MODE_IN 4    # MAVLink gimbal protocol v2
param set MNT_MODE_OUT 2   # MAVLink gimbal protocol v2

# Gimbal limits (degrees)
param set MNT_RANGE_PITCH 180
param set MNT_RANGE_ROLL 90
param set MNT_RANGE_YAW 360

# Mount point configuration
param set MNT_MAN_PITCH 1   # Enable manual pitch control
param set MNT_MAN_ROLL 1    # Enable manual roll control
param set MNT_MAN_YAW 1     # Enable manual yaw control
```

### MAVLink Configuration

```bash
# Configure MAVLink for gimbal (example using TELEM2)
param set MAV_1_CONFIG 102   # TELEM2
param set MAV_1_MODE 2       # Onboard
param set MAV_1_RATE 100000  # 100kbps
param set MAV_1_FORWARD 1    # Enable message forwarding
```

## Integration Example

### Complete Gimbal Control Example

```python
import asyncio
from mavsdk import System
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

class GimbalControlExample:
    def __init__(self):
        self.drone = System()
        self.pg = PegasusInterface()
        self.gimbal_mode = "downward"  # "downward", "follow", "manual"

    async def initialize(self):
        # Connect to PX4
        await self.drone.connect(system_address="udp://:14540")

        # Wait for connection
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break

    async def run_gimbal_control(self):
        while True:
            if self.gimbal_mode == "downward":
                # Always point downward
                await self.drone.gimbal.set_pitch_and_yaw(-90, 0)

            elif self.gimbal_mode == "follow":
                # Follow vehicle heading, point downward
                vehicle_heading = await self.get_vehicle_heading()
                await self.drone.gimbal.set_pitch_and_yaw(-90, vehicle_heading)

            elif self.gimbal_mode == "manual":
                # Manual control from ground station
                pass

            await asyncio.sleep(0.1)  # 10 Hz update rate

    async def get_vehicle_heading(self):
        async for attitude in self.drone.telemetry.attitude_euler():
            return attitude.yaw_deg
```

## Testing and Validation

### Verify Gimbal Control

1. **Check MAVLink messages**: Use QGroundControl or mavlink router logs
2. **Monitor gimbal status**: Watch `gimbal_device_attitude_status` messages
3. **Test control authority**: Switch between manual and automatic control
4. **Validate constraints**: Ensure gimbal respects joint limits

### Common Issues

- **Gimbal not responding**: Check MAVLink connection and component IDs
- **Wrong orientation**: Verify coordinate frame transformations
- **Jerky movement**: Implement rate limiting and smoothing
- **Joint limits**: Ensure gimbal commands respect physical constraints

## Real Vehicle Deployment

The same control code works on real vehicles with:

1. **Compatible gimbals**: Storm32, SimpleBGC, Gremsy, etc.
2. **MAVLink connection**: Serial or CAN bus
3. **Component ID setup**: Configure unique gimbal component ID
4. **Parameter tuning**: Adjust PID gains for your specific gimbal

## Advanced Features

### Object Tracking
```python
# Track detected objects
async def track_object(self, object_position):
    # Calculate look-at angles
    vehicle_pos = await self.get_vehicle_position()
    look_vector = object_position - vehicle_pos

    # Convert to gimbal angles
    pitch, yaw = self.vector_to_angles(look_vector)
    await self.drone.gimbal.set_pitch_and_yaw(pitch, yaw)
```

### Waypoint Camera Control
```python
# Automatic camera pointing during missions
async def mission_camera_control(self, waypoint):
    if waypoint.camera_action == "survey":
        await self.drone.gimbal.set_pitch_and_yaw(-90, 0)  # Nadir
    elif waypoint.camera_action == "inspect":
        await self.drone.gimbal.set_pitch_and_yaw(-45, waypoint.heading)
```

For more advanced camera features, refer to the [Isaac Sim Camera Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_camera.html).