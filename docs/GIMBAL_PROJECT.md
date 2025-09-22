# Gimbal Project - Multi-Sensor Gimbal System for EmissarePegasusSimulator

## Project Overview

This project implements a programmatic vehicle generation system with multi-sensor gimbal support for the EmissarePegasusSimulator extension. The goal is to create a streamlined, YAML-based configuration system that eliminates the need for USD asset files while providing flexible gimbal control for software pipeline testing.

## Architecture Goals

### Simplicity First
- **No USD dependencies**: All vehicles created programmatically from YAML (USD support removed)
- **YAML configuration**: Single source of truth for all vehicle/gimbal parameters
- **Predefined components**: Standard motors, symmetric airframes, consistent patterns
- **Manual testing**: Visual verification through Isaac Sim UI, no automated testing complexity

### Focus on Software Pipelines
- **Standardized platforms**: Consistent quadrotor_x geometry for testing
- **Multi-sensor gimbals**: Color camera, monochrome camera, laser rangefinder
- **Real MAVLink integration**: Same protocol used for real and simulated gimbals
- **UI controls**: Manual gimbal control panel for interactive testing

## Implementation Strategy

### Phase 1: Configuration System ✅
- [x] Project documentation (this file)
- [x] Motor database with predefined parameters
- [x] Vehicle configuration templates
- [x] Gimbal configuration with multi-sensor support

### Phase 2: Core Vehicle Implementation ✅
- [x] Multirotor class now uses YAML-based vehicle creation (no USD files)
- [x] Modified Vehicle base class supporting no-USD mode
- [x] Quadrotor_X structure with standard motor rotation pattern
- [x] Removed all USD vehicle dependencies
- [x] Removed MultirotorConfig class (simplified to YAML-only)
- [x] Simplified vehicle cleanup for single-vehicle scenarios

### Phase 3: Gimbal System ✅
- [x] GimbalSystem class with 3-axis articulation
- [x] LaserRangefinder sensor implementation
- [x] Multi-sensor attachment and management

### Phase 4: UI Integration ✅
- [x] Gimbal control panel in extension UI
- [x] MAVLink command integration
- [x] Manual control sliders and preset buttons

## Directory Structure

```
EmissarePegasusSimulator/
├── docs/
│   └── GIMBAL_PROJECT.md                    # This file
├── extensions/pegasus.simulator/
│   ├── config/
│   │   ├── vehicles/
│   │   │   ├── quadrotor.yaml              # Basic quadrotor
│   │   │   └── quadrotor_with_gimbal.yaml  # Quadrotor with gimbal
│   │   ├── gimbals/
│   │   │   └── multi_sensor_gimbal.yaml    # Multi-sensor gimbal config
│   │   └── motors/
│   │       └── motor_database.yaml         # Predefined motor library
│   └── pegasus/simulator/
│       └── logic/
│           ├── vehicles/
│           │   ├── multirotor.py            # UPDATED: YAML-based vehicle creation
│           │   └── vehicle.py               # MODIFIED: Support no-USD mode
│           ├── graphical_sensors/
│           │   ├── gimbal_system.py        # NEW: Gimbal implementation
│           │   └── laser_rangefinder.py    # NEW: Laser sensor
│           └── ui/
│               └── ui_window.py             # MODIFIED: Add gimbal controls
```

## Technical Specifications

### Vehicle Configuration

**Quadrotor X Layout:**
- Rotor positions: Diagonal arrangement (45° from forward axis)
- Motor rotation: [-1, -1, 1, 1] (matching current Iris configuration)
- Symmetric geometry: Single separation parameter defines all rotor positions
- Configurable mass and motor selection

**Supported Parameters:**
```yaml
vehicle:
  type: "quadrotor_x"          # Only type supported initially
  rotor_separation: 0.3        # meters (diagonal motor-to-motor distance)
  mass: 1.5                    # kg
  motor: "T_Motor_F40_2400KV"  # Reference to motor database
```

### Gimbal Configuration

**3-Axis Gimbal:**
- Yaw joint: ±180° (Z-axis rotation)
- Pitch joint: -135° to +45° (Y-axis rotation)
- Roll joint: ±45° (X-axis rotation)
- Configurable rate limits for each axis

**Multi-Sensor Support:**
- Color camera: High-resolution imaging
- Monochrome camera: Lower resolution, higher frequency
- Laser rangefinder: Distance measurement

### Motor Database

**Predefined Motors:**
- T-Motor F40 2400KV: Standard racing/freestyle motor
- E-Max RS2205 2300KV: Popular multirotor motor
- Generic options: Small, medium, large configurations

**Motor Parameters:**
- Rotor constant: Thrust coefficient
- Rolling moment coefficient: Torque generation
- Maximum rotor velocity: RPM limits
- Rotor radius: Propeller size

## Development Notes

### Isaac Sim Integration

**Programmatic Creation:**
- Use `DynamicCuboid` for vehicle body (physics-enabled)
- Use `DynamicCylinder` for rotors with `RevoluteJoint`
- Create gimbal structure with nested revolute joints
- Attach sensors to gimbal end-effector

**Physics Configuration:**
- Proper mass distribution for realistic flight dynamics
- Joint limits and damping for stable gimbal operation
- Collision shapes for sensor mounting

### MAVLink Integration

**Gimbal Control:**
- Subscribe to `gimbal_device_set_attitude` messages
- Publish `gimbal_device_attitude_status` feedback
- Support standard MAVLink Gimbal Protocol v2
- Compatible with real gimbal hardware

**Command Interface:**
- Manual control via UI sliders
- Preset positions (point down, stabilize, etc.)
- Same command structure as real vehicle

## Testing Strategy

### Manual Verification
1. **Vehicle Creation**: Load YAML configs through UI, verify structure
2. **Gimbal Movement**: Use control sliders, observe joint motion
3. **Sensor Output**: Check camera feeds and rangefinder data
4. **MAVLink Flow**: Monitor messages in QGroundControl
5. **Integration**: Test with PX4 SITL and companion computer

### Success Criteria
- ✅ Vehicles load from YAML configuration only (no USD files)
- ✅ Gimbal responds to UI controls
- ✅ Sensors provide expected output
- ✅ MAVLink commands reach PX4
- ✅ Same control interface works for real hardware

## Future Extensions

### Additional Vehicle Types
- `quadrotor_plus`: Plus configuration (cardinal directions)
- `hexarotor`: Six-rotor configuration
- `fixed_wing`: Traditional airplane
- `vtol`: Vertical takeoff and landing

### Advanced Gimbal Features
- Automatic target tracking
- Horizon lock stabilization
- Waypoint-based camera control
- Object detection integration

### Sensor Expansion
- Thermal camera support
- LiDAR integration
- Multi-spectral imaging
- Spotlight control

## Progress Tracking

**Current Status**: Phase 4 Complete - Debugging vehicle loading issues
**Last Updated**: 2025-09-21
**Next Milestone**: Resolve YAML vehicle loading errors and complete testing

**Key Decisions:**
- YAML-only configuration (USD support completely removed)
- Quadrotor_X standard pattern
- Manual testing approach
- MAVLink protocol compatibility
- Multi-sensor gimbal design

---

*This project enables rapid prototyping and testing of drone software pipelines with realistic sensor configurations and control interfaces.*