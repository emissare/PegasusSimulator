# Custom Vehicle Loading in Forked Pegasus

## Overview

This document describes the implementation of separate environment and vehicle loading/saving functionality in the forked Pegasus Simulator. The goal is to provide complete flexibility for working with environments and vehicles independently while maintaining clean asset separation.

## Current Limitation

### Existing Workflow
1. User clicks "Load Scene" → Environment loads without vehicles
2. User must separately click "Load Vehicle" → Choose from dropdown and spawn vehicle
3. No ability to save environments and vehicles separately
4. Risk of accidentally overwriting complex scenes when saving vehicle modifications

### Problem
- Cannot work on environments without vehicles interfering
- Cannot work on vehicles without complex environment loading
- No clean separation between environment and vehicle assets
- Risk of saving mistakes when working on specific components

## Goal

Modify the Pegasus UI to provide independent environment and vehicle operations:
- Load environments independently for scene development
- Load vehicles independently for vehicle testing
- Save environments without vehicle data
- Save vehicles without environment data
- Maintain clear asset separation using path-based identification

## Current Architecture Analysis

### Key Files and Components

#### 1. Vehicle Definitions (`pegasus/simulator/params.py`)
```python
ROBOTS = {
    "Iris": "Iris/iris.usd",
    "Crazyflie": "Crazyflie/crazyflie.usd",
    # ... other vehicles
}
```
- Defines available vehicles as dictionary
- Maps display names to USD file paths
- Used by vehicle dropdown in current UI

#### 2. Scene Loading UI (`pegasus/simulator/ui/ui_window.py`)
- Contains "Load Scene" button
- Currently only has environment dropdown
- No vehicle selection component for scene loading

#### 3. Scene Loading Logic (`pegasus/simulator/ui/ui_delegate.py`)
```python
def on_load_scene(self):
    # Current implementation only loads environment
    # No vehicle spawning logic
```

#### 4. Vehicle Loading Logic (`pegasus/simulator/ui/ui_delegate.py`)
```python
def on_load_vehicle(self):
    # Separate function for vehicle spawning
    # Contains vehicle selection and configuration
```

## Implementation Plan

### Phase 1: UI Modifications

#### A. Separate Environment and Vehicle Controls
**File**: `pegasus/simulator/ui/ui_window.py`

1. **Modify scene selection frame** (lines 111-196):
   - Rename to "Environment Selection"
   - Add "None" option to environment dropdown
   - Replace "Load Scene" with "Load Environment"
   - Add "Save Environment" button

2. **Modify vehicle selection frame** (lines 197-246):
   - Keep existing vehicle dropdown and settings
   - Keep "Load Vehicle" button
   - Add "Save Vehicle" button

3. **UI Layout**:
   ```python
   [Environment Selection]
     Dropdown: [None | Default Environment | Warehouse | ...]
     [Load Environment] [Save Environment]

   [Vehicle Selection]
     Dropdown: [Iris | Crazyflie | ...]
     Position/Orientation controls
     [Load Vehicle] [Save Vehicle]
   ```

#### B. Vehicle Identification Strategy
**Path-based identification**: Vehicle always at `/World/quadrotor`
- Simple, predictable
- Follows existing Pegasus convention
- No USD modifications needed
- Single vehicle constraint enforced

### Phase 2: Logic Implementation

#### A. Environment Operations
**File**: `pegasus/simulator/ui/ui_delegate.py`

```python
def on_load_environment(self):
    """Load only environment, no vehicle"""
    env_index = self._scene_dropdown.get_item_value_model().as_int
    if env_index > 0:  # Not "None"
        selected_world = self._scene_names[env_index - 1]
        await self._pegasus_sim.load_environment_async(...)

def on_save_environment(self):
    """Save everything EXCEPT /World/quadrotor"""
    stage = omni.usd.get_context().get_stage()
    # Filter out vehicle path when saving
```

#### B. Vehicle Operations
```python
def on_save_vehicle(self):
    """Save ONLY /World/quadrotor"""
    vehicle_prim = stage.GetPrimAtPath("/World/quadrotor")
    if not vehicle_prim:
        carb.log_error("No vehicle found")
        return
    # Export vehicle prim tree to USD
```

### Phase 3: Configuration Management

#### A. Support Custom Vehicle Library
**File**: `pegasus/simulator/params.py`

1. **Extend ROBOTS dictionary loading**:
   - Scan for user-defined vehicle files
   - Load custom vehicle configurations
   - Merge with built-in vehicles

2. **Custom vehicle directory structure**:
   ```
   PegasusSimulator/
   ├── assets/robots/        # Built-in vehicles
   └── custom_vehicles/      # User vehicles
       ├── my_quadrotor_1/
       │   ├── model.usd
       │   └── config.json
       └── my_quadrotor_2/
           ├── model.usd
           └── config.json
   ```

#### B. Vehicle Configuration Schema
**File**: `custom_vehicles/config.json`

```json
{
  "name": "My Custom Quadrotor",
  "usd_file": "model.usd",
  "description": "Quadrotor with custom sensor suite",
  "default_sensors": ["camera", "imu", "gps"],
  "spawn_height": 0.5
}
```

### Phase 4: Advanced Features

#### A. Scene-Vehicle Presets
- Save common scene+vehicle combinations
- Quick-load buttons for favorite setups
- User-defined presets file

#### B. Vehicle Validation
- Check USD file exists before showing in dropdown
- Validate vehicle compatibility with selected environment
- Error handling for missing or corrupt vehicle files

## Implementation Steps

### Step 1: Basic UI Integration
1. Fork Pegasus repository
2. Add vehicle dropdown to scene loading UI
3. Connect dropdown to delegate
4. Test with existing vehicles

### Step 2: Logic Integration
1. Modify `on_load_scene()` to check vehicle selection
2. Integrate vehicle spawning with scene loading
3. Test scene+vehicle loading workflow

### Step 3: Custom Vehicle Support
1. Implement custom vehicle directory scanning
2. Add vehicle configuration loading
3. Test with user-defined vehicles

### Step 4: Polish and Testing
1. Add error handling
2. Improve user feedback
3. Test edge cases
4. Update documentation

## Benefits

1. **Complete Flexibility**: Load/save environments and vehicles independently
2. **Clean Asset Separation**: No risk of accidentally overwriting wrong components
3. **Development Workflow**: Environment designers can work without vehicles
4. **Vehicle Testing**: Vehicle developers can test in minimal environments
5. **Safe Operations**: Path-based identification prevents ambiguity
6. **Backward Compatibility**: Existing workflows still work

## Files to Modify

1. **`pegasus/simulator/ui/ui_window.py`** - Add separate load/save buttons for environment and vehicle
2. **`pegasus/simulator/ui/ui_delegate.py`** - Implement separate load/save methods with path-based filtering
3. **Documentation** - Update user guide with new workflow

## Vehicle Identification

**Path-based Strategy**: Vehicle always at `/World/quadrotor`
- Single vehicle constraint ensures no ambiguity
- Follows existing Pegasus conventions
- Simple implementation with predictable behavior
- Save operations filter by path prefix

## Workflow Examples

**Environment Development**:
1. Load Environment → Work on scene → Save Environment
2. No vehicle interference

**Vehicle Development**:
1. Load Vehicle (into empty scene or existing environment) → Modify → Save Vehicle
2. Vehicle changes isolated from environment

**Testing**:
1. Load Environment → Load Vehicle → Test → Save separately if needed

## Future Enhancements

- Custom vehicle library support
- Environment-vehicle preset combinations
- Multi-vehicle scenarios (future consideration)
- Integration with external asset libraries

---

*This document will be updated as the implementation progresses.*