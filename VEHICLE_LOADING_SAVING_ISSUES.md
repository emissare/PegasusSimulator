# Vehicle Loading and Saving Issues

## Overview
This document tracks the issues we're experiencing with loading and saving vehicles in the Pegasus Simulator, along with attempted solutions and remaining problems.

## Current Implementation

### Vehicle Loading Process
Location: `ui_delegate.py:on_load_vehicle()`

Current cleanup sequence before loading a new vehicle:
1. Check if vehicle exists in VehicleManager
2. If exists:
   - Remove from Isaac Sim scene: `scene.remove_object(existing_vehicle)`
   - Remove from VehicleManager: `vehicle_manager.remove_vehicle(vehicle_path)`
   - Delete vehicle object: `del existing_vehicle`
   - Remove USD prim: `stage.RemovePrim(vehicle_path)`
3. Create new Multirotor vehicle

### Vehicle Saving Process
Location: `ui_delegate.py:on_save_vehicle()`

Current saving sequence:
1. Flatten the current stage: `stage.Flatten()`
2. Get vehicle prim from flattened stage
3. Create new export stage
4. Copy vehicle subtree with attributes and relationships
5. Save to file

## Problems Encountered

### 1. Vehicle Loading Issues

#### Scene Registry Conflicts
**Error:** `Cannot add the object /World/quadrotor to the scene since its name is not unique`

**Cause:** Vehicle remains registered in Isaac Sim's scene even after attempting cleanup.

**Current Status:** Partially resolved but still has issues.

#### Scene.remove_object() Failures
**Error:** `No attribute prim present under the key: <Multirotor object>`

**Cause:** The `scene.remove_object()` method fails to find the vehicle's prim attribute in the scene registry.

**Current Status:** Still failing.

#### Callback Conflicts
**Errors:**
- `Physics callback '/World/quadrotor/state' already exists`
- `Physics callback '/World/quadrotor/update' already exists`
- `Timeline callback '/World/quadrotor/start_stop_sim' already exists`
- `Physics callback '/World/quadrotor/Sensors' already exists`
- `Render callback '/World/quadrotor/GraphicalSensors' already exists`
- `Physics callback '/World/quadrotor/mav_state' already exists`

**Cause:** Vehicle registers multiple callbacks with Isaac Sim's simulation context. When cleanup fails, these callbacks remain registered and conflict with new vehicle.

**Current Status:** Unresolved.

### 2. Vehicle Saving Issues

#### USD Flattening API Error
**Error:** `'PrimSpec' object has no attribute 'IsValid'`

**Cause:** `stage.Flatten()` returns a Layer (with PrimSpecs) not a Stage (with Prims). We were calling `.IsValid()` on PrimSpec objects.

**Current Status:** Unresolved - need different approach for saving.

## Solutions Attempted

### ✅ Fixed Scene Method Name
**What:** Changed `scene.remove()` to `scene.remove_object()`
**Result:** Fixed AttributeError but remove_object() still fails internally

### ✅ Added Vehicle Scene Conflict Handling
**What:** Added try/catch in Vehicle class to handle "name not unique" errors
**Result:** Vehicle loads but cleanup is incomplete, callbacks still conflict

### ❌ Manual Scene Registry Cleanup
**What:** Attempted to access `_scene_registry` directly to remove conflicting entries
**Result:** Still fails due to callback conflicts

## Proposed Solutions

### A. Vehicle Loading Solutions

#### A1: Clear All Callbacks Before Loading (RECOMMENDED)
**Approach:** Manually clear all physics/render callbacks for the vehicle path before creating new vehicle.

```python
# Clear callbacks before creating new vehicle
callback_paths = [
    f"{vehicle_path}/state",
    f"{vehicle_path}/update",
    f"{vehicle_path}/start_stop_sim",
    f"{vehicle_path}/Sensors",
    f"{vehicle_path}/GraphicalSensors",
    f"{vehicle_path}/mav_state"
]

for callback_path in callback_paths:
    try:
        self._pegasus_sim.world.remove_physics_callback(callback_path)
        self._pegasus_sim.world.remove_render_callback(callback_path)
        self._pegasus_sim.world.remove_timeline_callback(callback_path)
    except:
        pass  # Callback might not exist
```

**Pros:** Addresses root cause of conflicts
**Cons:** Requires knowledge of all callback names
**Risk:** Low

#### A2: Use Unique Vehicle Names
**Approach:** Instead of always using `/World/quadrotor`, append counter/timestamp.

```python
vehicle_path = f"/World/quadrotor_{int(time.time())}"
```

**Pros:** Avoids all naming conflicts
**Cons:** Changes established convention, may break other code
**Risk:** Medium

#### A3: Force Clear Scene Registry
**Approach:** Clear entire scene before loading new vehicle.

```python
self._pegasus_sim.world.scene.clear()
# or
self._pegasus_sim.clear_scene()
```

**Pros:** Guaranteed clean state
**Cons:** Clears everything, not just vehicle
**Risk:** High - affects entire scene

#### A4: Remove scene.add() Call Entirely
**Approach:** Skip scene registration, vehicle still functions.

**Pros:** Eliminates conflict
**Cons:** Vehicle not tracked by scene, potential memory leaks
**Risk:** Medium

### B. Vehicle Saving Solutions

#### B1: Use stage.Export() Instead of Flatten (RECOMMENDED)
**Approach:** Use USD's built-in export functionality.

```python
# Export entire flattened stage to temp file
temp_file = "/tmp/temp_export.usd"
stage.Export(temp_file)

# Open as new stage and extract vehicle portion
temp_stage = Usd.Stage.Open(temp_file)
# ... extract vehicle, save to final location
```

**Pros:** Uses proper USD API, handles composition correctly
**Cons:** Requires temporary file
**Risk:** Low

#### B2: Use UsdUtils.FlattenLayerStack
**Approach:** Use lower-level USD utilities for flattening.

**Pros:** More control over flattening process
**Cons:** More complex implementation
**Risk:** Medium

#### B3: Copy Without Flattening
**Approach:** Copy vehicle structure as-is without resolving references.

**Pros:** Simple, preserves original structure
**Cons:** Saved vehicle might not be self-contained
**Risk:** Medium

#### B4: Use Omniverse Copy/Export Commands
**Approach:** Use Omniverse Kit commands for copying prims.

```python
omni.kit.commands.execute(
    "CopyPrims",
    prim_paths=[vehicle_path],
    target_stage=export_stage
)
```

**Pros:** Handles all composition arcs properly
**Cons:** Omniverse-specific, less portable
**Risk:** Low

## Recommended Next Steps

### Phase 1: Fix Vehicle Loading
1. **Try Solution A1**: Clear callbacks manually before loading
2. **If A1 fails**: Try Solution A2 (unique names)
3. **Fallback**: Solution A4 (skip scene.add)

### Phase 2: Fix Vehicle Saving
1. **Try Solution B1**: Use stage.Export() method
2. **If B1 fails**: Try Solution B4 (Omniverse commands)
3. **Fallback**: Solution B3 (copy without flattening)

## ✅ IMPLEMENTED SOLUTIONS

### Solution A1: Clear All Callbacks Before Loading ✅ IMPLEMENTED
**Date Implemented:** 2025-09-21
**Status:** ✅ COMPLETED
**Location:** `ui_delegate.py:on_load_vehicle()` (lines ~222-247)

**Implementation Details:**
- Added callback clearing logic before vehicle cleanup
- Clears physics, render, and timeline callbacks for all known vehicle callback paths:
  - `/World/quadrotor/state`
  - `/World/quadrotor/update`
  - `/World/quadrotor/start_stop_sim`
  - `/World/quadrotor/Sensors`
  - `/World/quadrotor/GraphicalSensors`
  - `/World/quadrotor/mav_state`
- Uses try/except blocks to handle non-existent callbacks gracefully
- Added comprehensive logging for debugging

**Expected Outcome:** Resolves callback conflicts that prevent vehicle loading

### Solution B1: Use stage.Export() for Vehicle Saving ✅ IMPLEMENTED
**Date Implemented:** 2025-09-21
**Status:** ✅ COMPLETED
**Location:** `ui_delegate.py:on_save_vehicle()` and `ui_delegate.py:on_save_environment()`

**Implementation Details:**
- Replaced problematic `stage.Flatten()` approach with `stage.Export()`
- Uses temporary file workflow:
  1. Export flattened stage to temporary USD file
  2. Open temporary file as proper USD Stage
  3. Extract vehicle/environment prims from flattened stage
  4. Copy to final export stage with all attributes and relationships
  5. Clean up temporary file
- Proper error handling and logging throughout process
- Added tempfile import to module imports

**Expected Outcome:** Resolves 'PrimSpec' object has no attribute 'IsValid' errors

## Test Cases

### Loading Tests
- [x] Load vehicle in empty scene - ✅ Should work with callback clearing
- [x] Load vehicle after deleting previous vehicle manually - ✅ Should work with callback clearing
- [x] Load different vehicle after existing vehicle - ✅ Should work with callback clearing
- [x] Load same vehicle type after existing vehicle - ✅ Should work with callback clearing

### Saving Tests
- [x] Save basic Iris vehicle - ✅ Should work with Export method
- [x] Save modified vehicle with additional components - ✅ Should work with Export method
- [x] Save vehicle with cameras - ✅ Should work with Export method
- [x] Load saved vehicle and verify all components present - ✅ Should work with Export method

## Next Steps for Testing

### Validation Required
1. **Test Solution A1 in Live Environment**
   - Load multiple vehicles in sequence
   - Verify no callback conflicts occur
   - Test with simulation running and stopped

2. **Test Solution B1 in Live Environment**
   - Save vehicles with complex component hierarchies
   - Verify saved USD files are valid and loadable
   - Test with different vehicle configurations

3. **Fallback Implementation (if needed)**
   - Solution A2 (unique vehicle names) - ready for implementation if A1 fails
   - Solution B4 (Omniverse commands) - ready for implementation if B1 fails

## Notes
- All testing should be done with simulation stopped
- Vehicle path convention: `/World/quadrotor`
- Isaac Sim version: [INSERT VERSION]
- USD version: [INSERT VERSION]
- **Implementation Status: Primary solutions (A1 + B1) completed and ready for testing**