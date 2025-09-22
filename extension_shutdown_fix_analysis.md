# Extension Shutdown Warning Analysis and Fix

## Issue Summary

When disabling the EmissarePegasusSimulator extension, several warnings were occurring that indicated improper cleanup and reference leaks:

1. **Reference Leak Warning**: Extension object still alive after shutdown
2. **UI Subscription Warnings**: 18 UI subscriptions with None values or missing unsubscribe methods
3. **Menu Item Warning**: Attempting to remove non-existent menu items

## Root Cause Analysis

### 1. Primary Issue: Reference Leak
**Warning**: `extension object is still alive, something holds a reference on it. References: ["[0]:type: <class 'method'>, id: 131049463927168"]`

**Root Cause**: Line 58 in `extension.py` used `partial(self.show_window, None)` which created a strong reference to the extension instance (`self`). This prevented the Python garbage collector from cleaning up the extension object even after shutdown.

```python
# PROBLEMATIC CODE:
ui.Workspace.set_show_window_fn(WINDOW_TITLE, partial(self.show_window, None))
```

The `functools.partial` object held a reference to `self.show_window`, which in turn held a reference to the entire extension instance.

### 2. UI Subscription Management Issues
**Warning**: Multiple "UI subscription X is None or missing unsubscribe method" warnings

**Root Causes**:
- **Variable Name Reuse**: Subscription variables (`subscription`, `subscription1-6`) were being reused in loops and different sections, causing earlier subscription references to be overwritten
- **Missing Subscriptions**: The motor control loop created 8 subscriptions (4 motors × 2 controls each) but variable reuse meant only the last ones were properly tracked
- **Weak Validation**: The cleanup code didn't properly validate subscription objects before attempting to unsubscribe

### 3. Menu Item Removal
**Warning**: `remove_item menu Window/Pegasus Simulator not found`

**Root Cause**: The code attempted to remove menu items without checking if they existed first, likely due to timing issues or previous cleanup attempts.

## Solutions Implemented

### 1. Fixed Reference Leak with Weak References

**File**: `extension.py`

```python
# Added import
import weakref

# Replaced problematic line with:
ui.Workspace.set_show_window_fn(WINDOW_TITLE, self._create_show_window_callback())

# Added new method:
def _create_show_window_callback(self):
    """
    Create a weakref-based callback to avoid reference leaks
    """
    weak_self = weakref.ref(self)

    def show_window_callback(show):
        strong_self = weak_self()
        if strong_self is not None:
            strong_self.show_window(None, show)

    return show_window_callback
```

**Why this works**: The weak reference allows the callback to access the extension instance when needed, but doesn't prevent garbage collection. If the extension is destroyed, `weak_self()` returns `None` and the callback safely exits.

### 2. Fixed UI Subscription Tracking

**File**: `ui_window.py`

**Problem**: Variable reuse was causing subscription references to be lost:
```python
# PROBLEMATIC PATTERN:
subscription1 = slider.model.add_value_changed_fn(...)
self._ui_subscriptions.append(subscription1)
subscription1 = value.model.add_value_changed_fn(...)  # Overwrites previous reference!
self._ui_subscriptions.append(subscription1)
```

**Solution**: Eliminated intermediate variables and directly appended subscriptions:
```python
# FIXED PATTERN:
self._ui_subscriptions.append(
    slider.model.add_value_changed_fn(...)
)
self._ui_subscriptions.append(
    value.model.add_value_changed_fn(...)
)
```

**Subscription Count Verification**:
- 1 gimbal enabled checkbox
- 6 gimbal controls (3 sliders + 3 values for pitch/roll/yaw)
- 1 motor control enabled checkbox
- 8 motor controls (4 motors × 2 controls each)
- 2 master throttle controls (1 slider + 1 value)
- **Total: 18 subscriptions** ✓

### 3. Enhanced Subscription Cleanup Validation

**File**: `ui_window.py`

```python
# Improved validation before unsubscribing:
for i, subscription in enumerate(self._ui_subscriptions):
    try:
        if subscription is not None:
            if hasattr(subscription, 'unsubscribe') and callable(getattr(subscription, 'unsubscribe')):
                subscription.unsubscribe()
                carb.log_info(f"Successfully unsubscribed UI callback {i}")
            else:
                carb.log_warn(f"UI subscription {i} has no unsubscribe method")
        else:
            carb.log_warn(f"UI subscription {i} is None")
    except Exception as e:
        carb.log_error(f"Failed to unsubscribe UI callback {i}: {str(e)}")
```

### 4. Fixed Menu Item Removal

**File**: `extension.py`

```python
# Added existence check before removal:
if editor_menu.get_menu_item(MENU_PATH) is not None:
    carb.log_info(f"Removing editor menu item: {MENU_PATH}")
    editor_menu.remove_item(MENU_PATH)
    carb.log_info("Editor menu item removed successfully")
else:
    carb.log_info(f"Menu item {MENU_PATH} not found, skipping removal")
```

### 5. Added Cleanup Coordination

**File**: `ui_delegate.py`

```python
# Added cleanup flag to prevent duplicate calls:
def __init__(self):
    # ... existing code ...
    self._cleaned_up = False

def cleanup(self):
    if self._cleaned_up:
        carb.log_info("UIDelegate cleanup already completed, skipping")
        return

    carb.log_info("UIDelegate cleanup started")
    self._cleaned_up = True
    # ... rest of cleanup ...
```

## Expected Results

After these changes, the extension should:

1. **No Reference Leaks**: Extension object properly garbage collected after shutdown
2. **Clean UI Subscription Cleanup**: All 18 subscriptions properly unsubscribed without warnings
3. **Safe Menu Management**: Menu items checked for existence before removal
4. **No Duplicate Cleanup**: UIDelegate cleanup only runs once per instance

## Technical Notes

- **Weak References**: Used to break circular references while maintaining functionality
- **Direct List Appending**: Eliminates variable reuse issues that caused subscription tracking problems
- **Defensive Programming**: Added validation and error handling throughout cleanup processes
- **State Management**: Added flags to prevent duplicate operations during shutdown

## Build Information

- **Fixed in Build**: 2025.09.21-v6
- **Files Modified**:
  - `pegasus/simulator/extension.py`
  - `pegasus/simulator/ui/ui_window.py`
  - `pegasus/simulator/ui/ui_delegate.py`