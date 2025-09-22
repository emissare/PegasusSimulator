# Pegasus Simulator Extension Architecture Refactoring Plan

## Problem Statement

The Pegasus Simulator extension has accumulated technical debt through incremental fixes, resulting in:
1. Reference leaks preventing proper extension cleanup
2. Duplicate cleanup attempts causing warnings
3. Unclear lifecycle management with defensive "if exists" checks
4. Lambda functions creating circular references
5. Missing window recreation after extension re-enable

## Core Architecture Issues

### 1. Redundant Cleanup Paths
- **Window Close (X button)**: `_visibility_changed_fn` → `_destroy_window_async()` → cleans window/delegate
- **Extension Disable**: `on_shutdown()` → attempts same cleanup again
- Result: Duplicate operations, uncertainty about object states

### 2. Lambda Reference Leaks
- Gimbal control buttons use lambdas: `clicked_fn=lambda: self._set_gimbal_preset(...)`
- Creates closure references to `self` preventing garbage collection
- Cannot be explicitly cleaned up

### 3. Unclear State Management
- No tracking of what has been created/destroyed
- Defensive checks everywhere: `if self.ui_window:`, `if editor_menu:`
- Code doesn't "know" its own state

### 4. Menu Management Confusion
- Menu removed in both `_destroy_window_async()` and `on_shutdown()`
- No tracking if menu already removed
- Results in "menu not found" warnings

## Proposed Architecture

### Design Principles
1. **Single Responsibility**: Each method has one clear purpose
2. **Explicit State Tracking**: Know exactly what exists at any time
3. **No Defensive Programming**: Don't guess, know the state
4. **Clean Separation**: Extension lifecycle vs Window lifecycle

### Lifecycle Flow

```
Extension Enable (on_startup)
├── Initialize state flags
├── Create menu item (track: _menu_created = True)
├── Register workspace function
└── Show window → triggers show_window()
    ├── Create UIDelegate
    ├── Create WidgetWindow
    └── Track: _window_created = True

Window Close (X button)
├── _visibility_changed_fn called
├── Update menu checkbox
└── Schedule _destroy_window_async()
    ├── Destroy window
    ├── Cleanup delegate
    └── Track: _window_created = False

Extension Disable (on_shutdown)
├── Remove menu item (if _menu_created)
├── Destroy window (if _window_created)
├── Unregister workspace function
└── Reset all state flags
```

## Implementation Changes

### 1. extension.py - Clean Lifecycle Management

**State Tracking:**
```python
class Pegasus_SimulatorExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        # Initialize state tracking
        self._menu_created = False
        self._window_created = False
        self._workspace_registered = False
```

**Responsibilities:**
- `on_startup()`: Create menu, register workspace
- `show_window()`: Create/destroy window only
- `_destroy_window_async()`: Clean window/delegate only
- `on_shutdown()`: Clean extension-level items only

**Key Changes:**
- Remove menu operations from `_destroy_window_async()`
- Track what has been created with boolean flags
- Single cleanup path for each resource

### 2. ui_window.py - Remove All Lambdas

**Replace Lambda Callbacks:**
```python
# BAD - Current implementation
clicked_fn=lambda: self._set_gimbal_preset(-90, 0, 0)

# GOOD - New implementation
clicked_fn=self._on_gimbal_point_down_clicked
```

**Add Proper Methods:**
```python
def _on_gimbal_point_down_clicked(self):
    self._set_gimbal_preset(-90, 0, 0)

def _on_gimbal_center_clicked(self):
    self._set_gimbal_preset(0, 0, 0)
```

**Track UI Elements for Cleanup:**
```python
def _gimbal_control_frame(self):
    # Store button references
    self._gimbal_buttons = []

    button = ui.Button("Point Down", clicked_fn=self._on_gimbal_point_down_clicked)
    self._gimbal_buttons.append(button)
```

**Enhanced Cleanup:**
```python
def destroy(self):
    # Clear button references explicitly
    self._gimbal_buttons.clear()

    # Unsubscribe all callbacks
    for subscription in self._ui_subscriptions:
        if subscription:
            subscription.unsubscribe()

    # Clear all UI element references
    self._gimbal_enabled_checkbox = None
    self._gimbal_pitch_slider = None
    # ... etc
```

### 3. ui_delegate.py - Improved State Management

**Add Singleton Reset:**
```python
def cleanup(self):
    # Clear all references
    self._window = None
    self._scene_dropdown = None
    # ... clear all fields

    # Reset PegasusInterface singleton if needed
    if hasattr(PegasusInterface, '_instance'):
        PegasusInterface._instance = None
```

### 4. PegasusInterface Singleton Management

**Add cleanup support:**
```python
class PegasusInterface:
    @classmethod
    def reset_instance(cls):
        """Reset the singleton instance during extension cleanup"""
        with cls._lock:
            if hasattr(cls, '_instance'):
                cls._instance = None
```

## Testing Plan

1. **Enable Extension**
   - Verify menu appears
   - Verify window opens
   - Check all UI elements functional

2. **Close Window (X button)**
   - Verify window closes
   - Verify menu checkbox unchecked
   - Verify can reopen from menu

3. **Disable Extension**
   - Verify no warnings in console
   - Verify menu removed
   - Verify all resources cleaned

4. **Re-enable Extension**
   - Verify menu recreated
   - Verify window appears
   - Verify no reference leak warnings

## Success Criteria

✓ No "menu not found" warnings
✓ No "extension object still alive" warnings
✓ Window reappears on extension re-enable
✓ Clean enable/disable cycle
✓ No defensive "if exists" checks
✓ Clear, predictable lifecycle

## Implementation Order

1. Create this document ✓
2. Refactor extension.py with state tracking
3. Remove all lambdas from ui_window.py
4. Improve cleanup in ui_delegate.py
5. Test complete enable/disable cycle

## Code Quality Improvements

- Remove all defensive programming patterns
- Add clear lifecycle documentation
- Use consistent naming conventions
- Add proper type hints
- Remove commented old code
- Add debug logging for lifecycle events

## Long-term Maintenance

- Document the lifecycle clearly in code comments
- Add unit tests for lifecycle management
- Create developer guide for extension patterns
- Regular code reviews to prevent regression