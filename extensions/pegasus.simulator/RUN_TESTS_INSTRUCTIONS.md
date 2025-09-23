# How to Run Pegasus Simulator Tests

## Using Script Editor (Recommended)

1. **Open Isaac Sim**

2. **Open Script Editor**
   - Window → Script Editor
   - Or press `Alt+S`

3. **Copy the Test Script**
   - Open the file: `/home/ubuntu/workspace/EmissarePegasusSimulator/extensions/pegasus.simulator/script_editor_test_runner.py`
   - Copy the entire contents (Ctrl+A, Ctrl+C)

4. **Run in Script Editor**
   - Paste the script into Script Editor
   - Click "Run" (Ctrl+Enter)

5. **Expected Output**
```
============================================================
PEGASUS SIMULATOR TEST SUITE
Testing Coordinate Transformations and Sensor Calculations
============================================================

CRITICAL TESTS (90° Yaw Offset Fix):
----------------------------------------
✓ Identity quaternion gives yaw=0°
  → Identity quaternion gives yaw=0.0° (CORRECT!)
✓ 90° rotations
  → 90° left rotation gives yaw=-90.0°
  → 90° right rotation gives yaw=90.0°
✓ 180° rotation
  → 180° rotation gives yaw=180.0°

IMU CALCULATION TESTS:
----------------------------------------
✓ Stationary vehicle
  → Stationary IMU shows gravity=9.81 m/s²
✓ Forward acceleration
  → Forward acceleration=50.0 m/s²

GPS CALCULATION TESTS:
----------------------------------------
✓ North velocity
  → GPS North velocity=10.0 m/s
✓ West velocity
  → GPS West velocity=-10.0 m/s (negative=west)

COORDINATE TRANSFORMATION TESTS:
----------------------------------------
✓ Angular velocity FLU→FRD
  → Yaw transformation: FLU=1.0 → FRD=-1.0

============================================================
TEST SUMMARY
============================================================
Passed: 8
Failed: 0

✓ ALL TESTS PASSED!
✓ The 90° yaw offset bug is FIXED!
✓ Coordinate transformations are working correctly!
============================================================
```

## What the Tests Verify

### Critical Tests
- **Identity quaternion gives yaw=0°**: Verifies the 90° offset bug is fixed
- **90° rotations**: Confirms left/right rotations work correctly
- **180° rotation**: Validates south-facing orientation

### IMU Tests
- **Stationary vehicle**: Checks gravity compensation
- **Forward acceleration**: Validates acceleration measurements

### GPS Tests
- **North velocity**: FLU X-axis → NED North transformation
- **West velocity**: FLU Y-axis → NED East transformation (negative)

### Transformation Tests
- **Angular velocity**: FLU body → FRD body coordinate transformation

## Troubleshooting

If tests fail:

1. **Check imports**: Ensure the calculation modules exist:
   - `pegasus/simulator/logic/sensors/calculations/imu_calc.py`
   - `pegasus/simulator/logic/sensors/calculations/gps_calc.py`
   - `pegasus/simulator/logic/rotations.py`

2. **Check Isaac Sim is running**: The Script Editor only works with Isaac Sim open

3. **Check extension is loaded**: The Pegasus extension should be enabled in Extensions window

## Alternative: Run Specific Tests

If you want to run just the critical yaw test, paste this in Script Editor:

```python
import numpy as np
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.sensors.calculations.imu_calc import calculate_imu_measurements

# Test identity quaternion
result = calculate_imu_measurements(
    attitude_flu=np.array([0, 0, 0, 1]),
    angular_velocity_flu=np.zeros(3),
    linear_velocity_flu=np.zeros(3),
    prev_linear_velocity_flu=np.zeros(3),
    dt=0.01
)

rot = Rotation.from_quat(result["orientation"])
euler = rot.as_euler('ZYX', degrees=True)
yaw = euler[0]

if abs(yaw) < 1.0:
    print(f"✓ YAW FIX WORKS! Identity quaternion gives yaw={yaw:.1f}°")
else:
    print(f"✗ YAW BUG STILL EXISTS! Identity quaternion gives yaw={yaw:.1f}°")
```