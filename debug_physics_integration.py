#!/usr/bin/env python3
"""
Debug Physics Integration

This script tests the physics integration by manually creating components
and testing the apply_forces flow without needing the full simulator.
"""

import sys
import os
import numpy as np

# Add the simulator logic to the path
sys.path.append('extensions/pegasus.simulator')

try:
    from pegasus.simulator.logic.force_generators import SpinningBody
    print("✅ Successfully imported SpinningBody")
except ImportError as e:
    print(f"❌ Failed to import SpinningBody: {e}")
    sys.exit(1)

def test_component_creation():
    """Test creating SpinningBody components manually"""
    print("\n🔧 Testing SpinningBody Creation...")

    # Create a rotor component (matching multirotor setup)
    rotor = SpinningBody(
        position=np.array([0.3, 0.3, 0.1]),
        thrust_coefficient=5.84e-6,
        torque_coefficient=1.0e-6,
        spin_direction=-1  # CCW
    )

    print(f"  Created rotor: {rotor}")

    # Test force calculation
    angular_velocity = 800.0  # rad/s
    force, torque = rotor.get_force_and_torque(angular_velocity)

    print(f"  Input angular velocity: {angular_velocity} rad/s")
    print(f"  Calculated force: {force}")
    print(f"  Calculated torque: {torque}")

    # Verify non-zero output
    assert np.any(force), "Force should be non-zero"
    assert np.any(torque), "Torque should be non-zero"

    print("  ✅ Component creation and force calculation working")
    return rotor

def test_dictionary_input_format():
    """Test the dictionary input format used by the vehicle"""
    print("\n📊 Testing Dictionary Input Format...")

    # Simulate multirotor component dictionary
    components = {}

    # Create 4 rotors like in the multirotor
    positions = [
        [0.3, 0.3, 0.1],   # Front-right
        [-0.3, 0.3, 0.1],  # Front-left
        [-0.3, -0.3, 0.1], # Rear-left
        [0.3, -0.3, 0.1],  # Rear-right
    ]
    directions = [-1, -1, 1, 1]  # CCW, CCW, CW, CW

    for i, (pos, direction) in enumerate(zip(positions, directions)):
        components[i] = SpinningBody(
            position=np.array(pos),
            thrust_coefficient=5.84e-6,
            torque_coefficient=1.0e-6,
            spin_direction=direction
        )

    print(f"  Created {len(components)} components")
    print(f"  Component indices: {list(components.keys())}")

    # Simulate manual control inputs (hover condition)
    inputs = {0: 800.0, 1: 800.0, 2: 800.0, 3: 800.0}

    print(f"  Test inputs: {inputs}")

    # Process inputs like Vehicle.apply_forces() does
    total_force = np.array([0.0, 0.0, 0.0])
    total_torque = np.array([0.0, 0.0, 0.0])

    for index, input_value in inputs.items():
        if index not in components:
            print(f"    ❌ No component for index {index}")
            continue

        component = components[index]

        if isinstance(component, SpinningBody):
            force, torque = component.get_force_and_torque(input_value)
            total_force += force
            total_torque += torque

            print(f"    Rotor {index}: input={input_value}, force={force}, torque={torque}")

    print(f"  Total force: {total_force}")
    print(f"  Total torque: {total_torque}")

    # In hover, should have upward force, no net yaw torque
    assert total_force[2] > 0, "Should have upward thrust"
    assert abs(total_torque[2]) < 1e-10, "Should have no net yaw torque"

    print("  ✅ Dictionary input processing working correctly")

def test_manual_control_conversion():
    """Test manual control percentage to rad/s conversion"""
    print("\n🎮 Testing Manual Control Conversion...")

    # This matches the logic in set_manual_motor_speeds()
    motor_percentages = [50.0, 75.0, 100.0, 25.0]  # Example manual inputs
    max_speed = 1000.0  # rad/s

    manual_motor_speeds = [
        (speed / 100.0) * max_speed for speed in motor_percentages
    ]

    print(f"  Input percentages: {motor_percentages}")
    print(f"  Converted to rad/s: {manual_motor_speeds}")

    # Convert to indexed dictionary (like in update() method)
    inputs = {i: vel for i, vel in enumerate(manual_motor_speeds)}

    print(f"  Dictionary format: {inputs}")

    # Verify reasonable values
    for i, vel in enumerate(manual_motor_speeds):
        assert 0 <= vel <= max_speed, f"Velocity {vel} out of range"
        assert vel == motor_percentages[i] * 10.0, f"Conversion error for motor {i}"

    print("  ✅ Manual control conversion working correctly")

def main():
    """Run all diagnostic tests"""
    print("🚁 Physics Integration Diagnostics")
    print("=" * 50)

    try:
        test_component_creation()
        test_dictionary_input_format()
        test_manual_control_conversion()

        print("\n🎯 All diagnostic tests passed!")
        print("\n📋 Next Steps for Debugging:")
        print("1. Check if apply_forces() is being called in the actual simulator")
        print("2. Verify debug logging output when using manual controls")
        print("3. Check if joint handles are being connected properly")
        print("4. Verify the rigid body path is correct")

    except Exception as e:
        print(f"\n❌ Diagnostic test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()