#!/usr/bin/env python3
"""
Test script to validate the physics architecture without Isaac Sim.
This tests the force generator setup logic and physics calculations.
"""

import numpy as np
import sys
sys.path.append('extensions/pegasus.simulator')

# Import core physics classes
from pegasus.simulator.logic.force_generators import ForceGenerator, SpinningBody, LiftingSurface

def test_spinning_body():
    """Test SpinningBody force and torque calculations"""
    print("Testing SpinningBody physics...")

    # Create a rotor at front-right position
    rotor = SpinningBody(
        position=np.array([0.3, 0.3, 0.1]),  # 30cm from center, 10cm up
        thrust_coefficient=5.84e-6,          # Typical motor thrust coeff
        torque_coefficient=1.0e-6,           # Typical motor torque coeff
        spin_direction=-1                    # Counter-clockwise
    )

    # Test at 800 rad/s (typical flight RPM)
    angular_velocity = 800.0
    force, torque = rotor.get_force_and_torque(angular_velocity)

    print(f"  Angular velocity: {angular_velocity} rad/s")
    print(f"  Force (N): {force}")
    print(f"  Torque (N⋅m): {torque}")

    # Verify physics
    expected_thrust = 5.84e-6 * angular_velocity**2
    expected_reaction_torque = -(1.0e-6 * angular_velocity**2 * -1)  # CCW motor

    assert abs(force[2] - expected_thrust) < 1e-10, "Thrust calculation error"
    assert abs(torque[2] - expected_reaction_torque) < 1e-10, "Torque calculation error"

    print("  ✅ SpinningBody physics correct")

def test_lifting_surface():
    """Test LiftingSurface force calculations"""
    print("Testing LiftingSurface physics...")

    # Create an aileron
    aileron = LiftingSurface(
        position=np.array([0.0, 0.5, 0.0]),  # 50cm from center on wing
        lift_coefficient=10.0,               # N per radian deflection
        lift_direction=np.array([0, 0, 1])   # Upward lift
    )

    # Test at 0.1 radian deflection (about 6 degrees)
    deflection = 0.1
    force, torque = aileron.get_force_and_torque(deflection)

    print(f"  Deflection: {deflection} rad")
    print(f"  Force (N): {force}")
    print(f"  Torque (N⋅m): {torque}")

    # Verify physics
    expected_lift = 10.0 * deflection  # 1.0 N upward

    assert abs(force[2] - expected_lift) < 1e-10, "Lift calculation error"
    assert np.allclose(torque, [0, 0, 0]), "Should have no direct torque"

    print("  ✅ LiftingSurface physics correct")

def test_quadrotor_configuration():
    """Test a complete quadrotor force generator setup"""
    print("Testing quadrotor configuration...")

    # Simulate the multirotor setup
    thrust_coeff = 5.84e-6
    torque_coeff = 1.0e-6
    separation = 0.6  # 60cm arm separation
    arm_length = separation / 2
    rotor_height = 0.1

    # X configuration
    positions = [
        [+arm_length, +arm_length, rotor_height],  # Front-right
        [-arm_length, +arm_length, rotor_height],  # Front-left
        [-arm_length, -arm_length, rotor_height],  # Rear-left
        [+arm_length, -arm_length, rotor_height],  # Rear-right
    ]
    directions = [-1, -1, 1, 1]  # CCW, CCW, CW, CW

    # Create rotors
    rotors = []
    for pos, direction in zip(positions, directions):
        rotor = SpinningBody(
            position=np.array(pos),
            thrust_coefficient=thrust_coeff,
            torque_coefficient=torque_coeff,
            spin_direction=direction
        )
        rotors.append(rotor)

    print(f"  Created {len(rotors)} rotors")
    print(f"  Positions: {[r.position.tolist() for r in rotors]}")
    print(f"  Directions: {[r.spin_direction for r in rotors]}")

    # Test hover condition (all rotors at same speed)
    hover_rpm = 800.0
    inputs = [hover_rpm] * 4

    total_force = np.array([0.0, 0.0, 0.0])
    total_torque = np.array([0.0, 0.0, 0.0])

    for rotor, input_val in zip(rotors, inputs):
        force, torque = rotor.get_force_and_torque(input_val)
        total_force += force
        total_torque += torque

    print(f"  Hover total force: {total_force}")
    print(f"  Hover total torque: {total_torque}")

    # In hover, should have only upward force, no net torque
    assert abs(total_force[0]) < 1e-10, "Should have no X force in hover"
    assert abs(total_force[1]) < 1e-10, "Should have no Y force in hover"
    assert total_force[2] > 0, "Should have upward thrust"
    assert abs(total_torque[2]) < 1e-10, "Should have no net yaw torque in hover"

    print("  ✅ Hover condition correct")

    # Test differential thrust for pitch control
    # Increase rear rotors, decrease front rotors
    pitch_inputs = [750.0, 750.0, 850.0, 850.0]  # Front lower, rear higher

    total_force = np.array([0.0, 0.0, 0.0])
    total_torque = np.array([0.0, 0.0, 0.0])

    for rotor, input_val in zip(rotors, pitch_inputs):
        force, torque = rotor.get_force_and_torque(input_val)
        total_force += force
        total_torque += torque

    print(f"  Pitch maneuver total torque: {total_torque}")

    # Should still have balanced yaw torque (CCW + CW rotors balanced)
    assert abs(total_torque[2]) < 1e-6, "Yaw torque should still be balanced"

    print("  ✅ Differential thrust physics correct")

def main():
    """Run all physics tests"""
    print("🚁 Testing Physics Architecture")
    print("=" * 50)

    test_spinning_body()
    print()

    test_lifting_surface()
    print()

    test_quadrotor_configuration()
    print()

    print("🎯 All physics tests passed!")
    print("✅ Architecture is ready for Isaac Sim testing")

if __name__ == "__main__":
    main()