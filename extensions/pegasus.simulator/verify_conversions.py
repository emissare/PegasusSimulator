"""
Verify coordinate conversions step by step
"""

import numpy as np

print("\n" + "="*60)
print("COORDINATE SYSTEM VERIFICATION")
print("="*60)

print("\n1. FLU (Isaac Sim):")
print("   X = Front (we call North)")
print("   Y = Left (we call West)")
print("   Z = Up")

print("\n2. NED (PX4):")
print("   X = North")
print("   Y = East")
print("   Z = Down")

print("\n3. ENU (Geographic/geodetic conversion):")
print("   X = East")
print("   Y = North")
print("   Z = Up")

print("\n" + "-"*40)
print("CONVERSION LOGIC:")
print("-"*40)

print("\nFLU to NED (180° rotation around X):")
print("   NED_x = FLU_x  (North = Front)")
print("   NED_y = -FLU_y (East = -Left)")
print("   NED_z = -FLU_z (Down = -Up)")

print("\nNED to ENU:")
print("   ENU_x = NED_y  (East = East)")
print("   ENU_y = NED_x  (North = North)")
print("   ENU_z = -NED_z (Up = -Down)")

print("\n" + "-"*40)
print("COMBINED FLU to ENU:")
print("-"*40)

print("\nSubstituting NED expressions into ENU:")
print("   ENU_x = NED_y = -FLU_y")
print("   ENU_y = NED_x = FLU_x")
print("   ENU_z = -NED_z = -(-FLU_z) = FLU_z")

print("\nSo FLU to ENU should be:")
print("   ENU = [-FLU_y, FLU_x, FLU_z]")

print("\n" + "-"*40)
print("TEST CASES:")
print("-"*40)

# Test case 1: North movement
flu_north = np.array([1000, 0, 0])
enu_north = np.array([-flu_north[1], flu_north[0], flu_north[2]])
print(f"\nFLU [1000,0,0] (North) → ENU {enu_north}")
print(f"   East={enu_north[0]}, North={enu_north[1]}, Up={enu_north[2]}")
if enu_north[1] > 0 and enu_north[0] == 0:
    print("   ✓ Correct: North component positive, East zero")
else:
    print("   ✗ Wrong!")

# Test case 2: East movement
flu_east = np.array([0, -1000, 0])  # Negative Y because Y is Left
enu_east = np.array([-flu_east[1], flu_east[0], flu_east[2]])
print(f"\nFLU [0,-1000,0] (East) → ENU {enu_east}")
print(f"   East={enu_east[0]}, North={enu_east[1]}, Up={enu_east[2]}")
if enu_east[0] > 0 and enu_east[1] == 0:
    print("   ✓ Correct: East component positive, North zero")
else:
    print("   ✗ Wrong!")

print("\n" + "="*60)