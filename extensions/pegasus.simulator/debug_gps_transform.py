"""
Debug GPS transformation to understand what's happening.
"""

import numpy as np
from scipy.spatial.transform import Rotation

print("\n" + "="*60)
print("DEBUG GPS COORDINATE TRANSFORMATIONS")
print("="*60)

# Test the transformation chain
from pegasus.simulator.logic.rotations import rot_NWU_to_NED

# Test 1: 1000m North in FLU (positive X)
flu_north = np.array([1000, 0, 0])
print("\nFLU position (1000m North): [1000, 0, 0]")

# Apply FLU to NED transformation
# Expected: NED should be [1000, 0, 0] (North, East=0, Down=0)
ned = rot_NWU_to_NED.apply(flu_north)
print(f"After FLU→NED transform: {ned}")
print(f"  North (X): {ned[0]:.1f} m")
print(f"  East (Y): {ned[1]:.1f} m")
print(f"  Down (Z): {ned[2]:.1f} m")

# Convert NED to ENU for geodetic conversion
# ENU = [East, North, Up] = [NED_y, NED_x, -NED_z]
enu = np.array([ned[1], ned[0], -ned[2]])
print(f"After NED→ENU convert: {enu}")
print(f"  East (X): {enu[0]:.1f} m")
print(f"  North (Y): {enu[1]:.1f} m")
print(f"  Up (Z): {enu[2]:.1f} m")

# Test what reprojection expects
print("\nReprojection function expects:")
print("  position[0] = East")
print("  position[1] = North")
print("  position[2] = Up")

print("\n" + "-"*40)

# Test 2: 1000m East in FLU (negative Y, since Y is Left)
flu_east = np.array([0, -1000, 0])
print("\nFLU position (1000m East): [0, -1000, 0]")

ned = rot_NWU_to_NED.apply(flu_east)
print(f"After FLU→NED transform: {ned}")
print(f"  North (X): {ned[0]:.1f} m")
print(f"  East (Y): {ned[1]:.1f} m")
print(f"  Down (Z): {ned[2]:.1f} m")

enu = np.array([ned[1], ned[0], -ned[2]])
print(f"After NED→ENU convert: {enu}")
print(f"  East (X): {enu[0]:.1f} m")
print(f"  North (Y): {enu[1]:.1f} m")
print(f"  Up (Z): {enu[2]:.1f} m")

print("\n" + "-"*40)

# Check the transformation matrix itself
print("\nFLU to NED transformation (180° around X-axis):")
print("Quaternion [x,y,z,w]:", [1.0, 0.0, 0.0, 0.0])
rot = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])
matrix = rot.as_matrix()
print("Rotation matrix:")
print(matrix)

print("\n" + "="*60)