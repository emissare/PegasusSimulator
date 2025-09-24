"""
| File: rotations.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Implements utilitary rotations between coordinate frame conventions.

COORDINATE SYSTEMS:
- Isaac World: NWU (North-West-Up) - X points North, Y points West, Z points Up
- PX4 World: NED (North-East-Down) - X points North, Y points East, Z points Down
- Isaac Body: FLU (Front-Left-Up) - X points Front, Y points Left, Z points Up
- PX4 Body: FRD (Front-Right-Down) - X points Front, Y points Right, Z points Down
"""
import numpy as np
from scipy.spatial.transform import Rotation

# Transformation from Isaac Sim NWU world frame to PX4 NED world frame
# Isaac NWU world: X=North, Y=West, Z=Up
# PX4 NED world: X=North, Y=East, Z=Down
# Transformation: X_nwu→X_ned (same), Y_nwu→-Y_ned (flip), Z_nwu→-Z_ned (flip)
# This is a 180° rotation around the X-axis (North axis)
q_NWU_to_NED = np.array([1.0, 0.0, 0.0, 0.0])  # 180° around North/X axis
rot_NWU_to_NED = Rotation.from_quat(q_NWU_to_NED)

# Transformation between Isaac FLU body frame and PX4 FRD body frame
# Isaac FLU body: X=Front, Y=Left, Z=Up
# PX4 FRD body: X=Front, Y=Right, Z=Down
# Transformation: X_flu→X_frd (same), Y_flu→-Y_frd (flip), Z_flu→-Z_frd (flip)
# This is a 180° rotation around the X-axis (Front axis)
# Note: While mathematically identical to NWU→NED, these are conceptually different transformations
q_FLU_to_FRD = np.array([1.0, 0.0, 0.0, 0.0])  # 180° around Front/X axis
rot_FLU_to_FRD = Rotation.from_quat(q_FLU_to_FRD)
