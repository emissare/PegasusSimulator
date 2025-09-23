"""
| File: rotations.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Implements utilitary rotations between ENU and NED inertial frame conventions and FLU and FRD body frame conventions.

IMPORTANT: Isaac Sim uses FLU (Front-Left-Up), NOT ENU (East-North-Up)!
The legacy variable names are misleading.
"""
import numpy as np
from scipy.spatial.transform import Rotation

# LEGACY: Quaternion for rotation between ENU and NED INERTIAL frames
# This was originally designed for ENU→NED but Isaac actually uses FLU, not ENU!
# Keeping for backward compatibility but should be replaced with FLU→NED
# Note: this quaternion follows the convention [qx, qy, qz, qw]
q_ENU_to_NED = np.array([0.70711, 0.70711, 0.0, 0.0])  # DEPRECATED - Isaac uses FLU not ENU
rot_ENU_to_NED = Rotation.from_quat(q_ENU_to_NED)  # DEPRECATED - use rot_FLU_inertial_to_NED_inertial

# CORRECT: Transformation from Isaac Sim FLU inertial to PX4 NED inertial frame
# Isaac FLU inertial: X=Front (we define as North), Y=Left (we define as West), Z=Up
# PX4 NED inertial: X=North, Y=East, Z=Down
# Transformation: X_flu→X_ned (same), Y_flu→-Y_ned (flip), Z_flu→-Z_ned (flip)
# This is a 180° rotation around the X-axis
q_FLU_inertial_to_NED_inertial = np.array([1.0, 0.0, 0.0, 0.0])  # 180° around X
rot_FLU_inertial_to_NED_inertial = Rotation.from_quat(q_FLU_inertial_to_NED_inertial)

# Quaternion for rotation between body FLU and body FRD frames
# Isaac FLU body: X=Front, Y=Left, Z=Up
# PX4 FRD body: X=Front, Y=Right, Z=Down
# Transformation: X_flu→X_frd (same), Y_flu→-Y_frd (flip), Z_flu→-Z_frd (flip)
# This is a 180° rotation around the X-axis (same as inertial transformation)
# Note: this quaternion follows the convention [qx, qy, qz, qw]
q_FLU_body_to_FRD_body = np.array([1.0, 0.0, 0.0, 0.0])  # 180° around X
rot_FLU_body_to_FRD_body = Rotation.from_quat(q_FLU_body_to_FRD_body)

# Keep legacy name for backward compatibility
q_FLU_to_FRD = q_FLU_body_to_FRD_body  # DEPRECATED - use q_FLU_body_to_FRD_body
rot_FLU_to_FRD = rot_FLU_body_to_FRD_body  # DEPRECATED - use rot_FLU_body_to_FRD_body
