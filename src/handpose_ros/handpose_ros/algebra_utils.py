#!/usr/bin/env python3
"""
algebra_utils.py
- Collection of frequently used linear algebra / robotics coordinate system utilities
"""

import math
import numpy as np

# ---------------------- General utilities ----------------------
def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """
    Normalize a vector.

    Args:
        v (np.ndarray): input vector
        eps (float): small epsilon to avoid division by zero

    Returns:
        np.ndarray: normalized vector, or original if norm < eps
    """
    n = np.linalg.norm(v)
    return v if n < eps else (v / n)

def angle_between_vectors(v1: np.ndarray, v2: np.ndarray,
                          normal: np.ndarray,
                          degrees: bool = False) -> float:
    """
    Compute the signed angle between two vectors given a reference normal.

    Args:
        v1 (np.ndarray): first vector
        v2 (np.ndarray): second vector
        normal (np.ndarray): reference normal to determine sign
        degrees (bool): if True, returns angle in degrees; otherwise radians

    Returns:
        float: signed angle between v1 and v2
    """
    v1u = normalize(v1)
    v2u = normalize(v2)
    dot_prod = float(np.clip(np.dot(v1u, v2u), -1.0, 1.0))
    cross_prod = np.cross(v1u, v2u)
    theta_abs = np.arctan2(np.linalg.norm(cross_prod), dot_prod)
    sign_dir = np.sign(np.dot(normalize(normal), cross_prod))
    return float(sign_dir * (np.rad2deg(theta_abs) if degrees else theta_abs))

def make_homogeneous(rotm: np.ndarray, trans: np.ndarray) -> np.ndarray:
    """
    Construct a 4x4 homogeneous transformation matrix.

    Args:
        rotm (np.ndarray): 3x3 rotation matrix
        trans (np.ndarray): 3x1 translation vector

    Returns:
        np.ndarray: 4x4 homogeneous transform
    """
    T = np.eye(4)
    T[:3, :3] = rotm
    T[:3,  3] = trans.reshape(3)
    return T

def make_inv_homogeneous(T: np.ndarray) -> np.ndarray:
    """
    Compute the inverse of a 4x4 homogeneous transform.

    Args:
        T (np.ndarray): 4x4 homogeneous transform

    Returns:
        np.ndarray: inverse transform
    """
    Rm = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = Rm.T
    Ti[:3, 3] = -Rm.T @ t
    return Ti

# ---------------------- Rotation / Quaternion utilities ----------------------
def rotm_to_quat(R: np.ndarray):
    """
    Convert a rotation matrix to a quaternion (qx, qy, qz, qw).

    Supports 3x3 or 4x4 (uses the upper 3x3 block). Uses the trace>0 formula
    with branching on the dominant diagonal element to improve numerical stability.
    The output quaternion is normalized.

    Args:
        R (np.ndarray): 3x3 or 4x4 rotation matrix

    Returns:
        tuple: (qx, qy, qz, qw)
    """
    R = np.asarray(R, dtype=float)
    if R.shape == (4, 4):
        R = R[:3, :3]
    elif R.shape != (3, 3):
        raise ValueError("R must be 3x3 or 4x4 rotation matrix")

    t = R[0, 0] + R[1, 1] + R[2, 2]

    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    q /= n
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
