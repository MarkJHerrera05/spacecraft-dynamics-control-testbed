"""Quaternion utilities for spacecraft attitude simulations.

Convention
----------
Quaternions are stored as [q0, q1, q2, q3], where q0 is the scalar term.
The vector part [q1, q2, q3] represents the rotation axis contribution.
"""

from __future__ import annotations

import numpy as np


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """Return a unit-length quaternion."""
    q = np.asarray(q, dtype=float)
    norm_q = np.linalg.norm(q)

    if norm_q <= 0.0:
        raise ValueError("Quaternion norm must be positive.")

    return q / norm_q


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """Return the conjugate of q = [q0, q1, q2, q3]."""
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quaternion_multiply(q_left: np.ndarray, q_right: np.ndarray) -> np.ndarray:
    """Return the Hamilton product q_left ⊗ q_right."""
    q_left = np.asarray(q_left, dtype=float)
    q_right = np.asarray(q_right, dtype=float)

    w1, x1, y1, z1 = q_left
    w2, x2, y2, z2 = q_right

    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quaternion_error(current_q: np.ndarray, desired_q: np.ndarray) -> np.ndarray:
    """Return quaternion error that rotates from desired attitude to current attitude.

    The returned quaternion is forced to the shortest rotation by keeping the
    scalar term non-negative. This avoids commanding a long rotation when q and
    -q represent the same attitude.
    """
    current_q = normalize_quaternion(current_q)
    desired_q = normalize_quaternion(desired_q)

    error_q = quaternion_multiply(quaternion_conjugate(desired_q), current_q)
    error_q = normalize_quaternion(error_q)

    if error_q[0] < 0.0:
        error_q = -error_q

    return error_q


def quaternion_kinematics(q: np.ndarray, omega_body_rad_s: np.ndarray) -> np.ndarray:
    """Compute quaternion derivative from body angular rate.

    Parameters
    ----------
    q : ndarray, shape (4,)
        Current attitude quaternion [q0, q1, q2, q3].
    omega_body_rad_s : ndarray, shape (3,)
        Body angular velocity [rad/s].
    """
    q = normalize_quaternion(q)
    omega_quat = np.array([0.0, *np.asarray(omega_body_rad_s, dtype=float)])
    return 0.5 * quaternion_multiply(q, omega_quat)


def quaternion_to_angle_deg(q: np.ndarray) -> float:
    """Return principal rotation angle represented by a unit quaternion [deg]."""
    q = normalize_quaternion(q)
    q0_abs = np.clip(abs(q[0]), -1.0, 1.0)
    return float(np.rad2deg(2.0 * np.arccos(q0_abs)))
