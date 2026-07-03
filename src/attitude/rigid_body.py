"""Rigid-body attitude dynamics utilities."""

from __future__ import annotations

import numpy as np


def euler_rigid_body_dynamics(
    omega_body_rad_s: np.ndarray,
    control_torque_nm: np.ndarray,
    inertia_kg_m2: np.ndarray,
) -> np.ndarray:
    """Compute angular acceleration from Euler's rotational equations.

    I * omega_dot + omega × (I * omega) = tau
    """
    omega_body_rad_s = np.asarray(omega_body_rad_s, dtype=float)
    control_torque_nm = np.asarray(control_torque_nm, dtype=float)
    inertia_kg_m2 = np.asarray(inertia_kg_m2, dtype=float)

    angular_momentum = inertia_kg_m2 @ omega_body_rad_s
    gyroscopic_term = np.cross(omega_body_rad_s, angular_momentum)

    return np.linalg.solve(inertia_kg_m2, control_torque_nm - gyroscopic_term)
