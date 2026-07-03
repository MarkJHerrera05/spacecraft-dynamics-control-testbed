"""Quaternion feedback attitude controller."""

from __future__ import annotations

from typing import Optional, Union

import numpy as np

from attitude.quaternion import quaternion_error


class QuaternionPDController:
    """PD attitude controller using quaternion vector error and body rates.

    The control law is

        tau = -Kp * q_error_vector - Kd * omega_body

    where q_error is the shortest-rotation attitude error between the desired
    quaternion and the current quaternion.
    """

    def __init__(
        self,
        proportional_gain: Union[float, np.ndarray],
        derivative_gain: Union[float, np.ndarray],
        max_torque_nm: Optional[Union[float, np.ndarray]] = None,
    ) -> None:
        self.kp = self._as_gain_vector(proportional_gain, "proportional_gain")
        self.kd = self._as_gain_vector(derivative_gain, "derivative_gain")
        self.max_torque_nm = None

        if max_torque_nm is not None:
            self.max_torque_nm = self._as_gain_vector(max_torque_nm, "max_torque_nm")

    @staticmethod
    def _as_gain_vector(value: Union[float, np.ndarray], name: str) -> np.ndarray:
        value_array = np.asarray(value, dtype=float)

        if value_array.ndim == 0:
            return np.full(3, float(value_array))

        if value_array.shape != (3,):
            raise ValueError(f"{name} must be a scalar or a 3-element vector.")

        return value_array

    def compute_torque(
        self,
        current_q: np.ndarray,
        desired_q: np.ndarray,
        omega_body_rad_s: np.ndarray,
    ) -> np.ndarray:
        """Compute commanded control torque [N*m]."""
        q_err = quaternion_error(current_q, desired_q)
        omega_body_rad_s = np.asarray(omega_body_rad_s, dtype=float)

        torque_nm = -self.kp * q_err[1:4] - self.kd * omega_body_rad_s

        if self.max_torque_nm is not None:
            torque_nm = np.clip(torque_nm, -self.max_torque_nm, self.max_torque_nm)

        return torque_nm
