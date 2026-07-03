"""Run a closed-loop quaternion attitude-control demonstration.

This script simulates a rigid spacecraft using quaternion kinematics, Euler
rotational dynamics, and a PD attitude-control law. It saves plots showing
attitude-error convergence, body-rate damping, and commanded control torque.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, Union

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from attitude.quaternion import (  # noqa: E402
    normalize_quaternion,
    quaternion_error,
    quaternion_kinematics,
    quaternion_to_angle_deg,
)
from attitude.rigid_body import euler_rigid_body_dynamics  # noqa: E402
from control.quaternion_pd import QuaternionPDController  # noqa: E402


def closed_loop_attitude_dynamics(
    time_s: float,
    state: np.ndarray,
    inertia_kg_m2: np.ndarray,
    controller: QuaternionPDController,
    desired_q: np.ndarray,
) -> np.ndarray:
    """Return state derivative for closed-loop attitude dynamics."""
    del time_s  # Dynamics are time-invariant for this demonstration.

    q = normalize_quaternion(state[0:4])
    omega_body_rad_s = state[4:7]

    torque_nm = controller.compute_torque(q, desired_q, omega_body_rad_s)
    q_dot = quaternion_kinematics(q, omega_body_rad_s)
    omega_dot = euler_rigid_body_dynamics(omega_body_rad_s, torque_nm, inertia_kg_m2)

    return np.hstack((q_dot, omega_dot))


def run_demo() -> Dict[str, Union[np.ndarray, float]]:
    """Run the demonstration and return histories for plotting/reporting."""
    inertia_kg_m2 = np.diag([12.0, 10.0, 8.0])
    desired_q = np.array([1.0, 0.0, 0.0, 0.0])

    initial_q = normalize_quaternion(np.array([0.9063, 0.0, 0.4226, 0.0]))  # ~50 deg about body y-axis
    initial_omega_rad_s = np.deg2rad(np.array([1.5, -2.0, 0.8]))
    initial_state = np.hstack((initial_q, initial_omega_rad_s))

    controller = QuaternionPDController(
        proportional_gain=np.array([0.45, 0.45, 0.45]),
        derivative_gain=np.array([5.0, 5.0, 5.0]),
        max_torque_nm=np.array([0.25, 0.25, 0.25]),
    )

    simulation_time_s = (0.0, 240.0)
    output_time_s = np.linspace(simulation_time_s[0], simulation_time_s[1], 1201)

    solution = solve_ivp(
        fun=lambda t, y: closed_loop_attitude_dynamics(t, y, inertia_kg_m2, controller, desired_q),
        t_span=simulation_time_s,
        y0=initial_state,
        t_eval=output_time_s,
        rtol=1.0e-9,
        atol=1.0e-11,
    )

    if not solution.success:
        raise RuntimeError(f"Attitude-control simulation failed: {solution.message}")

    quaternion_history = np.apply_along_axis(normalize_quaternion, 0, solution.y[0:4, :])
    omega_history_rad_s = solution.y[4:7, :]

    attitude_error_deg = np.array(
        [quaternion_to_angle_deg(quaternion_error(quaternion_history[:, k], desired_q)) for k in range(solution.t.size)]
    )
    torque_history_nm = np.array(
        [
            controller.compute_torque(quaternion_history[:, k], desired_q, omega_history_rad_s[:, k])
            for k in range(solution.t.size)
        ]
    ).T

    return {
        "time_s": solution.t,
        "quaternion_history": quaternion_history,
        "omega_history_rad_s": omega_history_rad_s,
        "attitude_error_deg": attitude_error_deg,
        "torque_history_nm": torque_history_nm,
        "final_attitude_error_deg": float(attitude_error_deg[-1]),
        "final_rate_norm_deg_s": float(np.linalg.norm(np.rad2deg(omega_history_rad_s[:, -1]))),
    }


def save_plots(results: Dict[str, Union[np.ndarray, float]]) -> Path:
    """Save attitude-control plots to the figures directory."""
    figures_dir = PROJECT_ROOT / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    output_path = figures_dir / "quaternion_pd_attitude_control.png"

    time_s = results["time_s"]
    attitude_error_deg = results["attitude_error_deg"]
    omega_deg_s = np.rad2deg(results["omega_history_rad_s"])
    torque_nm = results["torque_history_nm"]
    quaternion_history = results["quaternion_history"]

    fig, axes = plt.subplots(4, 1, figsize=(9.0, 10.0), sharex=True)

    axes[0].plot(time_s, attitude_error_deg)
    axes[0].set_ylabel("Attitude error [deg]")
    axes[0].grid(True)

    axes[1].plot(time_s, omega_deg_s[0, :], label=r"$\omega_x$")
    axes[1].plot(time_s, omega_deg_s[1, :], label=r"$\omega_y$")
    axes[1].plot(time_s, omega_deg_s[2, :], label=r"$\omega_z$")
    axes[1].set_ylabel("Body rates [deg/s]")
    axes[1].legend(loc="best")
    axes[1].grid(True)

    axes[2].plot(time_s, torque_nm[0, :], label=r"$\tau_x$")
    axes[2].plot(time_s, torque_nm[1, :], label=r"$\tau_y$")
    axes[2].plot(time_s, torque_nm[2, :], label=r"$\tau_z$")
    axes[2].set_ylabel("Torque [N m]")
    axes[2].legend(loc="best")
    axes[2].grid(True)

    axes[3].plot(time_s, quaternion_history[0, :], label=r"$q_0$")
    axes[3].plot(time_s, quaternion_history[1, :], label=r"$q_1$")
    axes[3].plot(time_s, quaternion_history[2, :], label=r"$q_2$")
    axes[3].plot(time_s, quaternion_history[3, :], label=r"$q_3$")
    axes[3].set_xlabel("Time [s]")
    axes[3].set_ylabel("Quaternion")
    axes[3].legend(loc="best")
    axes[3].grid(True)

    fig.suptitle("Quaternion PD Attitude-Control Demo")
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)

    return output_path


def main() -> None:
    results = run_demo()
    output_path = save_plots(results)

    print("Quaternion PD attitude-control demo complete.")
    print(f"Final attitude error: {results['final_attitude_error_deg']:.6f} deg")
    print(f"Final body-rate norm: {results['final_rate_norm_deg_s']:.6f} deg/s")
    print(f"Saved figure: {output_path.relative_to(PROJECT_ROOT)}")


if __name__ == "__main__":
    main()
