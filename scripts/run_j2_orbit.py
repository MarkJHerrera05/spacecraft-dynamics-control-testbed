"""Compare J2 nodal precession with two-body motion and first-order theory."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

from src.orbit.j2 import EARTH_EQUATORIAL_RADIUS, EARTH_J2, MU_EARTH, j2_dynamics
from src.orbit.two_body import two_body_dynamics


SECONDS_PER_DAY = 86400.0


def ascending_node(t, state, *args):
    """Detect south-to-north equator crossings."""
    return state[2]


ascending_node.direction = 1


def node_history(dynamics, initial_state, duration, period):
    solution = solve_ivp(
        dynamics, (0.0, duration), initial_state, args=(MU_EARTH,),
        method="DOP853", rtol=1e-10, atol=1e-12, max_step=period / 12,
        events=ascending_node,
    )
    if not solution.success:
        raise RuntimeError(solution.message)
    nodes = solution.y_events[0]
    days = solution.t_events[0] / SECONDS_PER_DAY
    # At the ascending node the position itself points along the node vector.
    raan = np.rad2deg(np.unwrap(np.arctan2(nodes[:, 1], nodes[:, 0])))
    return days, raan


def main():
    altitude_km = 650.0
    inclination = np.deg2rad(51.6)
    a = EARTH_EQUATORIAL_RADIUS + altitude_km
    speed = np.sqrt(MU_EARTH / a)
    initial_state = [a, 0, 0, 0, speed * np.cos(inclination), speed * np.sin(inclination)]
    mean_motion = np.sqrt(MU_EARTH / a**3)
    period = 2 * np.pi / mean_motion
    duration = 2 * SECONDS_PER_DAY

    fig, ax = plt.subplots()
    for model, label in [(two_body_dynamics, "Two-body"), (j2_dynamics, "Two-body + J2")]:
        days, raan = node_history(model, initial_state, duration, period)
        drift = raan - raan[0]
        measured = np.polyfit(days, drift, 1)[0]
        print(f"{label} fitted RAAN rate: {measured:+.5f} deg/day")
        ax.plot(days, drift, "o", ms=3, label=label)

    reference = (
        -1.5 * mean_motion * EARTH_J2
        * (EARTH_EQUATORIAL_RADIUS / a)**2 * np.cos(inclination)
    )
    reference = np.rad2deg(reference) * SECONDS_PER_DAY
    print(f"First-order prediction: {reference:+.5f} deg/day")
    endpoints = np.array([0, duration / SECONDS_PER_DAY])
    ax.plot(endpoints, reference * endpoints, "--", label="First-order prediction")
    ax.set(xlabel="Elapsed time [days]", ylabel="Change in RAAN [deg]",
           title="Nodal drift from Earth's oblateness")
    ax.legend()
    ax.grid(alpha=0.3)
    path = Path(__file__).resolve().parents[1] / "figures" / "j2_nodal_drift.png"
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {path}")


if __name__ == "__main__":
    main()
