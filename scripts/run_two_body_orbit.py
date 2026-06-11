import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from src.orbit.two_body import two_body_dynamics


def main():
    # Earth gravitational parameter [km^3/s^2]
    mu_earth = 398600.4418

    # Earth radius [km]
    earth_radius = 6378.137

    # Circular orbit altitude [km]
    altitude = 400.0

    # Initial orbit radius from Earth's center [km]
    r0_mag = earth_radius + altitude

    # Circular orbit speed [km/s]
    v0_mag = np.sqrt(mu_earth / r0_mag)

    # Initial position vector in ECI frame [km]
    r0 = np.array([r0_mag, 0.0, 0.0])

    # Initial velocity vector in ECI frame [km/s]
    v0 = np.array([0.0, v0_mag, 0.0])

    # Combined initial state vector
    state0 = np.concatenate((r0, v0))

    # Orbital period [s]
    orbital_period = 2 * np.pi * np.sqrt(r0_mag**3 / mu_earth)

    # Simulate for one orbit
    t_span = (0.0, orbital_period)

    # Time points where solution is saved
    t_eval = np.linspace(t_span[0], t_span[1], 1000)

    solution = solve_ivp(
        fun=lambda t, y: two_body_dynamics(t, y, mu_earth),
        t_span=t_span,
        y0=state0,
        t_eval=t_eval,
        rtol=1e-9,
        atol=1e-9
    )

    x = solution.y[0]
    y = solution.y[1]

    plt.figure()
    plt.plot(x, y, label="Spacecraft Orbit")

    earth = plt.Circle((0, 0), earth_radius, fill=False, label="Earth")
    plt.gca().add_patch(earth)

    plt.axis("equal")
    plt.xlabel("x [km]")
    plt.ylabel("y [km]")
    plt.title("Two-Body Orbit Propagation")
    plt.grid(True)
    plt.legend()
    plt.savefig("figures/two_body_orbit.png", dpi=300)
    plt.show()


if __name__ == "__main__":
    main()