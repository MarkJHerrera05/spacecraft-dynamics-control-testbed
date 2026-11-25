"""Point-mass gravity plus the unnormalized J2 zonal harmonic."""

import numpy as np

from .two_body import two_body_dynamics


MU_EARTH = 398600.4418  # km^3/s^2
EARTH_EQUATORIAL_RADIUS = 6378.137  # km
EARTH_J2 = 1.08263e-3  # dimensionless


def j2_dynamics(t, state, mu=MU_EARTH, radius=EARTH_EQUATORIAL_RADIUS, j2=EARTH_J2):
    """Return [velocity, acceleration] in km/s and km/s^2 for solve_ivp.

    Use a body-centered inertial frame with +z along the symmetry axis.
    Position is in km. The reference radius is the equatorial radius in km.
    """
    state = np.asarray(state, dtype=float)
    x, y, z = state[:3]
    r_squared = np.dot(state[:3], state[:3])
    if r_squared == 0:
        raise ValueError("Gravity is undefined at the center of the body")

    derivative = two_body_dynamics(t, state, mu)
    latitude_term = 5.0 * z**2 / r_squared
    scale = 1.5 * mu * j2 * radius**2 / r_squared**2.5
    derivative[3:] += scale * np.array([
        x * (latitude_term - 1.0),
        y * (latitude_term - 1.0),
        z * (latitude_term - 3.0),
    ])
    return derivative
