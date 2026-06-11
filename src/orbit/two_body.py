import numpy as np


def two_body_dynamics(t, state, mu):
    """
    Computes the two-body equations of motion.

    Parameters
    ----------
    t : float
        Time [s]. Included for compatibility with numerical integrators.
    state : ndarray
        Spacecraft state vector [x, y, z, vx, vy, vz].
        Position is in km, velocity is in km/s.
    mu : float
        Gravitational parameter of the central body [km^3/s^2].

    Returns
    -------
    dstate_dt : ndarray
        Time derivative of the state vector.
    """

    r_vec = state[0:3]
    v_vec = state[3:6]

    r_norm = np.linalg.norm(r_vec)

    acceleration = -mu * r_vec / r_norm**3

    dstate_dt = np.zeros(6)
    dstate_dt[0:3] = v_vec
    dstate_dt[3:6] = acceleration

    return dstate_dt