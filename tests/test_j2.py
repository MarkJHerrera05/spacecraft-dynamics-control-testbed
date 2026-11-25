import unittest

import numpy as np

from scripts.run_j2_orbit import SECONDS_PER_DAY, node_history
from src.orbit.j2 import EARTH_EQUATORIAL_RADIUS, EARTH_J2, MU_EARTH, j2_dynamics
from src.orbit.two_body import two_body_dynamics


class J2Tests(unittest.TestCase):
    def test_zero_j2_recovers_two_body(self):
        state = np.array([6500., -1500., 2200., 1., 6., -3.])
        np.testing.assert_array_equal(
            j2_dynamics(0, state, j2=0), two_body_dynamics(0, state, MU_EARTH)
        )

    def test_equatorial_and_polar_acceleration(self):
        r = 7000.
        g = MU_EARTH / r**2
        correction = 1.5 * g * EARTH_J2 * (EARTH_EQUATORIAL_RADIUS / r)**2
        for position, expected in [
            ([0, r, 0], [0, -g - correction, 0]),
            ([0, 0, r], [0, 0, -g + 2 * correction]),
        ]:
            np.testing.assert_allclose(j2_dynamics(0, position + [0]*3)[3:], expected,
                                       rtol=1e-13, atol=0)

    def test_secular_rate_for_prograde_and_retrograde_orbits(self):
        a = EARTH_EQUATORIAL_RADIUS + 650.0
        n = np.sqrt(MU_EARTH / a**3)
        for inclination_deg in [51.6, 98.0]:
            with self.subTest(inclination_deg=inclination_deg):
                i = np.deg2rad(inclination_deg)
                state = [a, 0, 0, 0, n*a*np.cos(i), n*a*np.sin(i)]
                days, raan = node_history(j2_dynamics, state, 2*SECONDS_PER_DAY, 2*np.pi/n)
                self.assertGreater(len(days), 25)
                measured = np.polyfit(days, raan, 1)[0]
                expected = -1.5 * n * EARTH_J2 * (EARTH_EQUATORIAL_RADIUS/a)**2 * np.cos(i)
                expected = np.rad2deg(expected) * SECONDS_PER_DAY
                # First-order theory uses mean elements. The initial state is osculating.
                self.assertAlmostEqual(measured / expected, 1., delta=0.01)
