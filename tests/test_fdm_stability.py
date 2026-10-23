import unittest
from src.fdm.aircraft import Aircraft
from src.environment.atmosphere import ft_to_m, kts_to_ms


class TestFDMStability(unittest.TestCase):
    def test_steep_dive(self):
        ac = Aircraft()
        ac.altitude_ft() if False else None
        ac.x = 0.0
        ac.z = -ft_to_m(10000.0)
        ac.vx = kts_to_ms(120.0)
        ac.pitch = -0.6
        ac.throttle = 0.0
        for _ in range(500):
            ac.update(1 / 120.0)
        import math
        self.assertTrue(math.isfinite(ac.x))
        self.assertFalse(ac.crashed or not math.isfinite(ac.z))

    def test_stall_recovery(self):
        ac = Aircraft()
        ac.x = 0.0
        ac.z = -ft_to_m(5000.0)
        ac.vx = kts_to_ms(40.0)
        ac.pitch = 0.3
        ac.throttle = 0.0
        for _ in range(300):
            ac.update(1 / 120.0)
        self.assertTrue(__import__("math").isfinite(ac.x))

    def test_ground_contact(self):
        ac = Aircraft()
        ac.x = 100.0
        ac.z = -0.5
        ac.vx = kts_to_ms(30.0)
        ac.throttle = 0.0
        for _ in range(100):
            ac.update(1 / 60.0)
        self.assertTrue(ac.on_ground)
        self.assertFalse(ac.crashed)


if __name__ == "__main__":
    unittest.main()
