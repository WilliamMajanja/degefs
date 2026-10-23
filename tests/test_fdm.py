import unittest
from src.fdm.aircraft import Aircraft


class TestFDM(unittest.TestCase):
    def test_no_crash_level(self):
        ac = Aircraft()
        ac.throttle = 0.4
        for _ in range(200):
            ac.update(1 / 60.0)
        self.assertFalse(ac.crashed)

    def test_state_roundtrip(self):
        ac = Aircraft()
        ac.x = 123.4
        d = ac.to_dict()
        ac2 = Aircraft()
        ac2.from_dict(d)
        self.assertAlmostEqual(ac2.x, 123.4)


if __name__ == "__main__":
    unittest.main()
