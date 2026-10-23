import unittest, math
from src.fdm.math3d import Vec3, Mat3


class TestMath3D(unittest.TestCase):
    def test_dot(self):
        a = Vec3(1, 2, 3)
        b = Vec3(4, 5, 6)
        self.assertEqual(a.dot(b), 32)

    def test_cross(self):
        a = Vec3(1, 0, 0)
        b = Vec3(0, 1, 0)
        c = a.cross(b)
        self.assertAlmostEqual(c.x, 0)
        self.assertAlmostEqual(c.y, 0)
        self.assertAlmostEqual(c.z, 1)

    def test_rotation_identity(self):
        m = Mat3.rotation_yaw_pitch_roll(0, 0, 0)
        v = m * Vec3(1, 2, 3)
        self.assertAlmostEqual(v.x, 1)
        self.assertAlmostEqual(v.y, 2)
        self.assertAlmostEqual(v.z, 3)


if __name__ == "__main__":
    unittest.main()
