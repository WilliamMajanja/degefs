import unittest, math
from src.fdm.math3d import Vec3, Mat3, Mat4, Quat, Frustum


class TestMath3DExtra(unittest.TestCase):
    def test_quat_slerp(self):
        q1 = Quat.from_euler(0, 0, 0)
        q2 = Quat.from_euler(0, 0, math.pi / 2)
        q = Quat.slerp(q1, q2, 0.5)
        r, p, y = q.to_euler()
        self.assertAlmostEqual(y, math.pi / 4, places=5)

    def test_mat4_perspective(self):
        m = Mat4.perspective(math.pi / 4, 1.5, 0.1, 1000.0)
        self.assertEqual(len(m.m), 16)

    def test_frustum(self):
        m = Mat4.perspective(math.pi / 2, 1.0, 0.1, 100.0)
        f = Frustum(m)
        self.assertTrue(f.contains_point(Vec3(0, 0, -1)))

    def test_vec3_normalize(self):
        v = Vec3(3, 4, 0).normalized()
        self.assertAlmostEqual(v.length(), 1.0)


if __name__ == "__main__":
    unittest.main()
