import unittest
from src.core.property_tree import PropertyNode


class TestPropertyTree(unittest.TestCase):
    def test_set_get(self):
        root = PropertyNode("root")
        root.set_value("sim/time", 1.5, "float")
        self.assertAlmostEqual(root.get_value("sim/time"), 1.5)

    def test_observer(self):
        root = PropertyNode("root")
        seen = []
        root.add_listener("x", lambda n: seen.append(n.value))
        root.set_value("x", 42, "int")
        self.assertEqual(seen, [42])

    def test_nested(self):
        root = PropertyNode("root")
        root.set_value("a/b/c", "hello", "string")
        self.assertEqual(root.get_value("a/b/c"), "hello")


if __name__ == "__main__":
    unittest.main()
