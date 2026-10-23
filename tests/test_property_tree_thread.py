import threading, unittest
from src.core.property_tree import PropertyNode


class TestPropertyTreeThread(unittest.TestCase):
    def test_concurrent_set_get(self):
        root = PropertyNode("root")
        errors = []

        def worker(i):
            try:
                for j in range(200):
                    root.set_value(f"a/{i}", j, "int")
                    root.get_value(f"a/{i}")
            except Exception as e:
                errors.append(e)

        ts = [threading.Thread(target=worker, args=(i,)) for i in range(8)]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        self.assertEqual(errors, [])

    def test_listener_thread(self):
        root = PropertyNode("root")
        seen = []
        lock = threading.Lock()

        def cb(n):
            with lock:
                seen.append(n.value)

        root.add_listener("x", cb)
        ts = []
        for i in range(4):
            def w():
                for j in range(50):
                    root.set_value("x", j, "int")
            ts.append(threading.Thread(target=w))
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        self.assertGreater(len(seen), 0)


if __name__ == "__main__":
    unittest.main()
