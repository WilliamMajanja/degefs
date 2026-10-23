"""DegeFS entrypoint (src package)."""

import sys, os
try:
    from .graphics.sim import DegeFS
except Exception:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from src.graphics.sim import DegeFS


def main():
    DegeFS().run()


if __name__ == "__main__":
    main()
