"""DegeFS entrypoint (compatibility shim)."""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.graphics.sim import DegeFS

def main():
    DegeFS().run()

if __name__ == "__main__":
    main()
