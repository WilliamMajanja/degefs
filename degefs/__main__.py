"""DegeFS entrypoint."""

import sys
from .sim import main

if __name__ == "__main__":
    main()
EOF
# fix __main__
cat > /home/minima/Documents/Projects/DegeFS/degefs/__main__.py <<'EOF'
from .sim import main

main()
EOF
cat > /home/minima/Documents/Projects/DegeFS/pyproject.toml <<'EOF'
[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[project]
name = "degefs"
version = "0.1.0"
description = "Zero-Dep Flight Simulator in Python (concepts ported from FlightGear)."
readme = "README.md"
requires-python = ">=3.9"
license = { text = "MIT" }

[project.scripts]
degefs = "degefs.sim:main"
EOF
echo done