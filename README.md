# DegeFS — Zero-Dependency Flight Simulator

Pure-Python flight simulator with zero third-party dependencies (Python
standard library + tkinter). Concepts ported in spirit from
[FlightGear](https://github.com/flightgear/flightgear).

## Features

- 6-DOF rigid-body flight model with blade-element aerodynamics
- ISA standard atmosphere and wind model
- Procedural terrain and runway world
- Glass-cockpit instruments (IAS, ALT, HDG, VS, ROLL, PITCH, THR, RPM)
- PID autopilot: altitude hold, heading select, airspeed hold
- Keyboard + mouse-look input, HUD overlay
- Hierarchical property tree (pub/sub backbone)

## Architecture

```
src/
  core/        property tree, config
  fdm/         6-DOF rigid body, aerodynamics, math3d (Vec3/Mat3/Mat4/Quat)
  environment/ ISA atmosphere, wind/turbulence, terrain & runway world
  avionics/    PID autopilot, glass-cockpit displays
  graphics/    tkinter renderer, HUD, sky/terrain/runway
  io/          persistence, input polling
```

- Physics core runs at a fixed 120 Hz timestep (accumulator in `graphics/sim.py`).
- Render loop decoupled at ~60 FPS.
- Property tree (`core/property_tree.py`) is the pub/sub backbone.

## Build & Run

```bash
make compile   # byte-compile
make test      # run unit tests
make run       # launch simulator
```

Or directly:

```bash
python -m src
# or
python -m degefs
```

## Controls

| Key | Action |
|---|---|
| W / S | Throttle up / down |
| A / D | Aileron (roll) |
| ↑ / ↓ | Elevator (pitch) |
| ← / → | Rudder (yaw) |
| F | Flaps (hold) |
| B | Brakes |
| G | Gear toggle |
| H | HUD toggle |
| M | Mouse-look toggle |
| I | Autopilot toggle |
| 1-5 | Scenarios |
| O / L | Save / Load state |
| P | Pause |
| R | Reset |
| Q | Quit |

## Scenarios

1. Cold & Dark
2. Takeoff
3. Cruise (default)
4. Landing
5. Engine Failure

## Performance Tuning

- `fixed_dt` in `src/graphics/sim.py` (default 1/120 s).
- Terrain LOD step adjusts with altitude.
- Accumulator capped to 3 steps to avoid spiral-of-death.

## Testing

```bash
python -m pytest -q
```

17 tests covering math, FDM stability, property tree thread safety.

## License

MIT.
