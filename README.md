# DegeFS — Zero-Dep Flight Simulator

Pure-Python flight simulator with zero third-party dependencies (Python
standard library + tkinter). Concepts ported in spirit from
[FlightGear](https://github.com/flightgear/flightgear).

## Architecture

```
src/
  core/        property tree, config
  fdm/         6-DOF rigid body, blade-element aero, math3d (Vec3/Mat3/Mat4/Quat)
  environment/ ISA atmosphere, wind/turbulence, terrain & runway world
  avionics/    PID autopilot (ALT/HDG/SPD hold, ILS/GS)
  graphics/    tkinter renderer, HUD, sky/terrain/runway
  io/          persistence, input polling
```

- Physics core runs at 120 Hz fixed timestep (accumulator in `graphics/sim.py`).
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

## Performance Tuning

- `fixed_dt` in `src/graphics/sim.py` (default 1/120 s).
- Terrain LOD step adjusts with altitude.
- Zero dynamic allocations in hot loops where practical.

## License

MIT.
