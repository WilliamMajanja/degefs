# DegeFS — Zero-Dep Flight Simulator

A pure-Python flight simulator with zero third-party dependencies (only
the Python standard library + tkinter). Concepts are ported in spirit from
[FlightGear](https://github.com/flightgear/flightgear) (C++, GPL):

- ISA standard atmosphere
- 6-DOF rigid-body flight model with aerodynamic forces
- Runway + procedural terrain world
- Cockpit instrument panel (IAS, ALT, HDG, VS, ROLL, PITCH, THR)
- Keyboard controls

## Run

```bash
python -m degefs
# or
python degefs/sim.py
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
| P | Pause |
| R | Reset |
| Q | Quit |

## License

MIT. FlightGear is GPL; this project reimplements concepts only and ships no
FlightGear source code.
