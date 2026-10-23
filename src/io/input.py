"""Zero-latency input polling engine (stdlib only)."""

import time


class InputManager:
    def __init__(self):
        self.keys = set()
        self.mouse_dx = 0.0
        self.mouse_dy = 0.0
        self.mouse_buttons = set()
        self.joystick_axes = {}
        self.joystick_buttons = set()
        self.deadzone = 0.08
        self.expo = 0.35
        self._last_poll = time.time()

    def key_down(self, key):
        self.keys.add(str(key).lower())

    def key_up(self, key):
        self.keys.discard(str(key).lower())

    def mouse_move(self, dx, dy):
        self.mouse_dx += dx
        self.mouse_dy += dy

    def mouse_button(self, btn, down):
        if down:
            self.mouse_buttons.add(btn)
        else:
            self.mouse_buttons.discard(btn)

    def apply_deadzone(self, value):
        v = float(value)
        if abs(v) < self.deadzone:
            return 0.0
        sign = 1.0 if v > 0 else -1.0
        mag = (abs(v) - self.deadzone) / (1.0 - self.deadzone)
        # exponential response curve
        mag = mag ** (1.0 + self.expo)
        return sign * mag

    def poll(self):
        now = time.time()
        dt = now - self._last_poll
        if dt <= 0:
            dt = 1e-3
        self._last_poll = now
        # consume mouse deltas
        dx, dy = self.mouse_dx, self.mouse_dy
        self.mouse_dx = 0.0
        self.mouse_dy = 0.0
        return {
            "keys": set(self.keys),
            "mouse_dx": dx,
            "mouse_dy": dy,
            "mouse_buttons": set(self.mouse_buttons),
            "joystick_axes": dict(self.joystick_axes),
            "joystick_buttons": set(self.joystick_buttons),
            "dt": dt,
        }

    def is_down(self, key):
        return str(key).lower() in self.keys

Input = InputManager

def poll_input(mgr):
    return mgr.poll()
