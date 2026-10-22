"""Terrain and runway world model — stdlib only."""

import math
import random


class World:
    def __init__(self, seed=42):
        self.seed = seed
        self.rng = random.Random(seed)
        # runway: origin at (0,0), heading 0 (north), length 2000m, width 45m
        self.runway_len = 2000.0
        self.runway_w = 45.0
        self.runway_heading = 0.0
        # simple terrain: flat around runway, gentle hills elsewhere
        self.hills = []
        for _ in range(60):
            self.hills.append((
                self.rng.uniform(-8000, 8000),
                self.rng.uniform(-8000, 8000),
                self.rng.uniform(20, 120),
                self.rng.uniform(200, 600),
            ))

    def terrain_height(self, x, y):
        h = 0.0
        # flatten runway corridor
        if abs(y) < self.runway_w * 2 and 0 <= x <= self.runway_len:
            return 0.0
        for hx, hy, amp, r in self.hills:
            d = math.hypot(x - hx, y - hy)
            if d < r:
                h += amp * (1.0 - d / r) ** 2
        return h

    def on_runway(self, x, y):
        return (0 <= x <= self.runway_len and abs(y) <= self.runway_w / 2.0)
