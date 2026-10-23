"""Glass cockpit display helpers (stdlib only)."""

import math


class PFD:
    def __init__(self):
        self.pitch = 0.0
        self.roll = 0.0
        self.ias = 0.0
        self.alt = 0.0
        self.vs = 0.0
        self.hdg = 0.0

    def update_from_aircraft(self, ac):
        self.pitch = ac.pitch
        self.roll = ac.roll
        self.ias = ac.airspeed_kts()
        self.alt = ac.altitude_ft()
        self.vs = ac.vs_fpm()
        self.hdg = ac.heading_deg()
        return self.to_dict()

    def to_dict(self):
        return {"pitch": self.pitch, "roll": self.roll, "ias": self.ias,
                "alt": self.alt, "vs": self.vs, "hdg": self.hdg}


class EngineDisplay:
    def __init__(self):
        self.rpm = 0.0
        self.mp = 0.0
        self.ff = 0.0
        self.oil_t = 0.0
        self.oil_p = 0.0
        self.egt = 0.0

    def update_from_aircraft(self, ac):
        self.rpm = ac.engine_rpm
        self.mp = ac.manifold_pressure
        self.ff = ac.fuel_flow
        self.oil_t = ac.oil_temp
        self.oil_p = ac.oil_pressure
        self.egt = ac.egt
        return self.to_dict()

    def to_dict(self):
        return {"rpm": self.rpm, "mp": self.mp, "ff": self.ff,
                "oil_t": self.oil_t, "oil_p": self.oil_p, "egt": self.egt}


class NavigationDisplay:
    def __init__(self):
        self.wp_distance = 0.0
        self.wp_bearing = 0.0

    def update(self, x, y, tx, ty):
        dx = tx - x
        dy = ty - y
        self.wp_distance = math.hypot(dx, dy)
        self.wp_bearing = math.degrees(math.atan2(dy, dx)) % 360.0
        return self.to_dict()

    def to_dict(self):
        return {"distance": self.wp_distance, "bearing": self.wp_bearing}
