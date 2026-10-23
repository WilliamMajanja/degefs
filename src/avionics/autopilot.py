"""PID-based autopilot with altitude/heading/airspeed hold and ILS tracking."""

import math

try:
    from ..environment.atmosphere import clamp, deg2rad, rad2deg
except Exception:
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from src.environment.atmosphere import clamp, deg2rad, rad2deg


class PID:
    def __init__(self, kp, ki, kd, limit=1.0, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral_limit = integral_limit if integral_limit is not None else limit
        self.integral = 0.0
        self.prev = 0.0
        self.prev_set = False

    def reset(self):
        self.integral = 0.0
        self.prev = 0.0
        self.prev_set = False

    def update(self, error, dt):
        dt = max(dt, 1e-4)
        self.integral = clamp(self.integral + error * dt, -self.integral_limit, self.integral_limit)
        if not self.prev_set:
            deriv = 0.0
            self.prev_set = True
        else:
            deriv = (error - self.prev) / dt
        self.prev = error
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        return clamp(out, -self.limit, self.limit)


class Autopilot:
    def __init__(self):
        self.enabled = False
        self.modes = {"alt_hold": True, "hdg_sel": True, "spd_hold": True, "ils": False, "gs": False}
        self.target_alt = 1000.0  # ft
        self.target_hdg = 0.0     # deg
        self.target_speed = 90.0  # kts
        self.trim_pitch = 0.04
        # ILS
        self.runway_x = 0.0
        self.runway_y = 0.0
        self.runway_heading = 0.0
        self.glideslope_deg = 3.0
        # PIDs
        self.roll_pid = PID(1.2, 0.05, 0.0, 0.6)
        self.pitch_pid = PID(5.0, 0.5, 0.0, 1.0)
        self.alt_pid = PID(0.005, 0.0001, 0.0, 0.35)
        self.hdg_pid = PID(0.08, 0.002, 0.0, 0.5)
        self.spd_pid = PID(0.02, 0.001, 0.0, 1.0)
        self.loc_pid = PID(0.05, 0.001, 0.0, 0.4)
        self.gs_pid = PID(0.08, 0.002, 0.0, 0.35)

    def engage(self, ac, modes=None):
        self.enabled = True
        self.target_alt = ac.altitude_ft()
        self.target_hdg = ac.heading_deg()
        self.target_speed = max(60.0, ac.airspeed_kts())
        if modes:
            self.modes.update(modes)
        for pid in (self.roll_pid, self.pitch_pid, self.alt_pid, self.hdg_pid,
                    self.spd_pid, self.loc_pid, self.gs_pid):
            pid.reset()

    def disengage(self):
        self.enabled = False

    def set_targets(self, alt=None, hdg=None, speed=None):
        if alt is not None:
            self.target_alt = alt
        if hdg is not None:
            self.target_hdg = hdg % 360.0
        if speed is not None:
            self.target_speed = speed

    def arm_ils(self, armed=True):
        self.modes["ils"] = armed
        self.modes["gs"] = armed

    def update(self, ac, dt):
        if not self.enabled or ac.crashed:
            return
        dt = max(dt, 1e-4)

        # ILS localizer
        if self.modes.get("ils"):
            # cross-track error
            dx = ac.x - self.runway_x
            dy = ac.y - self.runway_y
            # rotate to runway frame
            hr = math.radians(self.runway_heading)
            along = dx * math.cos(hr) + dy * math.sin(hr)
            cross = -dx * math.sin(hr) + dy * math.cos(hr)
            # desired heading correction
            self.target_hdg = (self.runway_heading - math.degrees(math.atan2(cross, max(along, 1.0)))) % 360.0
            # glideslope
            if self.modes.get("gs") and along > 0:
                alt_ft = ac.altitude_ft()
                gs_alt = self.runway_x * 0 + (along * math.tan(math.radians(self.glideslope_deg))) * 3.28084  # approx
                # simpler: target alt based on distance to threshold
                thr_dist = max(0.0, (10000.0 - along))  # dummy scale
                self.target_alt = max(0.0, along * math.tan(math.radians(self.glideslope_deg)) * 3.28084)

        # altitude hold -> desired pitch
        if self.modes.get("alt_hold") or self.modes.get("gs"):
            alt_err = self.target_alt - ac.altitude_ft()
            vs = ac.vs_fpm()
            desired_pitch = clamp(self.trim_pitch + self.alt_pid.update(alt_err, dt), -0.25, 0.25)
            # speed protection: avoid stall
            if ac.airspeed_kts() < self.target_speed - 5.0:
                desired_pitch = min(desired_pitch, 0.05)
        else:
            desired_pitch = 0.0
        pitch_err = desired_pitch - ac.pitch
        ac.elevator = self.pitch_pid.update(pitch_err, dt)

        # heading select -> desired roll
        if self.modes.get("hdg_sel") or self.modes.get("ils"):
            hdg_err = (self.target_hdg - ac.heading_deg() + 180.0) % 360.0 - 180.0
            hdg_err = math.radians(hdg_err)
            desired_roll = clamp(self.hdg_pid.update(hdg_err, dt), -0.6, 0.6)
        else:
            desired_roll = 0.0
        roll_err = desired_roll - ac.roll
        ac.aileron = self.roll_pid.update(roll_err, dt)

        # speed hold -> throttle
        if self.modes.get("spd_hold"):
            spd_err = self.target_speed - ac.airspeed_kts()
            ac.throttle = clamp(ac.throttle + self.spd_pid.update(spd_err, dt) * 0.05, 0.0, 1.0)

        # rudder coordination
        ac.rudder = clamp(-0.15 * ac.r + (self.target_hdg - ac.heading_deg()) * 0.001, -0.4, 0.4)

        ac.flaps = 0.0
        ac.brake = 0.0
