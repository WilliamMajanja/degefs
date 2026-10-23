"""Tkinter-based cockpit + outside view. Stdlib only."""

import math
import time
import tkinter as tk

import sys
import os

try:
    from ..fdm.aircraft import Aircraft
    from ..environment.world import World
    from ..environment.atmosphere import (clamp, ms_to_kts, m_to_ft, ft_to_m, kts_to_ms,
                                          isa, wind_vector, speed_of_sound)
    from ..avionics.autopilot import Autopilot
    from ..io.persist import load_state, save_state
    from ..core.property_tree import PropertyNode
    from ..io.input import InputManager
except Exception:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from src.fdm.aircraft import Aircraft
    from src.environment.world import World
    from src.environment.atmosphere import (clamp, ms_to_kts, m_to_ft, ft_to_m, kts_to_ms,
                                          isa, wind_vector, speed_of_sound)
    from src.avionics.autopilot import Autopilot
    from src.io.persist import load_state, save_state
    from src.core.property_tree import PropertyNode
    from src.io.input import InputManager


class DegeFS:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("DegeFS — Zero-Dep Flight Simulator")
        self.root.geometry("1200x780")
        self.root.configure(bg="#0a0f1a")
        self.ac = Aircraft()
        self.world = World()
        self.message = ""
        self.message_until = 0.0
        st = load_state()
        if st and isinstance(st, dict) and "aircraft" in st:
            try:
                self.ac.from_dict(st["aircraft"])
                self.message = "Loaded saved state"
            except Exception:
                pass
        self.running = True
        self.paused = False
        self.last = time.time()
        self.accumulator = 0.0
        self.fixed_dt = 1.0 / 120.0
        self.render_dt = 1.0 / 60.0
        self.keys = set()
        self.input_mgr = InputManager()
        self.pt = PropertyNode("sim")
        self.autopilot = Autopilot()
        self._scenario(3)
        self.autopilot.engage(self.ac)
        self.scenario = "cruise"
        self.wind_speed = 0.0
        self.wind_dir = 0.0
        self._wind_t = 0.0
        self.waypoints = [(0.0, 0.0), (5000.0, 0.0), (5000.0, 5000.0), (0.0, 5000.0)]
        self.wp_index = 0
        self.hud = True
        self.mouse_look = False
        self.mouse_sensitivity = 0.003
        self.cam_yaw_offset = 0.0
        self.cam_pitch_offset = 0.0
        self.time_of_day = 12.0  # hours
        self.time_scale = 0.02

        # canvas
        self.cv = tk.Canvas(self.root, width=1200, height=560, bg="#1a2a3a", highlightthickness=0)
        self.cv.pack(fill=tk.BOTH, expand=True)
        self.inf = tk.Frame(self.root, bg="#0a0f1a")
        self.inf.pack(fill=tk.X)
        self.labels = {}
        for name in ("IAS", "ALT", "HDG", "VS", "ROLL", "PITCH", "THR", "RPM", "STATUS", "AP", "WND", "NAV"):
            lbl = tk.Label(self.inf, text=f"{name}: --", fg="#7fffd4", bg="#0a0f1a", font=("Courier", 11, "bold"))
            lbl.pack(side=tk.LEFT, padx=8)
            self.labels[name] = lbl
        help_ = tk.Label(self.root, text="Controls: W/S thr  A/D ail  Up/Down ele  Left/Right rud  F flaps  B brake  G gear  I AP  H HUD  M mouse-look  1-5 scenarios  O save  L load  P pause  R reset  Q quit",
                         fg="#aaaaaa", bg="#0a0f1a", font=("Courier", 9))
        help_.pack(side=tk.BOTTOM, fill=tk.X)

        self.root.bind("<KeyPress>", self._down)
        self.root.bind("<KeyRelease>", self._up)
        self.cv.bind("<KeyPress>", self._down)
        self.cv.bind("<KeyRelease>", self._up)
        self.cv.focus_set()
        try:
            self.root.focus_force()
        except Exception:
            pass
        self.cv.bind("<Motion>", self._mouse_motion)
        self.cv.bind("<Button-1>", lambda e: self._set_mouse_look(True))
        self.root.bind("<ButtonRelease-1>", lambda e: self._set_mouse_look(False))
        self.root.protocol("WM_DELETE_WINDOW", self._quit)
        self.last = time.time()
        self.root.after(16, self._loop)

    def _set_mouse_look(self, v):
        self.mouse_look = v

    def _mouse_motion(self, e):
        if self.mouse_look:
            dx = e.x - 600
            dy = e.y - 280
            self.input_mgr.mouse_move(dx * 0.001, dy * 0.001)
            # reset cursor
            try:
                self.root.warp_pointer(600, 280)
            except Exception:
                pass

    def _down(self, e):
        k = e.keysym.lower()
        self.keys.add(k)
        self.input_mgr.key_down(k)
        if k == "q":
            self._quit()
        if k == "p":
            self.paused = not self.paused
        if k == "r":
            self._reset()
        if k == "i":
            if self.autopilot.enabled:
                self.autopilot.disengage()
                self._flash("Autopilot OFF")
            else:
                self.autopilot.engage(self.ac)
                self._flash("Autopilot ON")
        if k == "h":
            self.hud = not self.hud
        if k == "m":
            self.mouse_look = not self.mouse_look
        if k == "o":
            self._save_state()
        if k == "l":
            self._load_state()
        if k in ("1", "2", "3", "4", "5"):
            self._scenario(int(k))

    def _up(self, e):
        k = e.keysym.lower()
        self.keys.discard(k)
        self.input_mgr.key_up(k)

    def _quit(self):
        self.running = False
        try:
            save_state({"aircraft": self.ac.to_dict(), "scenario": self.scenario})
        except Exception:
            pass
        self.root.destroy()

    def _reset(self):
        self.ac = Aircraft()
        self.autopilot.disengage()
        self._flash("Reset")

    def _flash(self, msg):
        self.message = msg
        self.message_until = time.time() + 2.0

    def _update_weather(self, dt):
        self._wind_t += dt
        self.time_of_day = (self.time_of_day + self.time_scale * dt) % 24.0
        wx, wy, wz = wind_vector(self.ac.x, self.ac.y, self.ac.z, self._wind_t,
                                 base_speed=self.wind_speed if self.wind_speed > 0 else 0.0,
                                 base_dir=self.wind_dir)
        # smooth toward profile
        self.ac.wind_x = wx
        self.ac.wind_y = wy
        self.ac.wind_z = wz
        self.wind_speed = math.hypot(self.ac.wind_x, self.ac.wind_y)
        self.wind_dir = math.degrees(math.atan2(self.ac.wind_y, self.ac.wind_x)) % 360.0

    def _update_nav(self):
        if not self.waypoints:
            return
        if self.wp_index >= len(self.waypoints):
            self.wp_index = 0
        tx, ty = self.waypoints[self.wp_index]
        dx = tx - self.ac.x
        dy = ty - self.ac.y
        dist = math.hypot(dx, dy)
        if dist < 300.0:
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            self._flash("Waypoint reached")
            return
        brg = (math.degrees(math.atan2(dy, dx)) % 360.0)
        self.labels["NAV"].config(text=f"NAV: WP{self.wp_index + 1} {dist:6.0f}m {brg:03.0f}deg")

    def _sync_properties(self):
        ac = self.ac
        self.pt.set_value("position/x", ac.x, "float")
        self.pt.set_value("position/y", ac.y, "float")
        self.pt.set_value("position/z", ac.z, "float")
        self.pt.set_value("velocities/airspeed-kt", ac.airspeed_kts(), "float")
        self.pt.set_value("velocities/vx", ac.vx, "float")
        self.pt.set_value("velocities/vy", ac.vy, "float")
        self.pt.set_value("velocities/vz", ac.vz, "float")
        self.pt.set_value("orientation/roll-deg", math.degrees(ac.roll), "float")
        self.pt.set_value("orientation/pitch-deg", math.degrees(ac.pitch), "float")
        self.pt.set_value("orientation/heading-deg", ac.heading_deg(), "float")
        self.pt.set_value("controls/throttle", ac.throttle, "float")
        self.pt.set_value("controls/elevator", ac.elevator, "float")
        self.pt.set_value("controls/aileron", ac.aileron, "float")
        self.pt.set_value("controls/rudder", ac.rudder, "float")
        self.pt.set_value("engine/rpm", ac.engine_rpm, "float")
        self.pt.set_value("engine/manifold-pressure", ac.manifold_pressure, "float")
        self.pt.set_value("environment/wind-x", ac.wind_x, "float")
        self.pt.set_value("environment/wind-y", ac.wind_y, "float")

    def _save_state(self):
        try:
            data = {"aircraft": self.ac.to_dict(), "scenario": self.scenario}
            if save_state(data):
                self._flash("State saved")
            else:
                self._flash("Save failed")
        except Exception:
            self._flash("Save failed")

    def _load_state(self):
        try:
            st = load_state()
            if st and isinstance(st, dict) and "aircraft" in st:
                self.ac.from_dict(st["aircraft"])
                self.autopilot.disengage()
                self._flash("State loaded")
            else:
                self._flash("No saved state")
        except Exception:
            self._flash("Load failed")

    def _scenario(self, n):
        self.autopilot.disengage()
        ac = Aircraft()
        if n == 1:
            ac.x = 100.0
            ac.z = -0.5
            ac.vx = ac.vy = ac.vz = 0.0
            ac.throttle = 0.0
            ac.brake = 1.0
            ac.gear = True
            ac.flaps = 0.0
            self.scenario = "cold_dark"
            self._flash("Scenario: Cold & Dark")
        elif n == 2:
            ac.x = 100.0
            ac.z = -0.5
            ac.vx = ac.vy = ac.vz = 0.0
            ac.throttle = 0.0
            ac.brake = 0.0
            ac.gear = True
            self.scenario = "takeoff"
            self._flash("Scenario: Takeoff")
        elif n == 3:
            ac.x = 5000.0
            ac.z = -ft_to_m(3000.0)
            ac.vx = kts_to_ms(90.0)
            ac.vy = ac.vz = 0.0
            ac.throttle = 0.65
            ac.pitch = 0.04
            ac.elevator = 0.15
            ac.gear = False
            ac.flaps = 0.0
            self.scenario = "cruise"
            self._flash("Scenario: Cruise")
        elif n == 4:
            ac.x = 15000.0
            ac.z = -ft_to_m(1000.0)
            ac.vx = kts_to_ms(75.0)
            ac.vy = ac.vz = 0.0
            ac.throttle = 0.25
            ac.pitch = -0.05
            ac.gear = True
            ac.flaps = 0.5
            self.scenario = "landing"
            self._flash("Scenario: Landing")
        elif n == 5:
            ac.x = 5000.0
            ac.z = -ft_to_m(3000.0)
            ac.vx = kts_to_ms(90.0)
            ac.vy = ac.vz = 0.0
            ac.throttle = 0.0
            ac.pitch = 0.0
            ac.gear = False
            ac.flaps = 0.0
            self.scenario = "engine_failure"
            self._flash("Scenario: Engine Failure")
        self.ac = ac


    def _handle_input(self):
        ac = self.ac
        polled = self.input_mgr.poll()
        keys = polled["keys"] | self.keys
        if self.autopilot.enabled and any(k in keys for k in ("w", "s", "a", "d", "up", "down", "left", "right", "f", "b")):
            self.autopilot.disengage()
            self._flash("Autopilot OFF (manual)")
        if "w" in keys:
            ac.throttle = clamp(ac.throttle + 0.01, 0.0, 1.0)
        if "s" in keys:
            ac.throttle = clamp(ac.throttle - 0.01, 0.0, 1.0)
        target = 0.0
        if "a" in keys:
            target -= 0.4
        if "d" in keys:
            target += 0.4
        ac.aileron += (target - ac.aileron) * 0.2
        et = 0.0
        if "up" in keys:
            et += 0.35
        if "down" in keys:
            et -= 0.35
        ac.elevator += (et - ac.elevator) * 0.2
        rt = 0.0
        if "left" in keys:
            rt -= 0.35
        if "right" in keys:
            rt += 0.35
        ac.rudder += (rt - ac.rudder) * 0.2
        if "f" in keys:
            ac.flaps = clamp(ac.flaps + 0.005, 0.0, 1.0)
        ac.brake = 1.0 if "b" in keys else 0.0
        if "g" in keys:
            ac.gear = not ac.gear
            self.keys.discard("g")
            self.input_mgr.key_up("g")
        # mouse-look affects camera only
        if self.mouse_look:
            self.cam_yaw_offset += polled["mouse_dx"] * self.mouse_sensitivity * 10
            self.cam_pitch_offset += polled["mouse_dy"] * self.mouse_sensitivity * 10

    def _project(self, wx, wy, wz, cam_x, cam_y, cam_z, yaw, pitch):
        dx = wx - cam_x
        dy = wy - cam_y
        dz = wz - cam_z
        cy, sy = math.cos(-yaw), math.sin(-yaw)
        x = dx * cy - dy * sy
        y = dx * sy + dy * cy
        z = dz
        cp, sp = math.cos(-pitch), math.sin(-pitch)
        x2 = x * cp - z * sp
        z2 = x * sp + z * cp
        if z2 <= 1.0:
            return None
        fov = 700.0
        sx = 600 + (x2 / z2) * fov
        sy = 280 + (y / z2) * fov * 0.6
        return sx, sy

    def _sky_color(self):
        # time of day -> sun elevation approx
        hour = self.time_of_day
        # sun angle: 0 at 6, 90 at 12, 0 at 18
        if hour < 6 or hour > 18:
            elev = -10
        else:
            elev = 90 * math.sin((hour - 6) / 12 * math.pi)
        if elev > 0:
            top = "#1a3a6a"
            bot = "#87ceeb"
        elif elev > -6:
            top = "#2a2a4a"
            bot = "#ff8844"
        else:
            top = "#0a0a1a"
            bot = "#0a0a2a"
        return top, bot, elev

    def _draw(self):
        cv = self.cv
        cv.delete("all")
        w, h = 1200, 560
        top, bot, elev = self._sky_color()
        # sky gradient (simple bands)
        for i in range(12):
            t = i / 11.0
            # blend top->bot
            cv.create_rectangle(0, int(i * h * 0.55 / 12), w, int((i + 1) * h * 0.55 / 12), fill=top if i < 6 else bot, outline="")
        cv.create_rectangle(0, int(h * 0.55), w, h, fill="#2a4a2a", outline="")
        # sun/moon
        if elev >= 0:
            sx = 600 + 400 * math.cos(math.radians(90 - elev))
            sy = 60 - elev * 0.5
            cv.create_oval(sx - 20, sy - 20, sx + 20, sy + 20, fill="#ffdd66", outline="#ffaa00")
        else:
            cv.create_oval(100, 60, 130, 90, fill="#cccccc", outline="#888888")
        ac = self.ac
        yaw = ac.yaw + self.cam_yaw_offset
        pitch = ac.pitch + self.cam_pitch_offset
        cam_x = ac.x - 80 * math.cos(yaw)
        cam_y = ac.y - 80 * math.sin(yaw)
        cam_z = ac.z - 25
        # terrain grid with LOD
        step = 400
        rng = range(-2000, 2001, step)
        # draw lines
        for gx in rng:
            pts = []
            for gy in range(-2000, 2001, step):
                th = self.world.terrain_height(gx, gy)
                p = self._project(gx, gy, -th, cam_x, cam_y, cam_z, yaw, pitch)
                if p:
                    pts.append(p)
            if len(pts) > 1:
                for i in range(len(pts) - 1):
                    cv.create_line(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], fill="#1f3f1f", width=1)
        for gy in rng:
            pts = []
            for gx in range(-2000, 2001, step):
                th = self.world.terrain_height(gx, gy)
                p = self._project(gx, gy, -th, cam_x, cam_y, cam_z, yaw, pitch)
                if p:
                    pts.append(p)
            if len(pts) > 1:
                for i in range(len(pts) - 1):
                    cv.create_line(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], fill="#1f3f1f", width=1)
        # runway
        self._draw_runway(cv, cam_x, cam_y, cam_z, yaw, pitch)
        # aircraft symbol
        cx, cy = 600, 280
        cv.create_line(cx, cy - 14, cx, cy + 14, fill="#ffff00", width=2)
        cv.create_line(cx - 20, cy, cx + 20, cy, fill="#ffff00", width=2)
        # horizon
        cv.create_line(0, h // 2 + 40, w, h // 2 + 40, fill="#88aaff", width=1, dash=(4, 4))
        if ac.crashed:
            cv.create_text(w // 2, h // 2, text="CRASHED — press R to reset", fill="#ff4444", font=("Courier", 24, "bold"))
        if self.hud:
            self._draw_hud(cv, w, h)

    def _far(self):
        return abs(self.ac.altitude_ft()) > 5000

    def _draw_runway(self, cv, cam_x, cam_y, cam_z, yaw, pitch):
        world = self.world
        rw = world.runway_w
        rl = world.runway_len
        corners = [(0, -rw / 2), (rl, -rw / 2), (rl, rw / 2), (0, rw / 2)]
        pts = []
        for (rx, ry) in corners:
            p = self._project(rx, ry, 0.0, cam_x, cam_y, cam_z, yaw, pitch)
            if p:
                pts.append(p)
        if len(pts) == 4:
            cv.create_polygon(pts[0][0], pts[0][1], pts[1][0], pts[1][1],
                              pts[2][0], pts[2][1], pts[3][0], pts[3][1],
                              fill="#444444", outline="#ffffff")
        # centerline dashes
        for i in range(0, int(rl), 40):
            a = self._project(i, 0, 0, cam_x, cam_y, cam_z, yaw, pitch)
            b = self._project(i + 20, 0, 0, cam_x, cam_y, cam_z, yaw, pitch)
            if a and b:
                cv.create_line(a[0], a[1], b[0], b[1], fill="#ffffff", width=2)
        # threshold markings
        for i in range(0, 10):
            a = self._project(5, -rw / 2 + i * (rw / 10), 0, cam_x, cam_y, cam_z, yaw, pitch)
            b = self._project(25, -rw / 2 + i * (rw / 10), 0, cam_x, cam_y, cam_z, yaw, pitch)
            if a and b:
                cv.create_line(a[0], a[1], b[0], b[1], fill="#ffffff", width=3)
        # approach lights (simple)
        for i in range(5):
            ax = - (i + 1) * 60
            p = self._project(ax, 0, 0, cam_x, cam_y, cam_z, yaw, pitch)
            if p:
                cv.create_oval(p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3, fill="#ff4444", outline="")
        # PAPI (4 lights)
        for i in range(4):
            p = self._project(300, -rw / 2 - 20 + i * 10, 0, cam_x, cam_y, cam_z, yaw, pitch)
            if p:
                color = "#ff0000" if i < 2 else "#ffffff"
                cv.create_oval(p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4, fill=color, outline="")

    def _draw_hud(self, cv, w, h):
        ac = self.ac
        # attitude ladder (simple)
        pitch_px = math.degrees(ac.pitch) * 4
        for i in range(-3, 4):
            y = 280 - pitch_px + i * 40
            if 0 <= y <= h:
                cv.create_line(560, y, 640, y, fill="#7fffd4", width=1)
                cv.create_text(545, y, text=f"{i*5}", fill="#7fffd4", font=("Courier", 8))
        # speed/alt tapes
        cv.create_rectangle(40, 120, 120, 440, fill="", outline="#7fffd4")
        cv.create_text(80, 100, text="IAS", fill="#7fffd4", font=("Courier", 10))
        cv.create_rectangle(1080, 120, 1160, 440, fill="", outline="#7fffd4")
        cv.create_text(1120, 100, text="ALT", fill="#7fffd4", font=("Courier", 10))
        # compass rose
        cv.create_text(600, 500, text=f"HDG {ac.heading_deg():03.0f}", fill="#7fffd4", font=("Courier", 12, "bold"))

    def _update_instruments(self):
        ac = self.ac
        self.labels["IAS"].config(text=f"IAS: {ac.airspeed_kts():6.1f} kt")
        self.labels["ALT"].config(text=f"ALT: {ac.altitude_ft():7.0f} ft")
        self.labels["HDG"].config(text=f"HDG: {ac.heading_deg():5.1f}°")
        self.labels["VS"].config(text=f"VS:  {ac.vs_fpm():+7.0f} fpm")
        self.labels["ROLL"].config(text=f"ROLL: {math.degrees(ac.roll):+5.1f}°")
        self.labels["PITCH"].config(text=f"PITCH:{math.degrees(ac.pitch):+5.1f}°")
        self.labels["THR"].config(text=f"THR:  {ac.throttle * 100:5.1f}%")
        self.labels["RPM"].config(text=f"RPM: {ac.engine_rpm:6.0f}")
        st = "CRASHED" if ac.crashed else ("GROUND" if ac.on_ground else "AIR")
        self.labels["STATUS"].config(text=f"ST: {st}")
        ap_state = "ON" if self.autopilot.enabled else "OFF"
        self.labels["AP"].config(text=f"AP: {ap_state}")
        self.labels["WND"].config(text=f"WND: {ms_to_kts(self.wind_speed):4.1f}kt/{self.wind_dir:03.0f}")
        if self.message and time.time() < self.message_until:
            self.labels["STATUS"].config(text=f"ST: {st}  {self.message}")

    def _loop(self):
        if not self.running:
            return
        try:
            now = time.time()
            frame_time = now - self.last
            if frame_time < 0 or frame_time > 0.1:
                frame_time = 0.016
            self.last = now
            self.accumulator += frame_time
            if self.accumulator > self.fixed_dt * 3:
                self.accumulator = self.fixed_dt * 3
            while self.accumulator >= self.fixed_dt:
                if not self.paused and not self.ac.crashed:
                    # manual override detection
                    polled_keys = self.keys | self.input_mgr.keys
                    if self.autopilot.enabled and (polled_keys & {"w","s","a","d","up","down","left","right","f","b"}):
                        self.autopilot.disengage()
                        self._flash("Autopilot OFF (manual)")
                    if self.autopilot.enabled:
                        self.autopilot.update(self.ac, self.fixed_dt)
                    else:
                        self._handle_input()
                    self._update_weather(self.fixed_dt)
                    self.ac.update(self.fixed_dt)
                    self._sync_properties()
                self.accumulator -= self.fixed_dt
            self._update_nav()
            self._draw()
            self._update_instruments()
        except Exception as exc:
            self._flash(f"Error: {exc}")
        if self.running:
            self.root.after(16, self._loop)

    def run(self):
        self.root.mainloop()


def main():
    DegeFS().run()


if __name__ == "__main__":
    main()
