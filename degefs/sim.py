"""Tkinter-based cockpit + 3D-ish outside view. Stdlib only."""

import math
import time
import tkinter as tk

import sys, os
if __package__:
    from .aircraft import Aircraft
else:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from aircraft import Aircraft
if __package__:
    from .world import World
else:
    from world import World
if __package__:
    from .atmosphere import clamp, ms_to_kts, m_to_ft
else:
    from atmosphere import clamp, ms_to_kts, m_to_ft


class DegeFS:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("DegeFS — Zero-Dep Flight Simulator")
        self.root.geometry("1100x720")
        self.root.configure(bg="#0a0f1a")
        self.ac = Aircraft()
        self.world = World()
        self.running = True
        self.paused = False
        self.last = time.time()
        self.keys = set()

        # canvas
        self.cv = tk.Canvas(self.root, width=1100, height=520, bg="#1a2a3a", highlightthickness=0)
        self.cv.pack(fill=tk.BOTH, expand=True)
        # instruments frame
        self.inf = tk.Frame(self.root, bg="#0a0f1a")
        self.inf.pack(fill=tk.X)
        self.labels = {}
        for name in ("IAS", "ALT", "HDG", "VS", "ROLL", "PITCH", "THR", "STATUS"):
            lbl = tk.Label(self.inf, text=f"{name}: --", fg="#7fffd4", bg="#0a0f1a", font=("Courier", 12, "bold"))
            lbl.pack(side=tk.LEFT, padx=12)
            self.labels[name] = lbl
        # help
        help_ = tk.Label(self.root, text="Controls: W/S throttle  A/D aileron  ↑/↓ elevator  ←/→ rudder  F flaps  B brake  G gear  P pause  R reset  Q quit",
                         fg="#aaaaaa", bg="#0a0f1a", font=("Courier", 9))
        help_.pack(side=tk.BOTTOM, fill=tk.X)

        self.root.bind("<KeyPress>", self._down)
        self.root.bind("<KeyRelease>", self._up)
        self.root.protocol("WM_DELETE_WINDOW", self._quit)
        self._loop()

    def _down(self, e):
        self.keys.add(e.keysym.lower())
        if e.keysym.lower() == "q":
            self._quit()
        if e.keysym.lower() == "p":
            self.paused = not self.paused
        if e.keysym.lower() == "r":
            self._reset()

    def _up(self, e):
        self.keys.discard(e.keysym.lower())

    def _quit(self):
        self.running = False
        self.root.destroy()

    def _reset(self):
        self.ac = Aircraft()

    def _handle_input(self):
        ac = self.ac
        # throttle
        if "w" in self.keys:
            ac.throttle = clamp(ac.throttle + 0.01, 0.0, 1.0)
        if "s" in self.keys:
            ac.throttle = clamp(ac.throttle - 0.01, 0.0, 1.0)
        # aileron
        target = 0.0
        if "a" in self.keys:
            target -= 0.6
        if "d" in self.keys:
            target += 0.6
        ac.aileron += (target - ac.aileron) * 0.2
        # elevator
        et = 0.0
        if "up" in self.keys:
            et -= 0.5
        if "down" in self.keys:
            et += 0.5
        ac.elevator += (et - ac.elevator) * 0.2
        # rudder
        rt = 0.0
        if "left" in self.keys:
            rt -= 0.5
        if "right" in self.keys:
            rt += 0.5
        ac.rudder += (rt - ac.rudder) * 0.2
        # flaps
        if "f" in self.keys:
            ac.flaps = clamp(ac.flaps + 0.005, 0.0, 1.0)
        # brake
        ac.brake = 1.0 if "b" in self.keys else 0.0
        # gear
        if "g" in self.keys:
            ac.gear = not ac.gear
            self.keys.discard("g")

    def _project(self, wx, wy, wz, cam_x, cam_y, cam_z, yaw, pitch, roll):
        # world -> camera (NED: x north, y east, z down). Camera behind aircraft.
        dx = wx - cam_x
        dy = wy - cam_y
        dz = wz - cam_z
        # rotate by -yaw, -pitch (simplified: yaw only for view)
        cy, sy = math.cos(-self.ac.yaw), math.sin(-self.ac.yaw)
        x = dx * cy - dy * sy
        y = dx * sy + dy * cy
        z = dz
        # pitch
        cp, sp = math.cos(-self.ac.pitch), math.sin(-self.ac.pitch)
        x2 = x * cp - z * sp
        z2 = x * sp + z * cp
        # view
        if z2 <= 1.0:
            return None
        fov = 600.0
        sx = 550 + (x2 / z2) * fov
        sy = 260 + (y / z2) * fov * 0.6
        return sx, sy

    def _draw(self):
        cv = self.cv
        cv.delete("all")
        w, h = 1100, 520
        # sky gradient-ish
        cv.create_rectangle(0, 0, w, h // 2 + 60, fill="#2a4a6a", outline="")
        cv.create_rectangle(0, h // 2 + 60, w, h, fill="#3a5a3a", outline="")
        ac = self.ac
        cam_x, cam_y, cam_z = ac.x - 60 * math.cos(ac.yaw), ac.y - 60 * math.sin(ac.yaw), ac.z - 20
        # draw terrain grid
        step = 200
        rng = range(-4000, 4001, step)
        lines = []
        for gx in rng:
            pts = []
            for gy in range(-4000, 4001, step):
                th = self.world.terrain_height(gx, gy)
                p = self._project(gx, gy, -th, cam_x, cam_y, cam_z, ac.yaw, ac.pitch, ac.roll)
                if p:
                    pts.append(p)
            if len(pts) > 1:
                for i in range(len(pts) - 1):
                    cv.create_line(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], fill="#2a4a2a", width=1)
        for gy in rng:
            pts = []
            for gx in range(-4000, 4001, step):
                th = self.world.terrain_height(gx, gy)
                p = self._project(gx, gy, -th, cam_x, cam_y, cam_z, ac.yaw, ac.pitch, ac.roll)
                if p:
                    pts.append(p)
            if len(pts) > 1:
                for i in range(len(pts) - 1):
                    cv.create_line(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], fill="#2a4a2a", width=1)
        # runway
        rw = self.world.runway_w
        rl = self.world.runway_len
        corners = [(0, -rw / 2), (rl, -rw / 2), (rl, rw / 2), (0, rw / 2)]
        pts = []
        for (rx, ry) in corners:
            p = self._project(rx, ry, 0.0, cam_x, cam_y, cam_z, ac.yaw, ac.pitch, ac.roll)
            if p:
                pts.append(p)
        if len(pts) == 4:
            cv.create_polygon(pts[0][0], pts[0][1], pts[1][0], pts[1][1], pts[2][0], pts[2][1], pts[3][0], pts[3][1], fill="#444444", outline="#ffffff")
        # centerline
        for i in range(0, int(rl), 40):
            a = self._project(i, 0, 0, cam_x, cam_y, cam_z, ac.yaw, ac.pitch, ac.roll)
            b = self._project(i + 20, 0, 0, cam_x, cam_y, cam_z, ac.yaw, ac.pitch, ac.roll)
            if a and b:
                cv.create_line(a[0], a[1], b[0], b[1], fill="#ffffff", width=2)
        # aircraft symbol (center)
        cx, cy = 550, 260
        cv.create_line(cx, cy - 12, cx, cy + 12, fill="#ffff00", width=2)
        cv.create_line(cx - 18, cy, cx + 18, cy, fill="#ffff00", width=2)
        # horizon line
        cv.create_line(0, h // 2 + 60, w, h // 2 + 60, fill="#88aaff", width=1, dash=(4, 4))
        # attitude
        if ac.crashed:
            cv.create_text(w // 2, h // 2, text="CRASHED — press R to reset", fill="#ff4444", font=("Courier", 24, "bold"))

    def _update_instruments(self):
        ac = self.ac
        self.labels["IAS"].config(text=f"IAS: {ac.airspeed_kts():6.1f} kt")
        self.labels["ALT"].config(text=f"ALT: {ac.altitude_ft():7.0f} ft")
        self.labels["HDG"].config(text=f"HDG: {ac.heading_deg():5.1f}°")
        self.labels["VS"].config(text=f"VS:  {ac.vs_fpm():+7.0f} fpm")
        self.labels["ROLL"].config(text=f"ROLL: {math.degrees(ac.roll):+5.1f}°")
        self.labels["PITCH"].config(text=f"PITCH:{math.degrees(ac.pitch):+5.1f}°")
        self.labels["THR"].config(text=f"THR:  {ac.throttle*100:5.1f}%")
        st = "CRASHED" if ac.crashed else ("GROUND" if ac.on_ground else "AIR")
        self.labels["STATUS"].config(text=f"ST: {st}")

    def _loop(self):
        if not self.running:
            return
        now = time.time()
        dt = now - self.last
        self.last = now
        if not self.paused and not self.ac.crashed:
            self._handle_input()
            self.ac.update(dt if dt > 0 else 0.016)
        self._draw()
        self._update_instruments()
        self.root.after(16, self._loop)

    def run(self):
        self.root.mainloop()


def main():
    DegeFS().run()


if __name__ == "__main__":
    main()
