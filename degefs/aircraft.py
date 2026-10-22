"""Aircraft model: a simplified 6-DOF rigid body with aerodynamic forces.

Ported conceptually from FlightGear's JSBSim/FDM layer: mass, inertia,
wing area, lift/drag coefficients, control-surface deflections, engine
thrust and a simple ground-contact model. Stdlib only.
"""

import math

import sys,os
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from atmosphere import isa, clamp, deg2rad, rad2deg


class Aircraft:
    def __init__(self):
        # mass & inertia
        self.mass = 1500.0          # kg (light GA)
        self.Ixx = 2500.0
        self.Iyy = 3500.0
        self.Izz = 5000.0
        # geometry
        self.S = 16.0               # wing area m^2
        self.b = 10.0               # span
        self.c = 1.6                # mean chord
        # aerodynamic coefficients (linearised)
        self.CL0 = 0.35
        self.CL_alpha = 5.2
        self.CD0 = 0.03
        self.induced_k = 0.045
        self.Cm0 = -0.02
        self.Cm_alpha = -0.8
        self.Cm_q = -8.0
        self.Cl_beta = -0.12
        self.Cn_beta = 0.08
        # engine
        self.max_thrust = 3000.0    # N
        self.throttle = 0.0
        # state (SI, NED-ish: x north, y east, z down)
        self.x = 0.0
        self.y = 0.0
        self.z = -100.0             # 100 m above ground
        self.vx = 40.0
        self.vy = 0.0
        self.vz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        # controls
        self.elevator = 0.0
        self.aileron = 0.0
        self.rudder = 0.0
        self.flaps = 0.0
        self.brake = 0.0
        self.gear = True
        # ground
        self.on_ground = False
        self.crashed = False

    def _body_to_ned(self, u, v, w):
        # velocity vector components in body frame -> NED (approx, small angles ok-ish)
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        # body -> inertial rotation
        x = (cp * cy) * u + (sr * sp * cy - cr * sy) * v + (cr * sp * cy + sr * sy) * w
        y = (cp * sy) * u + (sr * sp * sy + cr * cy) * v + (cr * sp * sy - sr * cy) * w
        z = (-sp) * u + (sr * cp) * v + (cr * cp) * w
        return x, y, z

    def update(self, dt):
        if self.crashed:
            return
        dt = min(dt, 0.05)
        alt = -self.z
        _, _, rho = isa(alt)
        rho = max(rho, 0.01)
        # body-frame velocities
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        # transform NED velocity to body
        Rbi = [
            [cp * cy, cp * sy, -sp],
            [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp],
            [cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp],
        ]
        u = Rbi[0][0] * self.vx + Rbi[0][1] * self.vy + Rbi[0][2] * self.vz
        v = Rbi[1][0] * self.vx + Rbi[1][1] * self.vy + Rbi[1][2] * self.vz
        w = Rbi[2][0] * self.vx + Rbi[2][1] * self.vy + Rbi[2][2] * self.vz
        V = math.sqrt(u * u + v * v + w * w)
        V = max(V, 0.1)
        alpha = math.atan2(w, u) if abs(u) > 1e-6 else 0.0
        beta = math.asin(clamp(v / V, -1.0, 1.0)) if V > 0.1 else 0.0
        qbar = 0.5 * rho * V * V

        # lift coefficient
        CL = self.CL0 + self.CL_alpha * alpha + self.flaps * 0.4
        CD = self.CD0 + self.induced_k * CL * CL + abs(self.aileron) * 0.01
        L = qbar * self.S * CL
        D = qbar * self.S * CD
        # side force
        CY = -0.3 * beta + self.rudder * 0.05
        Y = qbar * self.S * CY

        # body-frame forces: x forward, y right, z down
        fx = -D * math.cos(alpha) + L * math.sin(alpha) + self.throttle * self.max_thrust
        fy = Y
        fz = -L * math.cos(alpha) - D * math.sin(alpha)

        # moments
        mx = qbar * self.S * self.b * (self.Cl_beta * beta + self.aileron * 0.15)
        my = qbar * self.S * self.c * (self.Cm0 + self.Cm_alpha * alpha + self.Cm_q * (self.q * self.c / (2 * V)) + self.elevator * 0.4)
        mz = qbar * self.S * self.b * (self.Cn_beta * beta + self.rudder * 0.08)

        # gravity in body frame
        gx = -self.mass * 9.80665 * math.sin(self.pitch)
        gy = self.mass * 9.80665 * math.sin(self.roll) * math.cos(self.pitch)
        gz = self.mass * 9.80665 * math.cos(self.roll) * math.cos(self.pitch)
        fx += gx
        fy += gy
        fz += gz

        # equations of motion (body frame)
        du = (fx / self.mass) + self.r * v - self.q * w
        dv = (fy / self.mass) + self.p * w - self.r * u
        dw = (fz / self.mass) + self.q * u - self.p * v
        dp = (mx - (self.Izz - self.Iyy) * self.q * self.r) / self.Ixx
        dq = (my - (self.Ixx - self.Izz) * self.p * self.r) / self.Iyy
        dr = (mz - (self.Iyy - self.Ixx) * self.p * self.q) / self.Izz

        u += du * dt
        v += dv * dt
        w += dw * dt
        self.p += dp * dt
        self.q += dq * dt
        self.r += dr * dt

        # integrate orientation (body rates -> Euler)
        self.roll += (self.p + self.q * math.sin(self.roll) * math.tan(self.pitch) + self.r * math.cos(self.roll) * math.tan(self.pitch)) * dt
        self.pitch += (self.q * math.cos(self.roll) - self.r * math.sin(self.roll)) * dt
        self.yaw += (self.q * math.sin(self.roll) / math.cos(self.pitch) + self.r * math.cos(self.roll) / math.cos(self.pitch)) * dt if abs(math.cos(self.pitch)) > 1e-6 else 0.0

        # body -> NED
        vx, vy, vz = self._body_to_ned(u, v, w)
        self.vx, self.vy, self.vz = vx, vy, vz
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # ground contact
        alt = -self.z
        if alt <= 0.5 and self.vz > 0:
            self.on_ground = True
            self.z = -0.5
            self.vz = 0.0
            # friction / braking
            spd = math.hypot(self.vx, self.vy)
            if spd > 0:
                dec = 4.0 * self.brake + 0.5
                self.vx *= max(0.0, 1.0 - dec * dt / max(spd, 1.0))
                self.vy *= max(0.0, 1.0 - dec * dt / max(spd, 1.0))
            # kill rates on ground
            self.p *= 0.8
            self.q *= 0.8
            self.r *= 0.8
            # prevent pitch/roll too far
            self.pitch = clamp(self.pitch, deg2rad(-5), deg2rad(5))
            self.roll = clamp(self.roll, deg2rad(-3), deg2rad(3))
        else:
            self.on_ground = False

        # crash detection
        if alt < -5.0 or abs(self.roll) > math.pi * 0.85 or abs(self.pitch) > math.pi * 0.45:
            self.crashed = True

    def airspeed_kts(self):
        return math.hypot(self.vx, self.vy, self.vz) * 1.94384

    def altitude_ft(self):
        return (-self.z) * 3.28084

    def heading_deg(self):
        return (math.degrees(self.yaw) % 360.0)

    def vs_fpm(self):
        return -self.vz * 196.85

    def alpha_deg(self):
        return 0.0
