"""Aircraft model: 6-DOF rigid body with blade-element aerodynamics.

Stdlib only. Concepts ported from FlightGear/JSBSim.
"""

import math
import sys
import os

try:
    from ..environment.atmosphere import (isa, clamp, deg2rad, rad2deg, speed_of_sound,
                                         kts_to_ms, ms_to_kts, m_to_ft, ft_to_m)
except Exception:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from src.environment.atmosphere import (isa, clamp, deg2rad, rad2deg, speed_of_sound,
                                           kts_to_ms, ms_to_kts, m_to_ft, ft_to_m)

from .math3d import Vec3, Mat3, Quat


class Aircraft:
    def __init__(self):
        # mass & inertia
        self.mass = 1100.0
        self.Ixx = 1285.0
        self.Iyy = 1741.0
        self.Izz = 2584.0
        # geometry
        self.S = 16.16
        self.b = 10.91
        self.c = 1.49
        # aerodynamic coefficients
        self.CL0 = 0.35
        self.CL_alpha = 5.2
        self.CD0 = 0.03
        self.induced_k = 0.045
        self.Cm0 = -0.02
        self.Cm_alpha = -0.8
        self.Cm_q = -8.0
        self.Cl_beta = -0.12
        self.Cn_beta = 0.08
        self.Cl_p = -0.8
        self.Cn_r = -0.2
        self.stall_alpha = deg2rad(16.0)
        # engine (piston)
        self.engine_type = "piston"
        self.max_thrust = 3000.0
        self.max_power = 110000.0  # W
        self.throttle = 0.0
        self.mixture = 1.0
        self.prop_pitch = 0.0  # -1..1
        self.engine_rpm = 800.0
        self.idle_rpm = 800.0
        self.max_rpm = 2700.0
        self.manifold_pressure = 10.0  # inHg
        self.fuel_flow = 0.0
        self.oil_temp = 80.0
        self.oil_pressure = 60.0
        self.egt = 600.0
        self.spool = 1.0
        # state (NED)
        self.x = 0.0
        self.y = 0.0
        self.z = -100.0
        self.vx = 40.0
        self.vy = 0.0
        self.vz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        # body velocities cached
        self.u = 40.0
        self.v = 0.0
        self.w = 0.0
        # controls
        self.elevator = 0.0
        self.aileron = 0.0
        self.rudder = 0.0
        self.flaps = 0.0
        self.brake = 0.0
        self.brake_left = 0.0
        self.brake_right = 0.0
        self.gear = True
        # ground / surface
        self.on_ground = False
        self.crashed = False
        self.surface = "tarmac"  # tarmac, grass, wet
        self._oleo_compression = 0.0
        self._tire_state = {}
        # wind
        self.wind_x = 0.0
        self.wind_y = 0.0
        self.wind_z = 0.0
        # aero state
        self.alpha = 0.0
        self.beta = 0.0
        self.airspeed = 40.0
        self.qbar = 0.0
        # quaternion orientation (optional)
        self.q_orient = Quat(1, 0, 0, 0)

    def _surface_friction(self):
        coeffs = {"tarmac": 0.8, "grass": 0.45, "wet": 0.35}
        return coeffs.get(self.surface, 0.7)

    def _pacejka(self, slip, Fz, mu):
        # simplified magic formula
        B = 10.0
        C = 1.9
        D = mu * Fz
        E = 0.97
        phi = B * slip
        return D * math.sin(C * math.atan(phi - E * (phi - math.atan(phi))))

    def _update_engine(self, dt):
        dt = max(dt, 1e-4)
        # target rpm from throttle
        target_rpm = self.idle_rpm + self.throttle * (self.max_rpm - self.idle_rpm)
        # mixture effect
        tau = 0.5
        self.engine_rpm += (target_rpm - self.engine_rpm) * (dt / (tau + dt))
        self.engine_rpm = clamp(self.engine_rpm, self.idle_rpm * 0.5, self.max_rpm)
        # manifold pressure (inHg) from throttle + rpm
        self.manifold_pressure = 10.0 + self.throttle * 18.0 + (self.engine_rpm / self.max_rpm) * 2.0
        # fuel flow gph (rough)
        self.fuel_flow = 2.0 + self.throttle * 8.0 * (self.engine_rpm / self.max_rpm)
        # oil temp
        self.oil_temp += (100.0 * self.throttle + 60.0 - self.oil_temp) * dt * 0.05
        self.oil_pressure = 30.0 + 30.0 * (self.engine_rpm / self.max_rpm)
        self.egt = 600.0 + self.throttle * 700.0 + (self.mixture - 1.0) * 200.0
        # spool for jet
        if self.engine_type == "jet":
            self.spool += (self.throttle - self.spool) * dt * 1.5
            self.spool = clamp(self.spool, 0.0, 1.0)

    def _body_transform(self):
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        Rbi = [
            [cp * cy, cp * sy, -sp],
            [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp],
            [cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp],
        ]
        return Rbi, (cr, sr, cp, sp, cy, sy)

    def _blade_element_aero(self, u, v, w, V, alpha, beta, rho, qbar):
        # multi-point-ish aggregate with stall model
        CL = self.CL0 + self.CL_alpha * alpha + self.flaps * 0.4
        if alpha > self.stall_alpha:
            CL = self.CL0 + self.CL_alpha * self.stall_alpha * (1.0 - (alpha - self.stall_alpha) / (self.stall_alpha + 1e-6))
        elif alpha < -self.stall_alpha * 0.8:
            CL = self.CL0 + self.CL_alpha * (-self.stall_alpha * 0.8) * (1.0 - (alpha + self.stall_alpha * 0.8) / (self.stall_alpha * 0.8 + 1e-6))
        CD = self.CD0 + self.induced_k * CL * CL + abs(self.aileron) * 0.01 + abs(self.flaps) * 0.02
        CY = -0.3 * beta + self.rudder * 0.05
        mx = qbar * self.S * self.b * (self.Cl_beta * beta + self.aileron * 0.15
              + self.Cl_p * (self.p * self.b / (2.0 * max(V, 1.0))) - 0.35 * self.roll)
        my = qbar * self.S * self.c * (self.Cm0 + self.Cm_alpha * alpha + self.Cm_q * (self.q * self.c / (2.0 * max(V, 1.0)))
              + self.elevator * 0.4 - 0.25 * self.pitch)
        mz = qbar * self.S * self.b * (self.Cn_beta * beta + self.rudder * 0.08 + self.Cn_r * (self.r * self.b / (2.0 * max(V, 1.0))))
        return CL, CD, CY, mx, my, mz

    def update(self, dt):
        if self.crashed:
            return
        dt = clamp(dt, 1e-4, 0.05)

        self.q_orient = Quat.from_euler(self.roll, self.pitch, self.yaw)
        self._update_engine(dt)

        alt = -self.z
        T, P, rho = isa(alt)
        rho = max(rho, 1e-3)
        # relative wind (NED)
        rx = self.vx - self.wind_x
        ry = self.vy - self.wind_y
        rz = self.vz - self.wind_z

        Rbi, (cr, sr, cp, sp, cy, sy) = self._body_transform()
        u = Rbi[0][0] * rx + Rbi[0][1] * ry + Rbi[0][2] * rz
        v = Rbi[1][0] * rx + Rbi[1][1] * ry + Rbi[1][2] * rz
        w = Rbi[2][0] * rx + Rbi[2][1] * ry + Rbi[2][2] * rz
        self.u, self.v, self.w = u, v, w
        V = math.sqrt(u*u + v*v + w*w)
        V = max(V, 0.1)
        self.airspeed = V
        alpha = math.atan2(w, u) if abs(u) > 1e-9 else 0.0
        # beta
        if V > 0.1:
            beta = math.asin(clamp(v / V, -1.0, 1.0))
        else:
            beta = 0.0
        self.alpha = alpha
        self.beta = beta
        qbar = 0.5 * rho * V * V
        self.qbar = qbar

        CL, CD, CY, mx, my, mz = self._blade_element_aero(u, v, w, V, alpha, beta, rho, qbar)

        # thrust
        if self.engine_type == "jet":
            thrust = self.spool * self.max_thrust * (rho / 1.225)
        else:
            # piston + prop
            thrust = (self.throttle * self.max_thrust * (rho / 1.225) *
                      clamp(1.0 - V / 90.0, 0.25, 1.0))
        # prop pitch effect
        thrust *= (1.0 + 0.1 * self.prop_pitch)
        thrust = max(0.0, thrust)

        # body-frame forces
        # body-frame forces via stability axes
        # Drag = qS CD, Lift = qS CL. In body frame: Fx = -D cos(alpha) + L sin(alpha) + thrust; Fz = -L cos(alpha) - D sin(alpha)
        D = qbar * self.S * CD
        L = qbar * self.S * CL
        fx = -D * math.cos(alpha) + L * math.sin(alpha) + thrust
        fy = qbar * self.S * CY
        fz = -L * math.cos(alpha) - D * math.sin(alpha)

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

        # integrate velocities (semi-implicit)
        u += du * dt
        v += dv * dt
        w += dw * dt
        self.p += dp * dt
        self.q += dq * dt
        self.r += dr * dt

        # clamp rates
        self.p = clamp(self.p, -4.0, 4.0)
        self.q = clamp(self.q, -4.0, 4.0)
        self.r = clamp(self.r, -4.0, 4.0)

        # integrate orientation via quaternion (avoid gimbal lock)
        # body rates -> quaternion derivative
        q = self.q_orient
        pq, qq, rq = self.p, self.q, self.r
        prod = Quat(0, pq, qq, rq) * q
        dwq = Quat(prod.w * 0.5, prod.x * 0.5, prod.y * 0.5, prod.z * 0.5)
        q.w += dwq.w * dt
        q.x += dwq.x * dt
        q.y += dwq.y * dt
        q.z += dwq.z * dt
        q = q.normalized()
        self.q_orient = q
        roll, pitch, yaw = q.to_euler()
        self.roll, self.pitch, self.yaw = roll, pitch, yaw

        # body -> NED velocity
        vx, vy, vz = self._body_to_ned(u, v, w, self.roll, self.pitch, self.yaw)
        self.vx, self.vy, self.vz = vx, vy, vz
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # ground contact
        alt = -self.z
        terrain_h = 0.0
        gear_height = 0.5
        if self.gear:
            gear_height = 0.5
        else:
            gear_height = 0.1
        if alt <= gear_height:
            self.on_ground = True
            # oleo compression
            comp = gear_height - alt
            self._oleo_compression = max(0.0, comp)
            self.z = -gear_height
            # prevent downward velocity
            if self.vz > 0:
                self.vz = 0.0
            # friction / braking
            spd = math.hypot(self.vx, self.vy)
            if spd > 0.01:
                mu = self._surface_friction()
                # normal force approx
                Fz = self.mass * 9.80665
                # pacejka longitudinal
                slip = (spd - 0.0) / max(spd, 1.0)
                Fbrake = self._pacejka(slip, Fz, mu * self.brake) if self.brake > 0 else 0.0
                dec = mu * 2.0 + 3.0 * self.brake + self.brake_left + self.brake_right
                # differential braking yaw moment
                if self.brake_left != self.brake_right:
                    self.r += (self.brake_right - self.brake_left) * 0.5 * dt
                factor = max(0.0, 1.0 - dec * dt / max(spd, 1.0))
                self.vx *= factor
                self.vy *= factor
            # kill rates
            self.p *= 0.75
            self.q *= 0.75
            self.r *= 0.75
            # clamp pitch/roll
            self.pitch = clamp(self.pitch, deg2rad(-8), deg2rad(8))
            self.roll = clamp(self.roll, deg2rad(-8), deg2rad(8))
            # sync quaternion
            self.q_orient = Quat.from_euler(self.roll, self.pitch, self.yaw)
        else:
            self.on_ground = False

        # crash detection
        alt = -self.z
        if alt < -10.0 or abs(self.roll) > math.pi * 0.9 or abs(self.pitch) > math.pi * 0.5 or V > 250.0:
            self.crashed = True

    def _body_to_ned(self, u, v, w, roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        x = (cp * cy) * u + (sr * sp * cy - cr * sy) * v + (cr * sp * cy + sr * sy) * w
        y = (cp * sy) * u + (sr * sp * sy + cr * cy) * v + (cr * sp * sy - sr * cy) * w
        z = (-sp) * u + (sr * cp) * v + (cr * cp) * w
        return x, y, z

    def to_dict(self):
        return {
            "mass": self.mass, "Ixx": self.Ixx, "Iyy": self.Iyy, "Izz": self.Izz,
            "S": self.S, "b": self.b, "c": self.c,
            "x": self.x, "y": self.y, "z": self.z,
            "vx": self.vx, "vy": self.vy, "vz": self.vz,
            "roll": self.roll, "pitch": self.pitch, "yaw": self.yaw,
            "p": self.p, "q": self.q, "r": self.r,
            "u": self.u, "v": self.v, "w": self.w,
            "throttle": self.throttle, "mixture": self.mixture, "prop_pitch": self.prop_pitch,
            "engine_rpm": self.engine_rpm, "engine_type": self.engine_type,
            "elevator": self.elevator, "aileron": self.aileron,
            "rudder": self.rudder, "flaps": self.flaps,
            "brake": self.brake, "brake_left": self.brake_left, "brake_right": self.brake_right,
            "gear": self.gear, "surface": self.surface,
            "crashed": self.crashed, "on_ground": self.on_ground,
            "wind_x": self.wind_x, "wind_y": self.wind_y, "wind_z": self.wind_z,
            "q_orient": [self.q_orient.w, self.q_orient.x, self.q_orient.y, self.q_orient.z],
        }

    def from_dict(self, d):
        for k, v in d.items():
            if k == "q_orient" and isinstance(v, (list, tuple)):
                self.q_orient = Quat(*v).normalized()
                continue
            if hasattr(self, k):
                setattr(self, k, v)
        return self

    def airspeed_kts(self):
        return math.hypot(self.vx - self.wind_x, self.vy - self.wind_y, self.vz - self.wind_z) * 1.94384

    def altitude_ft(self):
        return (-self.z) * 3.28084

    def heading_deg(self):
        return (math.degrees(self.yaw) % 360.0)

    def vs_fpm(self):
        return -self.vz * 196.85

    def alpha_deg(self):
        return math.degrees(self.alpha)

    def beta_deg(self):
        return math.degrees(self.beta)

    def to_vec_state(self):
        return Vec3(self.x, self.y, self.z)
