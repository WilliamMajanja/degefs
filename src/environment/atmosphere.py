"""Standard atmosphere (ISA) and wind/turbulence model — stdlib only."""

import math
import random

G0 = 9.80665
R = 287.05287
GAMMA = 1.4
T0 = 288.15
P0 = 101325.0
RHO0 = 1.225

# ISA layers: (base_alt, base_temp, base_pressure, lapse_rate K/m)
_LAYERS = [
    (0.0, 288.15, 101325.0, -0.0065),
    (11000.0, 216.65, 22632.1, 0.0),
    (20000.0, 216.65, 5474.89, 0.001),
    (32000.0, 228.65, 868.02, 0.0028),
    (47000.0, 270.65, 110.91, 0.0),
    (51000.0, 270.65, 66.94, -0.0028),
    (71000.0, 214.65, 3.96, -0.0020),
]


def _layer(alt):
    alt = max(-500.0, alt)
    for i in range(len(_LAYERS) - 1, -1, -1):
        base_alt, base_t, base_p, lapse = _LAYERS[i]
        if alt >= base_alt:
            return base_alt, base_t, base_p, lapse
    return _LAYERS[0]


def isa(alt_m):
    """Return (temperature K, pressure Pa, density kg/m^3) at geometric altitude."""
    base_alt, base_t, base_p, lapse = _layer(alt_m)
    dh = alt_m - base_alt
    if abs(lapse) < 1e-12:
        t = base_t
        p = base_p * math.exp(-G0 * dh / (R * base_t))
    else:
        t = base_t + lapse * dh
        if t <= 0:
            t = 1.0
        p = base_p * (t / base_t) ** (-G0 / (R * lapse))
    rho = p / (R * t)
    if rho <= 0 or not math.isfinite(rho):
        rho = 1e-6
    return float(t), float(p), float(rho)


def speed_of_sound(alt_m):
    t, _, _ = isa(alt_m)
    return math.sqrt(GAMMA * R * t)


def density_altitude(alt_m, rho):
    # inverse-ish: altitude where ISA density equals rho
    for a in range(-500, 12001, 50):
        _, _, r = isa(a)
        if r <= rho:
            return float(a)
    return 12000.0


def kts_to_ms(v):
    return v * 0.514444


def ms_to_kts(v):
    return v / 0.514444


def ft_to_m(v):
    return v * 0.3048


def m_to_ft(v):
    return v / 0.3048


def deg2rad(d):
    return d * math.pi / 180.0


def rad2deg(r):
    return r * 180.0 / math.pi


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def lerp(a, b, t):
    return a + (b - a) * t


# --- wind / turbulence model ---

_RNG = random.Random(1337)


def _smoothstep(t):
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def wind_profile(alt_m):
    """Mean wind speed (m/s) vs altitude (power law)."""
    return 4.0 + 0.002 * max(0.0, alt_m)


def wind_vector(x, y, z, t, base_speed=0.0, base_dir=0.0):
    """Return (wx, wy, wz) in m/s (NED). Includes shear, gusts, thermals, microbursts."""
    alt = max(0.0, -z)
    if base_speed <= 0:
        return 0.0, 0.0, 0.0
    ws = base_speed
    wd = base_dir
    # slow variation
    wd += 5.0 * math.sin(t * 0.01 + x * 1e-4)
    # gusts
    gust = 1.5 * math.sin(t * 0.7 + x * 0.001 + y * 0.001) + 0.8 * math.sin(t * 1.3 + y * 0.002)
    speed = max(0.0, ws + gust)
    rad = math.radians(wd)
    wx = speed * math.cos(rad)
    wy = speed * math.sin(rad)
    wz = 0.0
    # thermal updraft (localized)
    therm = 0.0
    for cx, cy in ((2000.0, 0.0), (-1500.0, 1200.0), (0.0, -3000.0)):
        d = math.hypot(x - cx, y - cy)
        if d < 800.0:
            therm += 3.0 * (1.0 - d / 800.0) * (0.5 + 0.5 * math.sin(t * 0.5))
    wz -= therm  # updraft is negative z (up) in NED? z down, so up = negative
    # microburst (rare, localized downdraft)
    mb_cx, mb_cy = 8000.0, 4000.0
    mbd = math.hypot(x - mb_cx, y - mb_cy)
    if mbd < 1200.0 and alt < 300.0:
        intensity = (1.0 - mbd / 1200.0) * 8.0
        # radial outflow near ground
        if mbd > 1e-3:
            rx = (x - mb_cx) / mbd
            ry = (y - mb_cy) / mbd
            wx += intensity * rx
            wy += intensity * ry
        wz += intensity * 2.0  # downdraft positive z
    # wind shear with altitude (linear)
    shear = 0.003 * alt
    wx += shear * math.sin(math.radians(wd + 90))
    wy += shear * math.cos(math.radians(wd + 90))
    return float(wx), float(wy), float(wz)


def turbulence(alt_m, airspeed, t):
    """Return (u,v,w) body-frame turbulence increments (m/s)."""
    sigma = 0.5 + 0.0005 * alt_m
    return (
        sigma * math.sin(t * 2.3 + alt_m * 0.01),
        sigma * math.sin(t * 1.7 + alt_m * 0.02),
        sigma * 0.6 * math.sin(t * 3.1),
    )


def wind(*args, **kwargs):
    return wind_vector(*args, **kwargs)
