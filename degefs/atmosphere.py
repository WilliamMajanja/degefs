"""Standard atmosphere (ISA) and unit helpers — stdlib only."""

import math

G0 = 9.80665
R = 287.05287
LAPSE = 0.0065
T0 = 288.15
P0 = 101325.0
RHO0 = 1.225


def isa(alt_m):
    """Return (temperature K, pressure Pa, density kg/m^3) at geometric altitude."""
    if alt_m < 11000:
        t = T0 - LAPSE * alt_m
        p = P0 * (t / T0) ** (G0 / (R * LAPSE))
    elif alt_m < 20000:
        t = 216.65
        p = 22632.0 * math.exp(-G0 * (alt_m - 11000) / (R * t))
    else:
        t = 216.65 + 0.001 * (alt_m - 20000)
        p = 5474.9 * (t / 216.65) ** (-G0 / (R * 0.001))
    rho = p / (R * t)
    return t, p, rho


def speed_of_sound(alt_m):
    t, _, _ = isa(alt_m)
    return math.sqrt(1.4 * R * t)


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
    return max(lo, min(hi, v))
