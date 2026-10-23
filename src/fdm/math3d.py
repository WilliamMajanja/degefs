"""Native 3D math (stdlib only) — SIMD-friendly flat storage."""

import math


class Vec3:
    __slots__ = ("x", "y", "z")

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __add__(self, o):
        return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o):
        return Vec3(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, s):
        if isinstance(s, Vec3):
            return Vec3(self.x * s.x, self.y * s.y, self.z * s.z)
        return Vec3(self.x * s, self.y * s, self.z * s)

    def __rmul__(self, s):
        return self.__mul__(s)

    def __truediv__(self, s):
        return Vec3(self.x / s, self.y / s, self.z / s)

    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)

    def dot(self, o):
        return self.x * o.x + self.y * o.y + self.z * o.z

    def cross(self, o):
        return Vec3(self.y * o.z - self.z * o.y,
                    self.z * o.x - self.x * o.z,
                    self.x * o.y - self.y * o.x)

    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def length2(self):
        return self.x * self.x + self.y * self.y + self.z * self.z

    def normalized(self):
        l = self.length()
        if l < 1e-12:
            return Vec3()
        inv = 1.0 / l
        return Vec3(self.x * inv, self.y * inv, self.z * inv)

    def lerp(self, o, t):
        return Vec3(self.x + (o.x - self.x) * t,
                    self.y + (o.y - self.y) * t,
                    self.z + (o.z - self.z) * t)

    def to_tuple(self):
        return (self.x, self.y, self.z)

    def __repr__(self):
        return f"Vec3({self.x}, {self.y}, {self.z})"


class Mat3:
    __slots__ = ("m",)

    def __init__(self, m=None):
        if m is None:
            self.m = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        else:
            self.m = [float(v) for v in m]

    def __mul__(self, o):
        a = self.m
        if isinstance(o, Mat3):
            b = o.m
            return Mat3([
                a[0]*b[0]+a[1]*b[3]+a[2]*b[6],
                a[0]*b[1]+a[1]*b[4]+a[2]*b[7],
                a[0]*b[2]+a[1]*b[5]+a[2]*b[8],
                a[3]*b[0]+a[4]*b[3]+a[5]*b[6],
                a[3]*b[1]+a[4]*b[4]+a[5]*b[7],
                a[3]*b[2]+a[4]*b[5]+a[5]*b[8],
                a[6]*b[0]+a[7]*b[3]+a[8]*b[6],
                a[6]*b[1]+a[7]*b[4]+a[8]*b[7],
                a[6]*b[2]+a[7]*b[5]+a[8]*b[8],
            ])
        if isinstance(o, Vec3):
            return Vec3(
                a[0]*o.x + a[1]*o.y + a[2]*o.z,
                a[3]*o.x + a[4]*o.y + a[5]*o.z,
                a[6]*o.x + a[7]*o.y + a[8]*o.z,
            )
        return NotImplemented

    def transposed(self):
        m = self.m
        return Mat3([m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]])

    def determinant(self):
        m = self.m
        return (m[0]*(m[4]*m[8]-m[5]*m[7])
                - m[1]*(m[3]*m[8]-m[5]*m[6])
                + m[2]*(m[3]*m[7]-m[4]*m[6]))

    @staticmethod
    def rotation_yaw_pitch_roll(yaw, pitch, roll):
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        return Mat3([
            cp*cy, cp*sy, -sp,
            sr*sp*cy - cr*sy, sr*sp*sy + cr*cy, sr*cp,
            cr*sp*cy + sr*sy, cr*sp*sy - sr*cy, cr*cp,
        ])

    @staticmethod
    def from_quat(q):
        # q: Quat
        w, x, y, z = q.w, q.x, q.y, q.z
        return Mat3([
            1-2*(y*y+z*z), 2*(x*y - w*z), 2*(x*z + w*y),
            2*(x*y + w*z), 1-2*(x*x+z*z), 2*(y*z - w*x),
            2*(x*z - w*y), 2*(y*z + w*x), 1-2*(x*x+y*y),
        ])


class Quat:
    __slots__ = ("w", "x", "y", "z")

    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def normalized(self):
        n = math.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
        if n < 1e-12:
            return Quat(1, 0, 0, 0)
        inv = 1.0 / n
        return Quat(self.w*inv, self.x*inv, self.y*inv, self.z*inv)

    def __mul__(self, o):
        if isinstance(o, Quat):
            return Quat(
                self.w*o.w - self.x*o.x - self.y*o.y - self.z*o.z,
                self.w*o.x + self.x*o.w + self.y*o.z - self.z*o.y,
                self.w*o.y - self.x*o.z + self.y*o.w + self.z*o.x,
                self.w*o.z + self.x*o.y - self.y*o.x + self.z*o.w,
            )
        return NotImplemented

    def rotate(self, v):
        return self.rotate_vec(v)

    def rotate_vec(self, v):
        qv = Quat(0, v.x, v.y, v.z)
        r = self * qv * self.conjugate()
        return Vec3(r.x, r.y, r.z)

    def conjugate(self):
        return Quat(self.w, -self.x, -self.y, -self.z)

    def to_euler(self):
        # returns (roll, pitch, yaw)
        sinr = 2*(self.w*self.x + self.y*self.z)
        cosr = 1 - 2*(self.x*self.x + self.y*self.y)
        roll = math.atan2(sinr, cosr)
        sinp = 2*(self.w*self.y - self.z*self.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi/2, sinp)
        else:
            pitch = math.asin(sinp)
        siny = 2*(self.w*self.z + self.x*self.y)
        cosy = 1 - 2*(self.y*self.y + self.z*self.z)
        yaw = math.atan2(siny, cosy)
        return roll, pitch, yaw

    @staticmethod
    def from_euler(roll, pitch, yaw):
        cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
        cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
        cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
        return Quat(
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
        ).normalized()

    @staticmethod
    def slerp(q1, q2, t):
        q1 = q1.normalized()
        q2 = q2.normalized()
        dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z
        if dot < 0:
            q2 = Quat(-q2.w, -q2.x, -q2.y, -q2.z)
            dot = -dot
        if dot > 0.9995:
            w = q1.w + t*(q2.w - q1.w)
            x = q1.x + t*(q2.x - q1.x)
            y = q1.y + t*(q2.y - q1.y)
            z = q1.z + t*(q2.z - q1.z)
            return Quat(w, x, y, z).normalized()
        theta0 = math.acos(clamp(dot, -1.0, 1.0))
        theta = theta0 * t
        sin0 = math.sin(theta0)
        s1 = math.cos(theta) - dot * math.sin(theta) / sin0
        s2 = math.sin(theta) / sin0
        return Quat(
            q1.w*s1 + q2.w*s2,
            q1.x*s1 + q2.x*s2,
            q1.y*s1 + q2.y*s2,
            q1.z*s1 + q2.z*s2,
        ).normalized()


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


class Mat4:
    __slots__ = ("m",)

    def __init__(self, m=None):
        if m is None:
            self.m = [1.0, 0.0, 0.0, 0.0,
                      0.0, 1.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, 0.0,
                      0.0, 0.0, 0.0, 1.0]
        else:
            self.m = [float(v) for v in m]

    def __mul__(self, o):
        a = self.m
        if isinstance(o, Mat4):
            b = o.m
            r = [0.0] * 16
            for i in range(4):
                ai = i * 4
                for k in range(4):
                    aik = a[ai + k]
                    if aik == 0.0:
                        continue
                    bk = k * 4
                    r[ai + 0] += aik * b[bk + 0]
                    r[ai + 1] += aik * b[bk + 1]
                    r[ai + 2] += aik * b[bk + 2]
                    r[ai + 3] += aik * b[bk + 3]
            return Mat4(r)
        if isinstance(o, Vec3):
            m = self.m
            x = m[0]*o.x + m[1]*o.y + m[2]*o.z + m[3]
            y = m[4]*o.x + m[5]*o.y + m[6]*o.z + m[7]
            z = m[8]*o.x + m[9]*o.y + m[10]*o.z + m[11]
            w = m[12]*o.x + m[13]*o.y + m[14]*o.z + m[15]
            if abs(w) > 1e-12:
                return Vec3(x/w, y/w, z/w)
            return Vec3(x, y, z)
        return NotImplemented

    def transposed(self):
        m = self.m
        return Mat4([m[0], m[4], m[8], m[12],
                     m[1], m[5], m[9], m[13],
                     m[2], m[6], m[10], m[14],
                     m[3], m[7], m[11], m[15]])

    @staticmethod
    def perspective(fovy, aspect, znear, zfar):
        f = 1.0 / math.tan(fovy * 0.5)
        nf = 1.0 / (znear - zfar)
        return Mat4([f/aspect, 0, 0, 0,
                     0, f, 0, 0,
                     0, 0, (zfar+znear)*nf, 2*zfar*znear*nf,
                     0, 0, -1, 0])

    @staticmethod
    def orthographic(l, r, b, t, n, f):
        return Mat4([2/(r-l), 0, 0, -(r+l)/(r-l),
                     0, 2/(t-b), 0, -(t+b)/(t-b),
                     0, 0, -2/(f-n), -(f+n)/(f-n),
                     0, 0, 0, 1])

    @staticmethod
    def look_at(eye, center, up):
        f = (center - eye).normalized()
        s = f.cross(up).normalized()
        u = s.cross(f)
        m = Mat4()
        m.m[0] = s.x; m.m[1] = s.y; m.m[2] = s.z; m.m[3] = -s.dot(eye)
        m.m[4] = u.x; m.m[5] = u.y; m.m[6] = u.z; m.m[7] = -u.dot(eye)
        m.m[8] = -f.x; m.m[9] = -f.y; m.m[10] = -f.z; m.m[11] = f.dot(eye)
        m.m[12] = 0; m.m[13] = 0; m.m[14] = 0; m.m[15] = 1
        return m

    @staticmethod
    def translation(x, y, z):
        return Mat4([1, 0, 0, x,
                     0, 1, 0, y,
                     0, 0, 1, z,
                     0, 0, 0, 1])

    @staticmethod
    def rotation_quat(q):
        r = Mat3.from_quat(q)
        m = r.m
        return Mat4([m[0], m[1], m[2], 0,
                     m[3], m[4], m[5], 0,
                     m[6], m[7], m[8], 0,
                     0, 0, 0, 1])

    @staticmethod
    def scale(sx, sy, sz):
        return Mat4([sx, 0, 0, 0,
                     0, sy, 0, 0,
                     0, 0, sz, 0,
                     0, 0, 0, 1])



    def transform_point(self, v):
        return self * v

    def transform_vector(self, v):
        m = self.m
        return Vec3(m[0]*v.x + m[1]*v.y + m[2]*v.z,
                    m[4]*v.x + m[5]*v.y + m[6]*v.z,
                    m[8]*v.x + m[9]*v.y + m[10]*v.z)

    def multiply(self, o):
        return self * o

    def inverse(self):
        # general 4x4 inverse via cofactors (basic)
        m = self.m
        # compute determinant
        def det3(a):
            return (a[0]*(a[4]*a[8]-a[5]*a[7]) - a[1]*(a[3]*a[8]-a[5]*a[6]) + a[2]*(a[3]*a[7]-a[4]*a[6]))
        # build cofactor
        cof = [0.0]*16
        for i in range(4):
            for j in range(4):
                minor = []
                for r in range(4):
                    if r == i: continue
                    row = []
                    for c in range(4):
                        if c == j: continue
                        row.append(m[r*4+c])
                    minor.append(row)
                # det of 3x3 minor
                a = [minor[0][0], minor[0][1], minor[0][2],
                     minor[1][0], minor[1][1], minor[1][2],
                     minor[2][0], minor[2][1], minor[2][2]]
                d = det3(a)
                cof[i*4+j] = ((-1)**(i+j)) * d
        det = sum(m[i]*cof[i] for i in range(4))  # wait cofactor transpose; approximate
        # Use adjugate transpose
        adj = [cof[j*4+i] for i in range(4) for j in range(4)]
        if abs(det) < 1e-12:
            return Mat4()
        invdet = 1.0/det
        return Mat4([adj[i]*invdet for i in range(16)])


class Frustum:
    def __init__(self, mvp=None):
        self.planes = []
        if mvp is not None:
            self.update(mvp)

    def update(self, mvp):
        m = mvp.m if hasattr(mvp, "m") else mvp
        self.planes = []
        # left, right, bottom, top, near, far
        self.planes.append((m[3]+m[0], m[7]+m[4], m[11]+m[8], m[15]+m[12]))
        self.planes.append((m[3]-m[0], m[7]-m[4], m[11]-m[8], m[15]-m[12]))
        self.planes.append((m[3]+m[1], m[7]+m[5], m[11]+m[9], m[15]+m[13]))
        self.planes.append((m[3]-m[1], m[7]-m[5], m[11]-m[9], m[15]-m[13]))
        self.planes.append((m[3]+m[2], m[7]+m[6], m[11]+m[10], m[15]+m[14]))
        self.planes.append((m[3]-m[2], m[7]-m[6], m[11]-m[10], m[15]-m[14]))

    def is_visible_point(self, p):
        return self.contains_point(p)

    def contains_point(self, p):
        for a, b, c, d in self.planes:
            if a*p.x + b*p.y + c*p.z + d < 0:
                return False
        return True

    def contains_sphere(self, center, radius):
        for a, b, c, d in self.planes:
            if a*center.x + b*center.y + c*center.z + d < -radius * math.sqrt(a*a+b*b+c*c):
                return False
        return True


# Backward/alternative names
Vector3 = Vec3
Matrix3 = Mat3
Matrix3x3 = Mat3
Matrix4x4 = Mat4
Quaternion = Quat

def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v


def slerp_quat(q1, q2, t):
    return Quat.slerp(q1, q2, t)
