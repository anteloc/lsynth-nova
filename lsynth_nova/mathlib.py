"""3-vector / 3x3-matrix helpers.  Port of mathlib.c plus the small vector helpers of curve.c.

Vectors are lists of 3 floats, matrices lists of 3 rows.  Every formula keeps the C evaluation order, so
results match the C reference bit for bit (plain IEEE doubles, libm via ``math``).  numpy is not used here
on purpose: its BLAS-backed dot/matmul may sum in a different order.
"""
import math

PI = 2 * math.atan2(1, 0)  # C: PRECISION pi = 2*atan2(1,0)


def identity():
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def mcopy(m):
    return [row[:] for row in m]


def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vsub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vlen(v):
    """C vectorlen()."""
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def dot(a, b):
    """C dotprod() (curve.c)."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def rot(m, v):
    """C vectorrot3(): m @ v."""
    return [v[0] * m[0][0] + v[1] * m[0][1] + v[2] * m[0][2],
            v[0] * m[1][0] + v[1] * m[1][1] + v[2] * m[1][2],
            v[0] * m[2][0] + v[1] * m[2][1] + v[2] * m[2][2]]


def mmul(a, b):
    """C matrixmult3(): a @ b.  The leading 0.0 mirrors C's accumulator (it fixes the sign of zeros)."""
    return [[0.0 + a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] for j in range(3)] for i in range(3)]


def mneg(m):
    return [[-x for x in row] for row in m]


def minv(src):
    """C matrixinv(): Gauss-Jordan with row pivoting, ported verbatim (singular input returns partial work)."""
    a = mcopy(src)
    p = [0, 0, 0]
    for k in range(3):
        sup = 0.0
        p[k] = 0
        for i in range(k, 3):
            s = 0.0
            for j in range(k, 3):
                s += abs(a[i][j])
            q = abs(a[i][k]) / s
            if sup < q:
                sup = q
                p[k] = i
        if sup == 0.0:
            return a
        if p[k] != k:
            a[k], a[p[k]] = a[p[k]], a[k]
        pivot = a[k][k]
        for j in range(3):
            if j != k:
                a[k][j] = -a[k][j] / pivot
                for i in range(3):
                    if i != k:
                        a[i][j] += a[i][k] * a[k][j]
        for i in range(3):
            a[i][k] /= pivot
        a[k][k] = 1 / pivot
    for k in (2, 1, 0):
        if p[k] != k:
            for i in range(3):
                a[i][k], a[i][p[k]] = a[i][p[k]], a[i][k]
    return a


def normalize(v):
    """C normalize() (curve.c): in place; returns False for a zero vector (left unchanged)."""
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n == 0.0:
        return False
    v[0] /= n
    v[1] /= n
    v[2] /= n
    return True


def cross_unit(lhs, rhs):
    """C mult_point() (curve.c): normalized cross product (unnormalized if zero) and its magnitude."""
    r = [lhs[1] * rhs[2] - lhs[2] * rhs[1],
         lhs[2] * rhs[0] - lhs[0] * rhs[2],
         lhs[0] * rhs[1] - lhs[1] * rhs[0]]
    tt = r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
    if tt > 0.0:
        tt = math.sqrt(tt)
        r = [r[0] / tt, r[1] / tt, r[2] / tt]
    return r, tt


def rot_y(angle):
    """The twist matrix used by hose.c/curve.c: rotation about Y."""
    return [[math.cos(angle), 0.0, math.sin(angle)], [0.0, 1.0, 0.0], [-math.sin(angle), 0.0, math.cos(angle)]]


# --- quaternions (curve.c) -------------------------------------------------------------------------

def axisangle2quat(v, a):
    a = a / 2.0
    s, c = math.sin(a), math.cos(a)
    return [v[0] * s, v[1] * s, v[2] * s, c]


def normalizequat(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n == 0.0:
        return
    for i in range(4):
        q[i] /= n


def quat2matrix(q):
    normalizequat(q)
    a, b, c, s = q
    return [[1 - 2 * b * b - 2 * c * c, 2 * a * b - 2 * s * c, 2 * a * c + 2 * s * b],
            [2 * a * b + 2 * s * c, 1 - 2 * a * a - 2 * c * c, 2 * b * c - 2 * s * a],
            [2 * a * c - 2 * s * b, 2 * b * c + 2 * s * a, 1 - 2 * a * a - 2 * b * b]]
