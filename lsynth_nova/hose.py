"""Hose-like synthesis: hoses, cables, flexible axles, string, minifig chain.  Port of hose.c and curve.c.

For each pair of consecutive constraints:
  1. synth_curve()        sample a cubic bezier between them (8192 points, re-spaced by arc length);
                          each constraint's -Y axis gives the direction the hose leaves/enters it.
  2. merge_segments_*()   reduce the samples: STRETCH by bend/twist angle, FIXED by part length,
                          FIXED<n> to exactly n parts over the whole hose (all pairs concatenated).
  3. orient()/orientq()   give every remaining segment a rotation (quaternion version for STRETCH/FIXED<n>).
  4. render_hose_segment() place start/mid/alt/end parts, stretched along Y for STRETCH hoses.

Points are [x, y, z] lists and the merges work *in place* on the full sample array, exactly like the C
globals: render_hose_segment() reads one point past the merged end (see the C quirk notes below).
"""
import math

import numpy as np

from .config import FIXED, STRETCH, Part
from .mathlib import (PI, axisangle2quat, cross_unit, dot, identity, mcopy, mmul, mneg, normalize,
                      normalizequat, quat2matrix, rot, rot_y, vadd, vlen, vsub)

MAX_SEGMENTS = 1024 * 8  # C: size of the oversampled curve
FLIP_Y = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]


def copy_part(p):
    return Part(p.type, list(p.offset), mcopy(p.orient), p.attrib)


def hose_length(n, pts):
    """C curve.c:hose_length(): length of the polyline through the first n points."""
    length = 0.0
    for i in range(n - 1):
        length += vlen(vsub(pts[i + 1], pts[i]))
    return length


def line_angle(va, vb):
    """C hose.c:line_angle(): angle between two vectors.  C quirk: an exact 180 degrees reads as 0."""
    denom = vlen(va) * vlen(vb)
    if abs(denom) < 1e-9:
        return 0.0
    theta = (va[0] * vb[0] + va[1] * vb[1] + va[2] * vb[2]) / denom
    return 0.0 if theta >= 1 or theta <= -1 else math.acos(theta)


def line_angle3(a, b, pts):
    """C hose.c:line_angle3(): angle between segments a->a+1 and b->b+1."""
    return line_angle(vsub(pts[a + 1], pts[a]), vsub(pts[b + 1], pts[b]))


# --- 1. curve ---------------------------------------------------------------------------------------

def synth_curve(start, end, n, attrib):
    """Port of curve.c:synth_curve(): n points from start to end.  attrib is the hose stiffness.

    Vectorised with numpy; every expression keeps the C operation order (elementwise IEEE ops, sequential
    sums), so the samples are identical to the C loops.
    """
    start_v = rot(start.orient, [0.0, attrib, 0.0])
    stop_v = rot(end.orient, [0.0, -attrib, 0.0])
    # Limit the stiffness to half the distance so the curve does not double back on itself.
    x = vlen(vsub(end.offset, start.offset))
    y = dot(start_v, stop_v)
    if x < attrib * 2 and y <= 0.001:
        attrib = x / 2.0
        start_v = rot(start.orient, [0.0, attrib, 0.0])
        stop_v = rot(end.orient, [0.0, -attrib, 0.0])

    p0, p3 = np.array(start.offset), np.array(end.offset)
    p1, p2 = np.array(vsub(start.offset, start_v)), np.array(vsub(end.offset, stop_v))

    def bezier(t):
        t = t[:, None]
        u = 1 - t
        return u * u * u * p0 + u * u * 3 * t * p1 + u * 3 * t * t * p2 + t * t * t * p3

    # First pass: uniform parameter steps, only used to measure the point-to-point distances.
    pts = bezier(np.arange(n) / n)
    d = pts[1:] - pts[:-1]
    ptp = np.sqrt(d[:, 0] * d[:, 0] + d[:, 1] * d[:, 1] + d[:, 2] * d[:, 2])
    ptp_sum = np.add.accumulate(ptp)[-1]
    # Second pass: parameter steps weighted by 1/distance (the spreadsheet columns E..H in the C comments).
    ratio = ptp * n / ptp_sum
    ratio[ratio == 0] = 1e-20
    i_time_sum = np.add.accumulate(1.0 / (n * ratio))[-1]
    foo = 1.0 / n / ratio / i_time_sum
    times = np.concatenate(([0.0], np.add.accumulate(foo)))
    return bezier(times).tolist()


# --- 2. merges (in place; return the new point count) -----------------------------------------------

def merge_segments_angular(start, end, pts, n, max_bend, max_twist):
    """Port of hose.c:merge_segments_angular(): merge points until the bend or the twist gets too big."""
    start_up = rot(start.orient, [1.0, 0.0, 0.0])
    cur_up = rot(start.orient, [1.0, 0.0, 0.0])
    end_up = rot(end.orient, [1.0, 0.0, 0.0])
    total_length = hose_length(n, pts)
    cur_length = 0.0
    a, b, m = 0, 1, 0
    while b < n - 1 and vlen(vsub(pts[a], pts[b])) < 1e-9:  # skip coincident leading points
        b += 1
    while b < n:
        if b < n - 1:
            cur_length += vlen(vsub(pts[b + 1], pts[b]))
        else:
            cur_length += (total_length - cur_length) / 2
        if cur_length != 0:
            normalized = cur_length / total_length
            left = 1 - normalized
            next_up = [start_up[k] * left + end_up[k] * normalized for k in range(3)]
            s = math.sqrt(next_up[0] * next_up[0] + next_up[1] * next_up[1] + next_up[2] * next_up[2])
            next_up = [next_up[0] / s, next_up[1] / s, next_up[2] / s]
            theta2 = line_angle(cur_up, next_up)
            # C quirk: at b == n-1 this reads pts[n], one past the samples (zero memory in C; a zero row here).
            theta1 = line_angle3(a, b, pts)
            if theta1 < max_bend and theta2 < max_twist:
                b += 1
            else:
                pts[m] = list(pts[a])
                m += 1
                a = b
                b += 1
                cur_up = next_up
        else:
            b += 1
    if m <= 2:
        pts[1] = list(pts[n - 1])
        m = 2
    elif b - a > 1:
        pts[m - 1] = list(pts[a])
    else:
        m -= 1
    return m


def merge_segments_length(pts, n, max_len):
    """Port of hose.c:merge_segments_length(): keep points about one fixed part length apart."""
    a, b, m = 0, 1, 1
    while b < n:
        if vlen(vsub(pts[a], pts[b])) + 0.5 < max_len:
            b += 1
        else:
            a = b
            pts[m] = list(pts[b])
            m += 1
            b += 1
    if m < 2:
        pts[1] = list(pts[n - 1])
        m = 2
    elif b - a > 1:
        pts[m - 1] = list(pts[a])
    else:
        m -= 1
    return m


def merge_segments_count(hose, end, pts, n, count):
    """Port of hose.c:merge_segments_count(): split the whole curve into `count` equal parts (FIXED<n>)."""
    length = 0.0
    for i in range(n - 1):
        length += vlen(vsub(pts[i], pts[i + 1]))
    if (hose.start.type.casefold() != hose.mid.type.casefold() or hose.end.type.casefold() != hose.mid.type.casefold()
            or hose.start.attrib != hose.mid.attrib or hose.end.attrib != hose.mid.attrib):
        # Different end parts are not part of the count and sit exactly on the constraints.
        len_s, len_e = hose.start.attrib, hose.end.attrib
        length = length - (len_s + len_e)
        length = length / (count - 1)
    else:
        length = length / (count - 1)
        len_s = len_e = length
    walked, m = 0.0, 1
    for i in range(n - 1):
        walked += vlen(vsub(pts[i], pts[i + 1]))
        if (walked + 0.05) > (((m - 1) * length) + len_s):
            pts[m] = list(pts[i + 1])
            m += 1
    if len_e != length:  # the end part did not match: place it at the end constraint
        pts[m - 1] = list(pts[n - 1])
    pts[m] = list(pts[n - 1])
    m += 1
    # Move the last point len_e back along the end constraint, to the far end of the end part.
    pts[m - 1] = vadd(pts[m - 1], rot(end.orient, [0.0, -len_e, 0.0]))
    return m


# --- 3. orientation ---------------------------------------------------------------------------------

def orient(start, end, n, pts):
    """Port of curve.c:orient(): front = segment direction, up interpolated between the constraints'
    -Z axes (their fins) along the hose.  Returns a rotation per segment (index n-1 unused)."""
    start_up = rot(start.orient, [0.0, 0.0, -1.0])
    end_up = rot(end.orient, [0.0, 0.0, -1.0])
    total_length = hose_length(n, pts)
    ori = [None] * n
    for i in range(n - 1):
        cur = hose_length(i, pts) / total_length
        up = [start_up[k] * (1 - cur) + end_up[k] * cur for k in range(3)]
        r = math.sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2])
        up = [up[0] / r, up[1] / r, up[2] / r]
        front = vsub(pts[i + 1], pts[i])
        r = math.sqrt(front[0] * front[0] + front[1] * front[1] + front[2] * front[2])
        front = [front[0] / r, front[1] / r, front[2] / r]
        side, _ = cross_unit(front, up)
        up, _ = cross_unit(side, front)
        side, _ = cross_unit(front, up)  # C make_rotation_pp(): columns are up, front, side
        ori[i] = [[up[k], front[k], side[k]] for k in range(3)]
    return ori


def get_turn_mat(M, a, b, T):
    """Port of curve.c:get_turn_mat(): turn M (in place) by the rotation taking direction a to b.

    T holds the previous turn axis (reused for a 180 degree turn); a 0 degree turn leaves M unchanged.
    """
    r = dot(b, a)
    r = math.acos(min(1.0, max(-1.0, r)))
    t, _ = cross_unit(a, b)
    if not normalize(t) or r == 0.0:
        if dot(b, a) < 0:
            t = list(T)
        else:
            return r
    q = axisangle2quat(t, r)
    normalizequat(q)
    m = mmul(quat2matrix(q), M)
    M[:] = m
    T[:] = t
    return r


def orientq(start, end, n, pts):
    """Port of curve.c:orientq(): incremental quaternion turns from the start constraint along the hose,
    then the remaining twist to the end constraint's fin is spread over the length.  Finally every
    segment is flipped in Y (constraints point -Y, hose parts grow +Y)."""
    start_v = rot(start.orient, [0.0, -1.0, 0.0])
    end_v = rot(end.orient, [0.0, -1.0, 0.0])
    normalize(start_v)
    normalize(end_v)
    start_up = rot(start.orient, [0.0, 0.0, -1.0])
    end_up = rot(end.orient, [0.0, 0.0, -1.0])
    normalize(start_up)
    normalize(end_up)

    ori = [None] * n
    t = [0.0, 0.0, 0.0]
    for i in range(n - 1):
        v = vsub(pts[i + 1], pts[i])
        normalize(v)
        if i:
            ori[i] = mcopy(ori[i - 1])
        else:
            ori[i] = mcopy(start.orient)
            t = list(start_up)
        get_turn_mat(ori[i], start_v, v, t)
        start_v = list(v)  # incremental turns

    # Extra twist: turn to the end direction, then measure the angle between our up and the end's fin.
    m = mcopy(ori[n - 2])
    get_turn_mat(m, start_v, end_v, t)
    up = rot(m, [0.0, 0.0, -1.0])
    normalize(up)
    r = math.acos(min(1.0, max(-1.0, dot(up, end_up))))
    t, _ = cross_unit(up, end_up)
    normalize(t)
    if dot(t, end_v) > 0.0:
        r = -r

    total_length = hose_length(n - 1, pts)
    for i in range(1, n):
        if r != 0.0 and total_length != 0:
            a = r * (hose_length(i, pts) / total_length)
            ori[i - 1] = mmul(ori[i - 1], rot_y(a))
        ori[i - 1] = mmul(ori[i - 1], FLIP_Y)
    return ori


# --- 4. rendering -----------------------------------------------------------------------------------

def render_hose_segment(hose, pts, ori, n, twist, first, last, out, group_size):
    """Port of hose.c:render_hose_segment(): one part per segment.  twist is a 1-item list (C pointer)."""
    for i in range(n - 1):
        if hose.fill != STRETCH:
            length = 1.0
        else:
            length = vlen(vsub(pts[i + 1], pts[i]))
            if n > 1:
                # Extend so the outer edges of the cross sections touch.  C quirk: for the last segment
                # this reads pts[n], one past the merged points (a stale sample, kept as in C).
                phi = math.sin(line_angle3(i + 1, i, pts)) * hose.diameter
                length += phi + phi
        m1 = [[1.0, 0.0, 0.0], [0.0, length, 0.0], [0.0, 0.0, 1.0]]  # stretch along Y

        if i == 0 and first:
            p = hose.start
            m2 = mcopy(p.orient) if p.attrib != 0 else mmul(p.orient, m1)  # fixed-size start: no stretch
        elif i == n - 2 and last:
            p = hose.end
            m2 = mcopy(p.orient) if p.attrib != 0 else mmul(p.orient, m1)
        elif (i & 1) and hose.alt is not None:
            p = hose.alt
            m2 = mmul(p.orient, m1)
        else:
            p = hose.mid
            m2 = mmul(p.orient, m1)

        m1 = identity()
        if hose.fill != STRETCH:  # twist helps with string and chains
            twist[0] += hose.twist
            if hose.fill > FIXED:  # FIXED<n>: a full twist per part
                angle = twist[0] * PI / 180
            else:  # half before and half after (draws the minifig chain 45 degrees off the start post)
                angle = twist[0] * PI / 360
                twist[0] += hose.twist
            m1 = rot_y(angle)
        m1 = mmul(m1, m2)
        m2 = mmul(ori[i], m1)
        pts[i] = vadd(pts[i], rot(m2, p.offset))
        out.part(pts[i], m2, p.type)
        if out.group:
            group_size += 1
    return group_size


def adjust_constraint(orig, last, hose_constraints):
    """Port of hose.c:adjust_constraint(): apply the constraint table's offset/orientation; the flip flag
    turns the final constraint around (e.g. flexible axle ends)."""
    part = copy_part(orig)
    for hc in hose_constraints:
        if part.type.casefold() == hc.type.casefold():
            part.offset = vadd(rot(hc.orient, hc.offset), orig.offset)
            part.orient = mmul(hc.orient, orig.orient)
            if hc.attrib and last:
                part.orient = mneg(part.orient)
            break
    return part


def render_hose(hose, constraints, hose_constraints, bend_res, twist_res, out, group_size):
    """Port of hose.c:render_hose(): synthesize the hose through all constraints (at least 2).

    Emits the parts through out.part() and returns the updated GROUP count.  The SYNTHESIZED BEGIN/END
    and GROUP lines are written by the caller (synth.py).
    """
    constraints = [copy_part(c) for c in constraints]
    twist = [0.0]

    if hose.fill == STRETCH:
        # Fixed-length start/end parts of a STRETCH hose: add a constraint one part length inside.
        if hose.start.attrib != 0.0:
            constraints.insert(0, copy_part(constraints[0]))
            constraints[1].offset = vadd(constraints[1].offset, rot(constraints[0].orient, [0.0, -hose.start.attrib, 0.0]))
        if hose.end.attrib != 0.0:
            constraints.append(copy_part(constraints[-1]))
            constraints[-2].offset = vadd(constraints[-2].offset, rot(constraints[-1].orient, [0.0, hose.end.attrib, 0.0]))

    nc = len(constraints)
    mid_constraint = constraints[0]
    seglist = []  # FIXED<n>: the curves of all pairs, merged at the end
    n = MAX_SEGMENTS
    for c in range(nc - 1):
        first = adjust_constraint(mid_constraint, False, hose_constraints)
        second = adjust_constraint(constraints[c + 1], c == nc - 2, hose_constraints)
        n = MAX_SEGMENTS // (nc - 1) if hose.fill > FIXED else MAX_SEGMENTS
        # n-1 curve points, the last slot gets the end constraint; one extra zero row models C's
        # read past the end of its global array.
        pts = synth_curve(first, second, n - 1, hose.stiffness) + [[0.0, 0.0, 0.0] for _ in range(MAX_SEGMENTS + 1 - (n - 1))]
        pts[n - 1] = list(second.offset)

        if hose.fill == STRETCH:
            n = merge_segments_angular(first, second, pts, n, bend_res, twist_res)
            pts[n - 1] = list(second.offset)
            mid_constraint = constraints[c + 1]
            ori = orientq(first, second, n, pts)
        elif hose.fill == FIXED:
            n = merge_segments_length(pts, n, hose.mid.attrib)
            mid_constraint = copy_part(constraints[c + 1])
            mid_constraint.offset = list(pts[n - 1])  # the next pair starts where the fixed parts ended
            ori = orient(first, second, n, pts)
        else:
            mid_constraint = copy_part(constraints[c + 1])
            mid_constraint.offset = list(pts[n - 2])
            seglist.extend(list(p) for p in pts[:n])

        if hose.fill <= FIXED:
            group_size = render_hose_segment(hose, pts, ori, n, twist, c == 0, c == nc - 2, out, group_size)

    if hose.fill > FIXED:
        first = adjust_constraint(constraints[0], False, hose_constraints)
        second = adjust_constraint(constraints[-1], True, hose_constraints)
        n = n * (nc - 1)
        seglist.append([0.0, 0.0, 0.0])
        n = merge_segments_count(hose, second, seglist, n, hose.fill)
        ori = orientq(first, second, n, seglist)
        group_size = render_hose_segment(hose, seglist, ori, n, twist, True, True, out, group_size)
    return group_size
