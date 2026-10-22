"""Band synthesis: rubber bands and belts, string loops, chains, treads.  Port of band.c.

The band runs around its constraints (pulleys, gears, wheels) in order and closes back to the first one.
Everything is computed in the X/Y plane of the first constraint:
  1. move/rotate all constraints into that plane (steps 1-5 in synth_band);
  2. calc_tangent_line()  the straight line from each constraint to the next (INSIDE/OUTSIDE/CROSS
                          choose which of the four common tangents);
  3. calc_crosses()       where straight lines cross, lift them apart in Z;
  4. calc_angles()        the arc to wrap around each constraint;
  5. draw_arc_line()      emit tangent and arc parts and rotate them back into the model (steps 4-9).
"""
import copy
import math
from dataclasses import dataclass, field

from .config import FIXED, FIXED3, STRETCH, Part
from .mathlib import PI, dot, mcopy, minv, mmul, normalize, rot, vadd, vsub

ACCY = 1e-6     # C lsynthcp.h
BAND_DIAM = 4   # C band.c: Z distance between crossing band lines


@dataclass
class BandConstraint:
    """C LSL_band_constraint.  INSIDE/OUTSIDE/CROSS markers are entries with radius 0."""
    part: Part
    radius: float = 0.0
    inside: int = 0
    was_cross: int = 0  # C quirk: never set by synth_band(); kept so calc_tangent_line() reads as in C
    cross: int = 0
    start_line: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    end_line: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    crossings: list = field(default_factory=list)
    layer: int = -1
    start_angle: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    end_angle: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    n_steps: int = 0
    s_angle: float = 0.0
    band_constraint_n: int = 0


def turn_vector(v1, v2):
    """C turn_vector(): 1 if v2 bends left of v1, -1 if right, 0 if straight (X/Y only)."""
    p = (v1[0] * v2[1]) - (v1[1] * v2[0])
    return 1 if p > 0.0 else -1 if p < 0.0 else 0


def calc_crosses(k, m, layer):
    """Port of band.c:calc_crosses(): record where the lines k and m cross.  Returns the next layer."""
    xlk = k.end_line[0] - k.start_line[0]
    ylk = k.end_line[1] - k.start_line[1]
    xnm = m.end_line[0] - m.start_line[0]
    ynm = m.end_line[1] - m.start_line[1]
    xmk = m.start_line[0] - k.start_line[0]
    ymk = m.start_line[1] - k.start_line[1]
    det = xnm * ylk - ynm * xlk
    if abs(det) < ACCY:
        return layer  # parallel lines
    detinv = 1.0 / det
    s = (xnm * ymk - ynm * xmk) * detinv
    t = (xlk * ymk - ylk * xmk) * detinv
    if 0 <= s <= 1.0 and 0 <= t <= 1.0:
        x = k.start_line[0] + xlk * s
        y = k.start_line[1] + ylk * s
        for c in (k, m):
            if len(c.crossings) < 8:  # C: at most 8 crossings per line
                c.crossings.append([x, y, 0.0])
        for c in (k, m):
            if c.layer == -1:
                c.layer = layer
                layer += 1
    return layer


def calc_angles(band_type, k, band_res):
    """Port of band.c:calc_angles(): the start angle and the number of arc steps around constraint k."""
    if k.cross or not k.inside:
        first, last = k.end_angle, k.start_angle
    else:
        first, last = k.start_angle, k.end_angle
    dx = (first[0] - k.part.offset[0]) / k.radius
    dy = (first[1] - k.part.offset[1]) / k.radius
    dx = min(1.0, max(-1.0, dx))
    angle = math.acos(dx) if dy > 0 else -math.acos(dx)

    # Turn angle between the incoming and outgoing tangent points (C: USE_TURN_ANGLE).
    if k.cross or not k.inside:
        a, b = list(k.end_angle), list(k.start_angle)
    else:
        b, a = list(k.end_angle), list(k.start_angle)
    a = vsub(a, k.part.offset)
    b = vsub(b, k.part.offset)
    normalize(a)
    normalize(b)
    r = min(1.0, max(-1.0, dot(b, a)))
    ta = r
    r = math.acos(r)
    i = turn_vector(b, a)
    if (ta + ACCY) > 1.0:  # round-off near a 0 degree turn
        i = 0
    if i > 0:
        r = 2 * PI - r
    elif i == 0:
        if ta < 0:
            r = PI
        elif k.layer == -2:  # the only constraint: wrap all the way round
            r = 2 * PI
        else:
            r = 0
    ta = r
    k.s_angle = angle

    if band_type.fill == FIXED3 or band_type.fill > FIXED:
        k.n_steps = int(ta * k.radius * band_type.scale + 0.5)
    elif band_type.fill == FIXED:
        k.n_steps = int(ta * k.radius * band_type.scale + 0.5) + 1  # segment end points: one more
    else:  # STRETCH: step round the circumference until close to where the band leaves
        n = 2 * PI * k.radius / band_res + 0.5  # C quirk: a float (not int) loop bound
        i = 0
        while i < n:
            f = i / n
            t = angle - 2 * PI * (1 - f)
            dx = k.radius * math.cos(t) + k.part.offset[0] - last[0]
            dy = k.radius * math.sin(t) + k.part.offset[1] - last[1]
            if math.sqrt(dx * dx + dy * dy) < band_type.thresh:
                break
            i += 1
        k.n_steps = i + 1


def intersect_line_circle_2D(xo, yo, f, g, xj, yj, rj):
    """Port of band.c:intersect_line_circle_2D(): the point on line (xo + f*t, yo + g*t) nearest the
    circle centre (the tangent point)."""
    fgsq = f * f + g * g
    xjo = xj - xo
    yjo = yj - yo
    t = (f * xjo + g * yjo) / fgsq
    return [xo + f * t, yo + g * t, 0.0]


def calc_tangent_line(k, l):
    """Port of band.c:calc_tangent_line(): tangent from circle k to circle l.  False if none exists."""
    inside1, inside2 = k.inside, l.inside
    if l.was_cross:
        if l.cross:
            inside2 ^= 1
        else:
            inside1 ^= 1
    rl, rk = l.radius, k.radius
    case = (inside1 << 1) | inside2
    if case == 2:    # inside to outside
        rl = -rl
    elif case == 1:  # outside to inside
        rk = -rk
    elif case == 0:  # outside to outside
        rk, rl = -rk, -rl
    rlk = rl - rk
    xlk = l.part.offset[0] - k.part.offset[0]
    ylk = l.part.offset[1] - k.part.offset[1]
    denom = xlk * xlk + ylk * ylk
    if denom < ACCY:
        return False  # coincident circles
    root = denom - rlk * rlk
    if root < -ACCY:
        return False  # circles overlap: this tangent does not exist
    root = math.sqrt(max(root, 0.0))
    deninv = 1.0 / denom
    a = (-rlk * xlk - ylk * root) * deninv
    b = (-rlk * ylk + xlk * root) * deninv
    c = -(rk + a * k.part.offset[0] + b * k.part.offset[1])
    # normalized form a*x + b*y + c = 0 -> parametric x = xo + f*t, y = yo + g*t
    root = 1.0 / (a * a + b * b)
    factor = -c * root
    xo, yo = a * factor, b * factor
    root = math.sqrt(root)
    f, g = b * root, -a * root
    k.start_line = intersect_line_circle_2D(xo, yo, f, g, k.part.offset[0], k.part.offset[1], k.radius)
    k.start_angle = list(k.start_line)
    k.end_line = intersect_line_circle_2D(xo, yo, f, g, l.part.offset[0], l.part.offset[1], l.radius)
    l.end_angle = list(k.end_line)
    return True


def _segment_orient(dx, dy, dz):
    """Orientation of a part along (dx, dy, dz) in the X/Y plane (shared by tangent and arc code)."""
    L1 = math.sqrt(dx * dx + dy * dy)
    L2 = math.sqrt(dx * dx + dy * dy + dz * dz)
    if L1 == 0:
        return [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]], L1, L2
    return [[dy / L1, dx / L2, -dx * dz / (L1 * L2)],
            [-dx / L1, dy / L2, -dy * dz / (L1 * L2)],
            [0.0, dz / L2, L1 / L2]], L1, L2


def _place(part, table_entry, absolute):
    """C draw_arc_line() steps 4-9: back from the first constraint's X/Y plane into the model."""
    inv = minv(table_entry.orient)
    offset = vsub(part.offset, table_entry.offset)
    offset = rot(absolute.orient, rot(inv, offset))
    part.offset = vadd(offset, absolute.offset)
    part.orient = mmul(absolute.orient, mmul(inv, part.orient))


def draw_arc_line(band_type, k, draw_line, out, absolute, first_entry, band_res):
    """Port of band.c:draw_arc_line(): the straight line(s) leaving k, then the arc around k."""
    fixed_n = band_type.fill == FIXED3 or band_type.fill > FIXED
    if draw_line:
        for j in range(len(k.crossings) - 1):
            p, q = k.crossings[j], k.crossings[j + 1]
            dx, dy, dz = q[0] - p[0], q[1] - p[1], q[2] - p[2]
            orient, L1, L2 = _segment_orient(dx, dy, dz)
            if band_type.fill == STRETCH:
                n, steps = 1, 0
            else:
                n, steps = int(L2 * band_type.scale + 0.5), (1 if fixed_n else 0)
            for i in range(steps, n):
                part = copy.deepcopy(band_type.tangent)
                if band_type.fill == STRETCH:  # one tangent part stretched to the full length
                    part.orient = mmul(part.orient, [[1.0, 0.0, 0.0], [0.0, L2, 0.0], [0.0, 0.0, 1.0]])
                part.orient = mmul(orient, part.orient)
                part.offset = rot(part.orient, band_type.tangent.offset)
                part.offset = [p[0] + dx * i / n - part.offset[0], p[1] + dy * i / n - part.offset[1],
                               p[2] + dz * i / n - part.offset[2]]
                _place(part, first_entry, absolute)
                out.part(part.offset, part.orient, band_type.tangent.type)

    # The arc around the constraint.
    if band_type.fill == STRETCH:
        n = int(2 * PI * k.radius / band_res)
    else:
        n = int(2 * PI * k.radius * band_type.scale)
    if n == 0:
        return False  # radius too small for this band type (C divides by zero here)
    f = [k.radius * math.cos(k.s_angle), k.radius * math.sin(k.s_angle), 0.0]
    steps = k.n_steps + 2 if fixed_n else k.n_steps
    for i in range(1, steps):
        # C quirk kept: the transition part of the first step is overwritten (missing `else` in C),
        # so FIXED3 bands only use end_trans.
        if fixed_n and i + 1 == steps:
            src = band_type.end_trans
        else:
            src = band_type.arc
        part = copy.deepcopy(src)
        s = [k.radius * math.cos(k.s_angle + 2 * PI * i / n), k.radius * math.sin(k.s_angle + 2 * PI * i / n), 0.0]
        dx, dy, dz = s[0] - f[0], s[1] - f[1], s[2] - f[2]
        orient, L1, L2 = _segment_orient(dx, dy, dz)
        if band_type.fill == STRETCH:
            length = band_type.scale * math.sin(2 * PI / n)
            if i + 1 == steps:
                length *= 5
            part.orient = mmul(part.orient, [[1.0, 0.0, 0.0], [0.0, L2 + length, 0.0], [0.0, 0.0, 1.0]])
        part.orient = mmul(orient, part.orient)
        part.offset = rot(part.orient, band_type.arc.offset)  # C: arc offset even for transition parts
        part.offset = [f[0] + k.part.offset[0] - part.offset[0], f[1] + k.part.offset[1] - part.offset[1],
                       f[2] + 0 - part.offset[2]]
        _place(part, first_entry, absolute)
        out.part(part.offset, part.orient, part.type)
        f = s
    return True


def _rotate_constraints(constraints, m):
    for c in constraints:
        c.part.offset = rot(m, c.part.offset)
        c.part.orient = mmul(m, c.part.orient)


def synth_band(band_type, constraints, table, band_res, out, ldraw_part, warn, lineno):
    """Port of band.c:synth_band().  constraints: BandConstraint list in model order (with markers)."""
    inside, cross, was_cross = 1, 0, 0
    first = -1
    for i, k in enumerate(constraints):
        k.radius, k.inside, k.cross, k.was_cross = 0.0, inside, cross, was_cross
        k.crossings, k.layer = [], -1
        kind = k.part.type.upper()
        if kind == "INSIDE":
            inside = 1
        elif kind == "OUTSIDE":
            inside = 0
        elif kind == "CROSS":
            inside ^= 1
        else:
            for idx, entry in enumerate(table):
                if k.part.type.casefold() == entry.type.casefold():
                    k.band_constraint_n = idx
                    k.radius = entry.attrib
                    break
        if first == -1 and k.radius:
            first = i
    if len(constraints) == 1:
        constraints[first].layer = -2  # the only constraint
    if not band_type.pulley:  # an extra constraint for the final state of the first pulley: close the loop
        constraints.append(copy.deepcopy(constraints[first]))
        constraints[-1].inside, constraints[-1].cross, constraints[-1].was_cross = inside, cross, was_cross
    n = len(constraints)
    absolute = copy.deepcopy(constraints[first].part)  # the first constraint, as placed in the model
    first_entry = table[constraints[first].band_constraint_n]

    # 1. Move the first constraint to the origin.
    for c in constraints:
        if c.radius:
            c.part.offset = vsub(c.part.offset, absolute.offset)
    # 2. Undo the first constraint's orientation.
    _rotate_constraints(constraints, minv(absolute.orient))
    # 3. Bring its gear plane into X/Y (e.g. technic turntables are modelled in another plane).
    _rotate_constraints(constraints, first_entry.orient)
    # 4. Put the centre of the first constraint's axle at the origin.
    for c in constraints:
        if c.radius:
            c.part.offset = vadd(c.part.offset, first_entry.offset)
    # 5. Subtract every constraint's own table offset (rotated by its orientation); flatten to Z = 0.
    inv = minv(constraints[first].part.orient)
    for c in constraints:
        if c.radius:
            rotoffset = rot(c.part.orient, rot(inv, table[c.band_constraint_n].offset))
            c.part.offset = vsub(c.part.offset, rotoffset)
            c.part.offset[2] = 0.0

    # Tangent lines between consecutive constraints.
    first = last = -1
    i = 0
    while i < n - 1:
        if constraints[i].radius:
            if first == -1:
                first = i
            for j in range(i + 1, n):
                if constraints[j].radius:
                    if not calc_tangent_line(constraints[i], constraints[j]):
                        warn(f"line {lineno}: {band_type.type}: no tangent between constraints {i + 1} and "
                             f"{j + 1} (they overlap); the band will be wrong")
                    i = last = j
                    break
            else:
                i += 1  # C quirk fixed: C would loop forever here
        else:
            i += 1
    constraints[first].end_angle = list(constraints[last].end_angle)

    # Crossing straight lines go round each other: lift them to different Z layers.
    for c in constraints:
        if c.radius:
            c.crossings = [list(c.start_line)]
    layer = 0
    for i in range(n - 1):
        if constraints[i].radius:
            for j in range(i + 1, n):
                if constraints[j].radius:
                    layer = calc_crosses(constraints[i], constraints[j], layer)
    if layer > 0:
        layer_offset = (((layer // 2) % 2) * BAND_DIAM) // 2
        for c in constraints[:n - 1]:
            if c.radius:
                for p in c.crossings[1:]:
                    p[2] = (c.layer - layer // 2) * BAND_DIAM + layer_offset
    for c in constraints[:n - 1]:
        if c.radius:
            c.crossings.append(list(c.end_line))
    for c in constraints[:n - 1]:
        if c.radius:
            calc_angles(band_type, c, band_res)

    # Draw lines and arcs, back in the model's coordinates.
    if not ldraw_part:
        out.emit("0 SYNTH SYNTHESIZED BEGIN")
    out.group_size = 0
    i = 0
    while i < n:
        if constraints[i].radius:
            for j in range(i + 1, n):
                if constraints[j].radius:
                    if not draw_arc_line(band_type, constraints[i], n > 1, out, absolute, first_entry, band_res):
                        warn(f"line {lineno}: {band_type.type}: constraint {constraints[i].part.type} is too small "
                             f"for this band type")
                    i = j
                    break
            else:
                i += 1
        else:
            i += 1
    if out.group:
        out.emit(f"0 GROUP {out.group_size} {out.group}")
    if not ldraw_part:
        out.emit("0 SYNTH SYNTHESIZED END")
