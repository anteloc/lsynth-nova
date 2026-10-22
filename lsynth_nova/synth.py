"""Scan an LDraw model and synthesize every SYNTH block.  Port of lsynthcp.c main() scan loop,
synth_hose_class() and synth_band_class().

The model is processed as a line stream, like the C program: lines that are not part of a SYNTH block are
copied unchanged.  A block looks like

    0 SYNTH BEGIN <TYPE|PRODUCT> <colour> [GROUP=<name>]
    0 SYNTH SHOW | HIDE                       (optional: show/hide the constraint parts in the output)
    1 ... <constraint part>                   (hose constraints, or pulleys/gears for bands)
    0 SYNTH INSIDE | OUTSIDE | CROSS          (bands only: how the band passes the next constraint)
    0 SYNTH END

and the synthesized parts are inserted just before `0 SYNTH END`, between
`0 SYNTH SYNTHESIZED BEGIN` / `0 SYNTH SYNTHESIZED END`.
"""
from dataclasses import dataclass, field

from .config import Part, eq_ci, parse_type1, split_lines, startswith_ci, strclean
from . import band, hose

# C globals (lsynthcp.c).  The defaults are C float literals (0.05f, 0.0174f), kept bit-exact.
MAX_BEND = 0.05000000074505806
MAX_TWIST = 0.017400000244379044
BAND_RES = 1.0


class SynthError(ValueError):
    pass


def fmt(v):
    """C prints "%1.4f"; trailing zeros are trimmed (fewer tokens, same value) and -0 becomes 0."""
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s == "-0" else s


@dataclass
class Output:
    """Collects output lines.  Port of hose.c:output_line() and its global group_size counter."""
    ghost: bool = False
    group: str | None = None
    color: str = "16"
    lines: list = field(default_factory=list)
    group_size: int = 0

    def emit(self, line):
        self.lines.append(line)

    def part(self, offset, m, type_name):
        if self.group:
            self.emit(f"0 MLCAD BTG {self.group}")
        nums = " ".join(fmt(v) for v in (*offset, *m[0], *m[1], *m[2]))
        self.emit(f"{'0 GHOST ' if self.ghost else ''}1 {self.color} {nums} {type_name}")
        self.group_size += 1


class Reader:
    """Line source shared by the scan loop and the block collectors (C passes the FILE* around)."""

    def __init__(self, text):
        self.lines = split_lines(text)
        self.pos = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self.pos >= len(self.lines):
            raise StopIteration
        self.pos += 1
        return self.lines[self.pos - 1]


def _clean(line, nonwhite):
    """C strclean() applied to a pointer into the line: it rewrites the line itself."""
    cleaned = strclean(nonwhite)
    return line[:len(line) - len(nonwhite)] + cleaned, cleaned


def _is_synthesized_begin(s):
    # C quirk fixed: C only skipped the obsolete `0 SYNTHESIZED BEGIN` marker (and its skip loop read
    # 7 bytes at a time), so re-running on output duplicated the parts.  Skip both markers.
    return eq_ci(s, "0 SYNTH SYNTHESIZED BEGIN") or eq_ci(s, "0 SYNTHESIZED BEGIN")


def _skip_synthesized(rd):
    for line in rd:
        s = line.strip()
        if eq_ci(s, "0 SYNTH SYNTHESIZED END") or eq_ci(s, "0 SYNTHESIZED END"):
            return True
    return False


class Synthesizer:
    def __init__(self, config, ldraw_part=False, info=None, warn=None):
        self.cfg = config
        self.ldraw_part = ldraw_part   # C -l: format the output as an official LDraw part
        self.info = info or (lambda msg: None)
        self.warn = warn or (lambda msg: None)
        self.max_bend, self.max_twist, self.band_res = MAX_BEND, MAX_TWIST, BAND_RES

    def run(self, text):
        """Return the synthesized model text.  Raises SynthError on input LSynth cannot continue past."""
        rd = Reader(text)
        out = Output()
        for line in rd:
            nonwhite = line.lstrip(" \t")
            line, nonwhite = _clean(line, nonwhite)
            if startswith_ci(nonwhite, "0 SYNTH BEGIN "):
                self._block(line, nonwhite, rd, out)
                continue
            words = nonwhite.split()
            if startswith_ci(nonwhite, "0 SYNTH HOSE_RES ") and self._floats(words[4:6]):
                # C quirk fixed: C applied HOSE_RES only when exactly one value parsed (then twist was garbage).
                vals = self._floats(words[4:6])
                self.max_bend = vals[0]
                if len(vals) > 1:
                    self.max_twist = vals[1]
                if not self.ldraw_part:
                    out.emit(line)
            elif startswith_ci(nonwhite, "0 SYNTH BAND_RES ") and self._floats(words[4:5]):
                self.band_res = self._floats(words[4:5])[0]
                if not self.ldraw_part:
                    out.emit(line)
            else:
                out.emit(line)
        return "".join(ln + "\n" for ln in out.lines)

    @staticmethod
    def _floats(words):
        vals = []
        for w in words:
            try:
                vals.append(float(w))
            except ValueError:
                break
        return vals

    def _block(self, line, nonwhite, rd, out):
        """One `0 SYNTH BEGIN` block (the body of the C scan loop)."""
        lineno = rd.pos
        words = nonwhite[len("0 SYNTH BEGIN "):].split()
        if len(words) < 2:
            raise SynthError(f"line {lineno}: SYNTH BEGIN needs a type and a colour: {line.strip()}")
        token, color = words[0], words[1]
        group = None
        for w in line.split():
            if w.startswith("GROUP="):
                group = w[len("GROUP="):]  # C quirk fixed: C cut the line (and its newline) after the name
        product = self.cfg.find_product(token)
        if product:
            out.emit(f"0 LPUB PLI BEGIN SUB {product.name} {color}")
            method = product.method
        else:
            method = token
        if not self.ldraw_part:
            out.emit(line)

        if self.cfg.is_hose_type(method):
            done = self._hose_block(method, color, rd, out, group, lineno)
        elif self.cfg.is_band_type(method):
            done = self._band_block(method, color, rd, out, group, lineno)
        else:
            self.warn(f"line {lineno}: unknown synthesis type {token} (see lsynth --list)")
            done = False
        if product:
            out.emit("0 LPUB PLI END")  # C quirk fixed: C skipped this in stdin/stdout mode
        if done:
            self.info(f"Synthesized {product.name} ({product.nickname})" if product else f"Synthesized {method}")

    # --- hoses --------------------------------------------------------------------------------------

    def _hose_block(self, method, color, rd, out, group, lineno):
        """Port of lsynthcp.c:synth_hose_class(): gather hose constraints up to SYNTH END, then render.

        C skip_rot() only echoed `0 ROTATION C*` / `0 COLOR` lines; the final `else` does the same here.
        """
        constraints = []
        hide, ghost, group_count = True, False, 0
        for line in rd:
            nonwhite = line.lstrip(" \t")
            if startswith_ci(nonwhite, "0 GHOST "):
                ghost = True
                nonwhite = nonwhite[len("0 GHOST "):].lstrip(" \t")
            line, nonwhite = _clean(line, nonwhite)
            part = parse_type1(nonwhite)
            if part:
                if self.cfg.hose_constraint(part.type):
                    if not self.ldraw_part:
                        out.emit(("0 " if hide else "") + line)  # hidden constraints become comments
                    constraints.append(Part(part.type, part.offset, part.orient, 0.0))
                else:
                    if group:
                        out.emit(f"0 MLCAD BTG {group}")
                        group_count += 1
                    out.emit(line)
            elif eq_ci(nonwhite, "0 SYNTH HIDE"):
                out.emit(line)
                hide = True
            elif eq_ci(nonwhite, "0 SYNTH SHOW"):
                out.emit(line)
                hide = False
            elif startswith_ci(nonwhite, "0 MLCAD BTG "):
                if not group:  # C quirk: the BTG line itself is consumed
                    group = nonwhite[len("0 MLCAD BTG "):].rstrip()
            elif startswith_ci(nonwhite, "0 GROUP "):
                out.emit(line)
            elif _is_synthesized_begin(nonwhite):
                if not _skip_synthesized(rd):
                    break
            elif eq_ci(nonwhite, "0 SYNTH END"):
                hose_type = self.cfg.hose_type(method)
                if hose_type is None:
                    self.warn(f"line {lineno}: unknown hose type {method}")
                    rendered = False
                else:
                    if len(constraints) < 2:
                        self.warn(f"line {lineno}: {method} needs at least 2 visible hose constraints, found "
                                  f"{len(constraints)} (hidden constraints are ignored)")
                    rendered = len(constraints) >= 2
                    out.ghost, out.group, out.color = ghost, group, color
                    if not self.ldraw_part:
                        out.emit("0 SYNTH SYNTHESIZED BEGIN")
                    if rendered:
                        group_count = hose.render_hose(hose_type, constraints, self.cfg.hose_constraints,
                                                       self.max_bend, self.max_twist, out, group_count)
                    if group:
                        out.emit(f"0 GROUP {group_count} {group}")
                    if not self.ldraw_part:
                        out.emit("0 SYNTH SYNTHESIZED END")
                if not self.ldraw_part:
                    out.emit(line)
                return rendered
            else:
                out.emit(line)
        self.warn(f"line {lineno}: missing SYNTH END")
        return False

    # --- bands --------------------------------------------------------------------------------------

    def _band_block(self, method, color, rd, out, group, lineno):
        """Port of lsynthcp.c:synth_band_class(): gather band constraints and INSIDE/OUTSIDE/CROSS."""
        constraints = []
        hide, ghost = False, False
        for line in rd:
            nonwhite = line.lstrip(" \t")
            if startswith_ci(nonwhite, "0 GHOST "):
                ghost = True
                nonwhite = nonwhite[len("0 GHOST "):].lstrip(" \t")
            part = parse_type1(nonwhite)  # C checks for a part before strclean() here
            if part:
                if self.cfg.band_constraint_index(part.type) is not None:
                    out.emit(("0 " if hide else "") + line)  # C: written even with -l
                    constraints.append(band.BandConstraint(part))
                else:
                    out.emit(line)  # C quirk fixed: C silently dropped non-constraint parts in band blocks
                continue
            line, nonwhite = _clean(line, nonwhite)
            word = nonwhite.split()[2].upper() if len(nonwhite.split()) == 3 else ""
            if startswith_ci(nonwhite, "0 SYNTH ") and word in ("INSIDE", "OUTSIDE", "CROSS"):
                out.emit(line)
                constraints.append(band.BandConstraint(Part(word, [0.0] * 3, [[0.0] * 3 for _ in range(3)])))
            elif eq_ci(nonwhite, "0 SYNTH HIDE"):
                out.emit(line)
                hide = True
            elif eq_ci(nonwhite, "0 SYNTH SHOW"):
                out.emit(line)
                hide = False
            elif startswith_ci(nonwhite, "0 MLCAD BTG "):
                if not group:
                    group = nonwhite[len("0 MLCAD BTG "):].rstrip()
                out.emit(line)
            elif startswith_ci(nonwhite, "0 GROUP "):
                pass  # C quirk: dropped (LSynth writes its own GROUP line)
            elif _is_synthesized_begin(nonwhite):
                if not _skip_synthesized(rd):
                    break
            elif eq_ci(nonwhite, "0 SYNTH END"):
                band_type = self.cfg.band_type(method)
                rendered = False
                if band_type is None:
                    self.warn(f"line {lineno}: unknown band type {method}")
                elif not any(c.part.type.casefold() not in ("inside", "outside", "cross") for c in constraints):
                    self.warn(f"line {lineno}: {method} has no visible band constraints (see lsynth --list)")
                else:
                    out.ghost, out.group, out.color = ghost, group, color
                    band.synth_band(band_type, constraints, self.cfg.band_constraints, self.band_res,
                                    out, self.ldraw_part, self.warn, lineno)
                    rendered = True
                if not self.ldraw_part:
                    out.emit(line)
                return rendered
            else:
                out.emit(line)
        self.warn(f"line {lineno}: missing SYNTH END")
        return False
