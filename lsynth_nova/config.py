"""Synthesis definitions (lsynth.mpd).  Port of lsynthcp.c:parse_descr().

lsynth.mpd is an LDraw MPD whose `0 SYNTH ...` meta lines describe:
  0 SYNTH PART <part.dat> <nickname> <type>          complete parts (LPub PLI annotation)
  0 SYNTH BEGIN DEFINE <type> HOSE <fill> <diameter> <stiffness> <twist>   + start, mid, end [, alt] parts
  0 SYNTH BEGIN DEFINE <type> BAND <fill> <scale> <thresh>                 + tangent, arc [, start/end trans]
  0 SYNTH BEGIN DEFINE HOSE CONSTRAINTS / BAND CONSTRAINTS                 + one type-1 line per constraint
The type-1 lines reuse the colour field as a number: a length (hose parts), a flip flag (hose constraints)
or a radius (band constraints).  See the comment block at the top of lsynth.mpd for the full description.
"""
import re
from dataclasses import dataclass
from pathlib import Path

from . import VERSION

DEFAULT_CONFIG = Path(__file__).parent / "data" / "lsynth.mpd"

# C fill codes (lsynthcp.h).  FIXED<n> is stored as n itself (n > 1), so "fill > FIXED" means FIXED<n>.
STRETCH, FIXED, FIXED3 = -1, 0, -2


@dataclass
class Part:
    """One LDraw type-1 line (C part_t).  orient is row-major, exactly as written in the line."""
    type: str
    offset: list
    orient: list
    attrib: float = 0.0


@dataclass
class HoseType:
    type: str
    fill: int
    diameter: float
    stiffness: float
    twist: float
    start: Part
    mid: Part
    end: Part
    alt: Part | None  # optional alternate mid part (e.g. STRING_WITH_GRIPS_21L)


@dataclass
class BandType:
    type: str
    fill: int
    scale: float
    thresh: float
    pulley: bool
    tangent: Part
    arc: Part
    start_trans: Part | None = None  # FIXED3 only
    end_trans: Part | None = None


@dataclass
class Product:
    name: str      # e.g. 32580.dat
    nickname: str  # e.g. PLI_TECHNIC_AXLE_FLEXIBLE_7
    method: str    # the hose/band type that synthesizes it


def parse_type1(text):
    """`1 <colour> x y z a b c d e f g h i <part>` -> Part, or None (C: sscanf(...) == 14).

    The colour field becomes Part.attrib; a non-numeric colour (e.g. a direct colour) reads as 0.
    """
    t = text.split()
    if len(t) < 15 or t[0] != "1":
        return None
    try:
        n = [float(x) for x in t[2:14]]
    except ValueError:
        return None
    try:
        attrib = float(t[1])
    except ValueError:
        attrib = 0.0
    return Part(t[14], n[0:3], [n[3:6], n[6:9], n[9:12]], attrib)


def startswith_ci(s, prefix):
    return s[:len(prefix)].casefold() == prefix.casefold()


def eq_ci(s, other):
    return s.strip().casefold() == other.casefold()


def split_lines(text):
    """C L3fgets(): split on \\r\\n, \\r or \\n only (str.splitlines() also splits on other characters);
    CTRL+Z characters are skipped."""
    lines = re.split(r"\r\n|\r|\n", text.replace("\x1a", ""))
    if lines and lines[-1] == "":
        lines.pop()
    return lines


def strclean(s):
    """C strclean(): MLCad's `0 WRITE <text>` becomes `0 <text>`."""
    return "0 " + s[8:] if startswith_ci(s, "0 WRITE ") else s


class ConfigError(ValueError):
    pass


@dataclass
class Config:
    version: str
    products: list
    hose_types: list
    band_types: list
    hose_constraints: list
    band_constraints: list

    # Lookups keep the C semantics: products and type routing match a case-insensitive *prefix* of the
    # token (isproduct/ishosetype/isbandtype); the definition itself is found by exact name (synth_hose,
    # synth_band).  First definition wins (lsynth.mpd defines ELECTRIC_CABLE twice).

    def find_product(self, token):
        for p in self.products:
            if startswith_ci(token, p.name) or startswith_ci(token, p.nickname):
                return p
        return None

    def is_hose_type(self, token):
        return any(startswith_ci(token, h.type) for h in self.hose_types)

    def is_band_type(self, token):
        return any(startswith_ci(token, b.type) for b in self.band_types)

    def hose_type(self, name):
        return next((h for h in self.hose_types if h.type.casefold() == name.casefold()), None)

    def band_type(self, name):
        return next((b for b in self.band_types if b.type.casefold() == name.casefold()), None)

    def hose_constraint(self, name):
        return next((c for c in self.hose_constraints if c.type.casefold() == name.casefold()), None)

    def band_constraint_index(self, name):
        return next((i for i, c in enumerate(self.band_constraints) if c.type.casefold() == name.casefold()), None)


def _parse_fill(word, kind, type_name):
    w = word.upper()
    if w == "STRETCH":
        return STRETCH
    if w == "FIXED":
        return FIXED
    if kind == "HOSE" and w.startswith("FIXED") and w[5:].isdigit() and int(w[5:]) > 1:
        return int(w[5:])  # FIXED<n>
    if kind != "HOSE" and w == "FIXED3":
        return FIXED3
    raise ConfigError(f"unrecognized fill type {word} for {kind.lower()} type {type_name}")


def load_config(path=None, warn=None):
    """Parse lsynth.mpd (default: the bundled one).  Raises ConfigError on malformed definitions."""
    path = Path(path) if path else DEFAULT_CONFIG
    try:
        raw = path.read_text(encoding="utf-8", errors="surrogateescape")
    except OSError as e:
        raise ConfigError(f"cannot read synthesis definitions {path}: {e.strerror}") from None
    lines = split_lines(raw)
    pos = 0

    def fgetline():
        """C fgetline(): next line, skipping MLCad ROTATION/COLOR lines, with `0 WRITE` cleaned."""
        nonlocal pos
        while pos < len(lines):
            line = lines[pos]
            pos += 1
            s = line.lstrip(" \t")
            if startswith_ci(s, "0 ROTATION C") or startswith_ci(s, "0 COLOR"):
                continue
            return strclean(s)
        return None

    def next_part(what):
        while (line := fgetline()) is not None:
            if (p := parse_type1(line)) is not None:
                return p
        raise ConfigError(f"{path}: unexpected end of file while reading {what}")

    cfg = Config("UNKNOWN", [], [], [], [], [])
    warned = False
    while (line := fgetline()) is not None:
        t = line.split()
        if len(t) >= 3 and t[0] == "0" and t[1] == "!VERSION":
            cfg.version = t[2]
        elif line.startswith("0 SYNTH ") and not warned:
            warned = True  # C warns once, at the first SYNTH line, if the versions differ.
            if cfg.version != VERSION and warn:
                warn(f"lsynth.mpd version {cfg.version} does not match lsynth version {VERSION}")

        up = [w.upper() for w in t]
        if up[:3] == ["0", "SYNTH", "PART"] and len(t) >= 6:
            cfg.products.append(Product(t[3], t[4], t[5]))

        elif up[:6] in (["0", "SYNTH", "BEGIN", "DEFINE", "HOSE", "CONSTRAINTS"],
                        ["0", "SYNTH", "BEGIN", "DEFINE", "BAND", "CONSTRAINTS"]) and len(t) == 6:
            table = cfg.hose_constraints if up[4] == "HOSE" else cfg.band_constraints
            while (line := fgetline()) is not None and not eq_ci(line, "0 SYNTH END"):
                if (p := parse_type1(line)) is not None:
                    table.append(p)

        elif up[:4] == ["0", "SYNTH", "BEGIN", "DEFINE"] and len(t) >= 10 and up[5] == "HOSE":
            name = t[4]
            fill = _parse_fill(t[6], "HOSE", name)
            start, mid, end = (next_part(f"hose type {name}") for _ in range(3))
            # C: every line up to SYNTH END overwrites the alternate part; a non-part line clears it.
            alt = None
            while (line := fgetline()) is not None and not eq_ci(line, "0 SYNTH END"):
                alt = parse_type1(line)
            if line is None:
                raise ConfigError(f"{path}: unexpected end of file in hose type {name}")
            if cfg.hose_type(name) is None:
                cfg.hose_types.append(HoseType(name, fill, float(t[7]), float(t[8]), float(t[9]),
                                               start, mid, end, alt))

        elif up[:4] == ["0", "SYNTH", "BEGIN", "DEFINE"] and len(t) >= 9 and up[5] in ("BAND", "PULLEY"):
            name = t[4]
            fill = _parse_fill(t[6], "BAND", name)
            n = 4 if fill == FIXED3 else 2
            parts = [next_part(f"band type {name}") for _ in range(n)]
            # C reads the very next raw line (no ROTATION skipping) and requires SYNTH END.
            nxt = lines[pos] if pos < len(lines) else None
            pos += 1
            if nxt is None or not eq_ci(strclean(nxt.lstrip(" \t")), "0 SYNTH END"):
                raise ConfigError(f"{path}: expected SYNTH END after band type {name}, got: {nxt}")
            # C quirk fixed: PULLEY definitions parse like BAND but without closing the loop
            # (the C PULLEY parser was broken; lsynth.mpd defines none).
            if cfg.band_type(name) is None:
                cfg.band_types.append(BandType(name, fill, float(t[7]), float(t[8]), up[5] == "PULLEY", *parts))
    return cfg
