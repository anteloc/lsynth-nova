"""Command line interface.  Port of lsynthcp.c main() and usage().

    lsynth model.ldr out.ldr        synthesize (SRC/DST may be "-" for stdin/stdout; DST defaults to stdout)
    lsynth --list                   what can be synthesized, and with which constraints
    lsynth --install-parts [DIR]    make the LSynth parts (LS*.dat) available to LDraw renderers
"""
import argparse
import os
import shutil
import sys
from pathlib import Path

from . import VERSION
from .config import FIXED, FIXED3, STRETCH, ConfigError, load_config
from .synth import SynthError, Synthesizer

PARTS_DIR = Path(__file__).parent / "data" / "parts"
ENCODING = dict(encoding="utf-8", errors="surrogateescape")  # pass any bytes through unchanged


def _say(msg):
    print(f"lsynth: {msg}", file=sys.stderr)


def _fill_name(fill):
    return {STRETCH: "STRETCH", FIXED: "FIXED", FIXED3: "FIXED3"}.get(fill, f"FIXED{fill}")


def list_definitions(cfg):
    """C usage() listings: products, hose/band types and their constraints."""
    print("Hose types (constraints in order from start to end; each constraint's -Y axis points along the hose):")
    for h in cfg.hose_types:
        print(f"  {h.type:38s} {_fill_name(h.fill)}")
    print("\nBand types (constraints in order around the loop; 0 SYNTH INSIDE/OUTSIDE/CROSS between them):")
    for b in cfg.band_types:
        print(f"  {b.type:38s} {_fill_name(b.fill)}")
    print("\nProducts (LPub PLI part; synthesized as the given type):")
    for p in cfg.products:
        print(f"  {p.nickname:38s} {p.name:12s} {p.method}")
    print("\nHose constraints:")
    print("  " + " ".join(c.type for c in cfg.hose_constraints))
    print("\nBand constraints (radius in LDU):")
    for c in cfg.band_constraints:
        print(f"  {c.type:14s} {c.attrib:g}")


def _child(parent, name):
    """An existing child directory matching name in any letter case, else parent/name (lowercase)."""
    if parent.is_dir():
        for entry in parent.iterdir():
            if entry.is_dir() and entry.name.casefold() == name:
                return entry
    return parent / name


def install_parts(target):
    """Copy the bundled LS*.dat parts into target (default $LDRAW_DIR/unofficial/parts); never overwrite."""
    if not target:
        lib = os.environ.get("LDRAW_DIR") or os.environ.get("LDRAWDIR")
        if not lib:
            _say("--install-parts needs a DIR, or LDRAW_DIR set to your LDraw library")
            return 1
        target = _child(_child(Path(lib).expanduser(), "unofficial"), "parts")
    target = Path(target).expanduser()
    target.mkdir(parents=True, exist_ok=True)
    present = {p.name.casefold() for p in target.iterdir()}
    copied = 0
    sources = sorted(PARTS_DIR.glob("*.dat"))
    for src in sources:
        if src.name.casefold() not in present:
            shutil.copy2(src, target / src.name)
            copied += 1
    print(f"{copied} LSynth parts copied to {target} ({len(sources) - copied} already present)")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(
        prog="lsynth",
        description="Synthesize flexible LEGO parts (hoses, cables, bands, chains, treads) in an LDraw file. "
                    "Put the constraints between `0 SYNTH BEGIN <TYPE> <COLOUR>` and `0 SYNTH END`.",
        epilog="Environment: LSYNTH_CONFIG (definitions file, like -c), LDRAW_DIR (library for --install-parts).")
    p.add_argument("src", nargs="?", metavar="SRC", help='input LDraw file (.ldr/.mpd/.dat), or "-" for stdin')
    p.add_argument("dst", nargs="?", metavar="DST", help='output file (default: stdout); "-" for stdout')
    p.add_argument("-c", "--config", metavar="FILE", default=os.environ.get("LSYNTH_CONFIG") or None,
                   help="synthesis definitions (default: the bundled lsynth.mpd)")
    p.add_argument("-l", "--ldraw-part", action="store_true",
                   help="format the output as an official LDraw part (drop SYNTH BEGIN/END and the hose constraints)")
    p.add_argument("--list", action="store_true", help="list synthesis types, products and constraints")
    p.add_argument("--install-parts", nargs="?", const="", metavar="DIR",
                   help="copy the LSynth parts to DIR (default: $LDRAW_DIR/unofficial/parts), skipping existing files")
    p.add_argument("-v", "--version", action="version",
                   version=f"lsynth {VERSION} (Python port of LSynth by Kevin Clague and Don Heyse)")
    args = p.parse_args(argv)

    if args.install_parts is not None:
        return install_parts(args.install_parts)
    try:
        cfg = load_config(args.config, warn=lambda m: _say(f"warning: {m}"))
    except ConfigError as e:
        _say(f"error: {e}")
        return 1
    if args.list:
        list_definitions(cfg)
        return 0
    if not args.src:
        p.error('missing SRC (use "-" to read stdin)')

    to_stdout = args.dst in (None, "-")
    if args.src != "-" and not to_stdout and Path(args.dst).exists() and Path(args.dst).samefile(args.src):
        # Re-synthesis must start from the unsynthesized source: hidden constraints are ignored on input.
        _say("error: DST is the input file; write the result to a different file")
        return 1
    try:
        text = sys.stdin.buffer.read().decode(**ENCODING) if args.src == "-" else Path(args.src).read_text(**ENCODING)
    except OSError as e:
        _say(f"error: cannot read {args.src}: {e.strerror}")
        return 1

    synth = Synthesizer(cfg, ldraw_part=args.ldraw_part, info=_say, warn=lambda m: _say(f"warning: {m}"))
    try:
        result = synth.run(text).encode(**ENCODING)
    except SynthError as e:
        _say(f"error: {e}")
        return 1
    if to_stdout:
        sys.stdout.buffer.write(result)
        sys.stdout.flush()
    else:
        try:
            Path(args.dst).write_bytes(result)
        except OSError as e:
            _say(f"error: cannot write {args.dst}: {e.strerror}")
            return 1
    return 0
