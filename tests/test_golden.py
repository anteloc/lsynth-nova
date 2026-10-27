"""The port must reproduce the C LSynth for every example.

tests/golden/ holds the output of the C code built with PRECISION=double (upstream LSynth); the
Python port computes in float64 with the same operation order, so every number must agree.
"""
from pathlib import Path

import pytest

from lsynth_nova.config import load_config
from lsynth_nova.synth import Synthesizer

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = sorted(ROOT.glob("examples/*-Constraints.*"))
GOLDEN = ROOT / "tests" / "golden"


def golden_for(src):
    return GOLDEN / (src.name.split("-Constraints")[0] + ".ldr")


def read(path):
    return path.read_text(encoding="utf-8", errors="surrogateescape")


def same_line(a, b, tol=1e-3):
    """Equal tokens; numbers may differ in formatting (C prints %1.4f, the port trims zeros)."""
    ta, tb = a.split(), b.split()
    if len(ta) != len(tb):
        return False
    for x, y in zip(ta, tb):
        if x != y:
            try:
                if abs(float(x) - float(y)) > tol:
                    return False
            except ValueError:
                return False
    return True


def test_every_example_has_a_golden():
    assert EXAMPLES, "examples/*-Constraints.* missing"
    assert sorted(golden_for(s).name for s in EXAMPLES) == sorted(p.name for p in GOLDEN.glob("*.ldr"))


@pytest.mark.parametrize("src", EXAMPLES, ids=lambda p: p.name)
def test_matches_c_reference(src):
    out = Synthesizer(load_config()).run(read(src)).split("\n")
    gold = read(golden_for(src)).split("\n")
    assert len(out) == len(gold)
    bad = [(i + 1, a, b) for i, (a, b) in enumerate(zip(out, gold)) if not same_line(a, b)]
    assert not bad, f"{len(bad)} lines differ, first: line {bad[0][0]}\n  python: {bad[0][1]}\n  c:      {bad[0][2]}"
