"""Command line behaviour: I/O, options, error handling, re-running on output."""
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "examples" / "TECHNIC_PNEUMATIC_HOSE-Constraints.ldr"

# A small hose with visible constraints: the first leaves up (-Y), the second is turned to face +Z.
HOSE = """0 Hose test
1 7 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat
0 SYNTH BEGIN TECHNIC_PNEUMATIC_HOSE 4
0 SYNTH SHOW
1 1 0 -24 0 1 0 0 0 1 0 0 0 1 LS01.dat
1 1 0 -100 80 1 0 0 0 0 -1 0 1 0 LS01.dat
0 SYNTH END
"""


def lsynth(*args, stdin=None, env=None):
    return subprocess.run([sys.executable, "-m", "lsynth_nova", *map(str, args)], input=stdin,
                          capture_output=True, cwd=ROOT, env={**os.environ, **(env or {})})


def synthesized(text):
    return [ln for ln in text.splitlines() if ln.startswith("1 ") and "LS2" in ln]


def test_stdin_stdout_equals_files(tmp_path):
    out = tmp_path / "out.ldr"
    assert lsynth(EXAMPLE, out).returncode == 0
    piped = lsynth("-", stdin=EXAMPLE.read_bytes())
    assert piped.returncode == 0
    assert piped.stdout == out.read_bytes()
    assert lsynth(EXAMPLE).stdout == piped.stdout  # DST defaults to stdout
    assert b"Synthesized TECHNIC_PNEUMATIC_HOSE_BEVELED" in piped.stderr


def test_ldraw_part_drops_meta_and_hose_constraints():
    out = lsynth("-l", "-", stdin=HOSE.encode()).stdout.decode()
    assert not [ln for ln in out.splitlines() if "SYNTH BEGIN" in ln or "SYNTH END" in ln or "SYNTHESIZED" in ln]
    assert "LS01.dat" not in out
    assert synthesized(out)


def test_list():
    out = lsynth("--list").stdout.decode()
    for name in ("TECHNIC_PNEUMATIC_HOSE", "RUBBER_BAND", "PLI_MINIFIG_CHAIN_16L", "LS01.dat", "4185.DAT"):
        assert name in out


def test_install_parts(tmp_path):
    target = tmp_path / "parts"
    first = lsynth("--install-parts", target)
    assert first.returncode == 0 and b"32 LSynth parts copied" in first.stdout
    assert len(list(target.glob("LS*.dat"))) == 32
    assert b"0 LSynth parts copied" in lsynth("--install-parts", target).stdout


def test_install_parts_default_uses_ldraw_dir(tmp_path):
    (tmp_path / "Unofficial").mkdir()  # an existing folder is reused whatever its letter case
    assert lsynth("--install-parts", env={"LDRAW_DIR": str(tmp_path)}).returncode == 0
    assert (tmp_path / "Unofficial" / "parts" / "LS01.dat").is_file()
    assert lsynth("--install-parts", env={"LDRAW_DIR": "", "LDRAWDIR": ""}).returncode == 1


def test_refuses_to_overwrite_input(tmp_path):
    model = tmp_path / "model.ldr"
    model.write_text(HOSE)
    result = lsynth(model, model)
    assert result.returncode == 1 and b"DST is the input file" in result.stderr
    assert model.read_text() == HOSE


def test_unknown_type_passes_through():
    text = "0 SYNTH BEGIN NO_SUCH_HOSE 4\n1 1 0 0 0 1 0 0 0 1 0 0 0 1 LS01.dat\n0 SYNTH END\n"
    result = lsynth("-", stdin=text.encode())
    assert result.returncode == 0
    assert result.stdout.decode() == text
    assert b"unknown synthesis type NO_SUCH_HOSE" in result.stderr


def test_malformed_begin_fails_without_output(tmp_path):
    out = tmp_path / "out.ldr"
    result = lsynth("-", out, stdin=b"0 SYNTH BEGIN TECHNIC_PNEUMATIC_HOSE\n0 SYNTH END\n")
    assert result.returncode == 1 and b"needs a type and a colour" in result.stderr
    assert not out.exists()


def test_bad_config(tmp_path):
    result = lsynth("--list", env={"LSYNTH_CONFIG": str(tmp_path / "missing.mpd")})
    assert result.returncode == 1 and b"cannot read synthesis definitions" in result.stderr


def test_rerun_on_output_is_stable():
    once = lsynth("-", stdin=HOSE.encode()).stdout
    twice = lsynth("-", stdin=once).stdout
    assert synthesized(once.decode())
    assert twice == once


def test_hidden_constraints_are_ignored():
    hidden = HOSE.replace("0 SYNTH SHOW\n", "")  # hoses hide their constraints by default
    once = lsynth("-", stdin=hidden.encode()).stdout
    assert "0 1 1 0 -24 0" in once.decode()  # written as a comment
    again = lsynth("-", stdin=once)
    assert b"hidden constraints are ignored" in again.stderr
    assert not synthesized(again.stdout.decode())


def test_untouched_lines_are_byte_identical():
    raw = b"0 caf\xe9 \t trailing  \n1 16 0 0 0 1 0 0 0 1 0 0 0 1 3001.dat\n0 STEP\n"
    assert lsynth("-", stdin=raw).stdout == raw


def test_hose_res_applies():
    coarse = HOSE.replace("0 Hose test\n", "0 Hose test\n0 SYNTH HOSE_RES 0.5 0.5\n")
    fine = synthesized(lsynth("-", stdin=HOSE.encode()).stdout.decode())
    assert 0 < len(synthesized(lsynth("-", stdin=coarse.encode()).stdout.decode())) < len(fine)
