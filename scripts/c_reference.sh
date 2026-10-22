#!/usr/bin/env bash
# Build the original C LSynth (lsynth/lsynth) as a test oracle and regenerate tests/golden/.
#
#   scripts/c_reference.sh            # rebuild both binaries, regenerate goldens, print float-vs-double report
#
# Two binaries are built in a temp dir (the C sources in the repo are never modified):
#   lsynthcp-float   the C code as-is (PRECISION = float, i.e. float32)
#   lsynthcp-double  the same code with PRECISION = double (sed-patched copy)
# The Python port computes in float64, so its line-by-line oracle is the double build. The report at the
# end shows how far the float32 build drifts from it: little on bands and fixed parts, a lot on some long
# STRETCH hoses, where the double build stays close to the MLCad-made examples/*-Synthesis.* files.
# -ffp-contract=off keeps C from fusing multiply-adds, so plain IEEE double arithmetic in Python can match
# it bit for bit.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="$ROOT/lsynth_nova/data/lsynth.mpd"
GOLDEN="$ROOT/tests/golden"
WORK="${LSYNTH_C_WORK:-$(mktemp -d)}"
echo "work dir: $WORK" >&2

build() {  # build <name> <patch: yes|no>
  local dir="$WORK/$1"
  rm -rf "$dir"; mkdir -p "$dir"
  cp "$ROOT"/lsynth/lsynth/*.c "$ROOT"/lsynth/lsynth/*.h "$ROOT"/lsynth/lsynth/Makefile "$dir/"
  if [ "$2" = yes ]; then
    # PRECISION float -> double; float locals -> double; sscanf %f -> %lf (all %f in lsynthcp.c feed doubles now).
    perl -pi -e 's/#define PRECISION float/#define PRECISION double/' "$dir/mathlib.h"
    perl -pi -e 's/\bfloat\b/double/g; s/%f/%lf/g' "$dir/lsynthcp.c"
    perl -pi -e 's/^  float n;/  double n;/' "$dir/band.c"
  fi
  make -s -C "$dir" CFLAGS="-O0 -g -w -ffp-contract=off -Wno-error=implicit-function-declaration" lsynthcp >&2
  cp "$dir/lsynthcp" "$WORK/lsynthcp-$1"
}

build float no
build double yes

mkdir -p "$GOLDEN" "$WORK/out-float"
for src in "$ROOT"/examples/*-Constraints.*; do
  name="$(basename "$src")"; name="${name%-Constraints.*}.ldr"
  "$WORK/lsynthcp-double" -c "$CONFIG" "$src" "$GOLDEN/$name" > /dev/null
  "$WORK/lsynthcp-float"  -c "$CONFIG" "$src" "$WORK/out-float/$name" > /dev/null
done

# Float vs double report: line counts and the largest numeric difference per file.
echo "float-vs-double (lines float/double, max |delta| on equal-length files):"
for g in "$GOLDEN"/*.ldr; do
  f="$WORK/out-float/$(basename "$g")"
  python3 - "$f" "$g" <<'EOF'
import sys
a, b = (open(p).read().split("\n") for p in sys.argv[1:3])
d = 0.0
if len(a) == len(b):
    for x, y in zip(a, b):
        for s, t in zip(x.split(), y.split()):
            try: d = max(d, abs(float(s) - float(t)))
            except ValueError: pass
print(f"  {sys.argv[2].split('/')[-1]:45s} {len(a):5d}/{len(b):5d}  {'%.4f' % d if len(a) == len(b) else 'n/a'}")
EOF
done
