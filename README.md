# LSynth
[LSynth](https://deeice.github.io/lsynth) is a program that synthesizes bendable parts for LDraw files.  
For information on LDraw, please visit www.ldraw.org, the centralized LDraw information site.

![lsynth example](doc/LSynthExample.png?raw=true)

## Command line: `lsynth` (Python port)

`lsynth_nova/` is a Python port of LSynth 3.1 that runs from the command line, without MLCad. Its output
matches the original C program (see [Verification](#verification)).

### Install

```sh
uv sync                    # or: pip install .   (Python 3.12+, numpy)
lsynth --install-parts     # once: copy the LSynth parts (LS*.dat) to $LDRAW_DIR/unofficial/parts
```

Synthesized models reference LSynth's own parts (`LS01.dat`, `LS23.dat`, …), which are not in the official
library. LeoCAD reads `<library>/unofficial/parts`. Use `--install-parts DIR` to put them somewhere else.
Existing files are never overwritten.

### Use

```sh
lsynth model.ldr out.ldr          # also: lsynth model.ldr > out.ldr   or   lsynth - < model.ldr > out.ldr
lsynth --list                     # synthesis types, products and constraints (band constraints with radius)
lsynth -l part.ldr part.dat       # official-part format: no SYNTH BEGIN/END lines, no hose constraints
```

- `-c FILE` or `LSYNTH_CONFIG` replaces the bundled definitions, `lsynth_nova/data/lsynth.mpd` (2018 edition,
  which you can extend with your own types).
- Messages go to stderr. Exit status: 0 OK (warnings possible), 1 error (nothing written), 2 bad usage.
- Keep the unsynthesized model as your source and write to another file. As in LSynth, hidden constraints
  (`0 1 …`, `0 MLCAD HIDE 1 …`) are ignored on input, so `lsynth` refuses to overwrite its input. Re-running
  on output whose constraints are visible replaces the earlier synthesized parts.

### Describing a flexible part

```
0 SYNTH BEGIN TECHNIC_PNEUMATIC_HOSE 1        type (or product, e.g. PLI_TECHNIC_AXLE_FLEXIBLE_11) and colour
0 SYNTH SHOW                                  optional: keep the constraints visible (hoses hide them)
1 1 0 -24 0 1 0 0 0 1 0 0 0 1 LS01.dat        start: the hose leaves upwards (-Y)
1 1 120 -24 0 1 0 0 0 -1 0 0 0 -1 LS01.dat    end: the hose arrives downwards
0 SYNTH END
```

- **Hoses, cables, flexible axles, string, minifig chain.** List hose constraints (`LS01.dat` … see
  `--list`) from start to end; any in between are pass-through points. The hose runs through each
  constraint's origin along that constraint's **−Y axis**: at the start it points where the hose leaves,
  at the end where it arrives. The example above makes an arch. Fewer constraints give smoother curves.
  `0 SYNTH HOSE_RES <bend> <twist>` (radians; default 0.05 0.0174) sets the resolution of STRETCH hoses.
- **Bands, belts, chains, treads.** List the pulleys, gears or wheels themselves (band constraints in
  `--list`) in order around the loop. The band closes back to the first one and lies in its gear plane.
  `0 SYNTH OUTSIDE` before a wheel puts that wheel outside the loop (the band bends in around it, like an
  idler), `0 SYNTH INSIDE` (the default) switches back, and `0 SYNTH CROSS` toggles, making the band cross
  itself. `0 SYNTH BAND_RES <ldu>` sets the
  resolution of STRETCH bands.
- The parts are inserted before `0 SYNTH END`, between `0 SYNTH SYNTHESIZED BEGIN` and
  `0 SYNTH SYNTHESIZED END`. `examples/*-Constraints.*` has working models of most types.

### Docker

The tool is pure Python plus numpy, with its data inside the package and all I/O through arguments,
environment variables or stdin/stdout:

```dockerfile
COPY . /opt/lsynth
RUN pip install /opt/lsynth && LDRAW_DIR=/path/to/ldraw lsynth --install-parts
```

### Verification

- `lsynth_nova/` mirrors the files of the [original C code](https://github.com/deeice/lsynth/tree/master/lsynth/lsynth)
  (no longer in this repository): `synth.py` ← lsynthcp.c, `hose.py` ← hose.c + curve.c,
  `band.py` ← band.c, `config.py` ← `parse_descr()`, `mathlib.py` ← mathlib.c. Every function names its C
  original. `C quirk` comments mark C behaviour that was kept on purpose, or fixed where C corrupted output.
- `tests/golden/` holds the C code's output for every `examples/*-Constraints.*`, from a double-precision
  build of the original C code (`PRECISION double`, `-ffp-contract=off`). The port reproduces all 26
  examples number for number: `uv run pytest`.
- Why double: in float32, the C program drifts on long hoses (one fiber-optic cable grows from 3 to 3500
  segments). The double build, and so the port, stays within a few segments of the MLCad-made
  `examples/*-Synthesis.*` files.
- Not covered: ldraw-nova's `review` rejects the stretched (non-rigid) segment parts that LSynth emits.

## Features

Synthesizes  
  * flexible and rigid hoses, pneumatic hoses, ribbed tubes, electrical cables, fiber optic cables, and flexible axles.  
  * rubber bands, rubber belts, rubber tread, plastic tread, and chains.  
  * string, and string with plastic grips.  
  
Can be expanded with user generated synthesis types. Examine the lsynth.mpd file for instructions.  

## Instructions

The original C LSynth (for MLCad and other editors) is no longer in this repository. Download a package for your OS:  
  * [The LSynth 3.1.1 Windows installer](https://deeice.github.io/lsynth/LSynth_3_1_1_setup.exe)  
  * [Some old Mac OS X packages](http://anoved.net/2009/11/lsynth-3-1-for-mac-os-x/)  
      It's built into Bricksmith  
  * [The LSynth 3.1.1 source code](https://github.com/deeice/lsynth/tree/master/lsynth)

Follow the [setup instructions and tutorials.](http://www.holly-wood.it/lsynth/tutorial-en.html)  
Examine these [detailed examples.](http://www.holly-wood.it/lsynth/examples-en.html)  
Then make your own. Have Fun!  
