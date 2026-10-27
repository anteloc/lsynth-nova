"""LSynth for the command line: synthesize flexible LEGO parts (hoses, cables, bands, chains, treads)
in LDraw files.  A Python port of LSynth 3.1 by Kevin Clague and Don Heyse (https://github.com/deeice/lsynth).

Pipeline:  cli.py (arguments, I/O)  ->  synth.py (scan the model, collect SYNTH blocks)
           ->  hose.py / band.py (geometry)  ->  LDraw type-1 lines.   config.py reads lsynth.mpd.
"""

VERSION = "3.1"  # LSynth version this port follows; compared against lsynth.mpd's !VERSION as in C.
