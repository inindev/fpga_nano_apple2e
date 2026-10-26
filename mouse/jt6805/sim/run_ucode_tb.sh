#!/bin/sh
# Brute-force test of jt6805_ucode against the Gowin pROM IP it replaces.
# Needs Icarus Verilog and Gowin's GW2A primitive models:
#   ./run_ucode_tb.sh <gowin>/IDE/simlib/gw2a/prim_sim.v
set -e
[ -f "$1" ] || { echo "usage: $0 <path to gw2a prim_sim.v>"; exit 1; }
here=$(cd "$(dirname "$0")" && pwd)
jt=$here/..
out=${TMPDIR:-/tmp}/ucode_tb.vvp

iverilog -g2012 -o "$out" -s ucode_tb \
    "$here/ucode_tb.v" "$jt/jt6805_ucode.v" \
    "$jt/../../src/gowin_prom/gowin_prom_uc.v" "$1"

# $readmemh looks for the index hex in the current directory
cd "$jt" && vvp -n "$out"
