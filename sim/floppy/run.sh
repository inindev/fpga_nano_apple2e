#!/bin/bash
# floppy_track tests: .dsk/.po load and save against dsk2nib, and .nib
# behaviour compared with the floppy_track in git HEAD.
# Needs Icarus Verilog, a C compiler and Gowin's GW2A primitive models:
#   ./run.sh <gowin>/IDE/simlib/gw2a/prim_sim.v
set -e
[ -f "$1" ] || { echo "usage: $0 <path to gw2a prim_sim.v>"; exit 1; }
prim=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
here=$(cd "$(dirname "$0")" && pwd)
repo=$here/../..
work=$(mktemp -d "${TMPDIR:-/tmp}/floppy_tb.XXXXXX")

cc -O2 -o "$work/dsk2nib" "$repo/disk2nib/dsk2nib.c"
python3 -I "$here/make_refs.py" "$work/dsk2nib" "$work"

common="$here/floppy_tb.v $repo/src/gowin_dpb/gowin_dpb_trkbuf.v $prim"
new="$repo/src/floppy_track.sv $repo/src/nib_codec.sv"
# the old module uses some signals before declaring them, which Icarus rejects:
# move those declarations to the top of the module
git -C "$repo" show HEAD:src/floppy_track.sv | python3 -I -c '
import re, sys
s = sys.stdin.read()
decls = re.findall(r"^reg (?:\[5:0\] )?(?:ram_weD|trackD|activeD|readyi|busyi)\b.*;\n", s, re.M)
decls.append("reg  [31:0] lba;\n")
for d in decls:
    s = s.replace(d, "", 1)
print(s.replace("assign ready = readyD2;", "".join(decls) + "assign ready = readyD2;", 1), end="")
' > "$work/floppy_track_old.sv"

iverilog -g2012 -o "$work/dsk.vvp" -s floppy_tb $common $new
iverilog -g2012 -DNIBTEST -o "$work/nib_new.vvp" -s floppy_tb $common $new
iverilog -g2012 -DNIBTEST -DOLD -o "$work/nib_old.vvp" -s floppy_tb $common "$work/floppy_track_old.sv"

cd "$work"
echo "== .dsk/.po"
vvp -n dsk.vvp | grep -v '^VCD\|\$finish'
echo "== .nib, new"
vvp -n nib_new.vvp > nib_new.log
grep -v '^SD ' nib_new.log | grep -v '\$finish'
echo "== .nib, old"
vvp -n nib_old.vvp > nib_old.log
grep -v '^SD ' nib_old.log | grep -v '\$finish'
echo "== .nib, SD transactions old vs new ($(grep -c '^SD ' nib_new.log) each)"
if diff <(grep '^SD \|^IMG' nib_old.log) <(grep '^SD \|^IMG' nib_new.log); then
    echo "identical"
else
    echo "DIFFERENT"; exit 1
fi
rm -rf "$work"
