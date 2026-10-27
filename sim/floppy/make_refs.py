#!/usr/bin/env python3
# Reference data for floppy_tb.v, written to the directory given:
#   a_dsk.hex, b_dsk.hex   two random 140K images
#   a_nib.hex, b_nib.hex   the same as .dsk, nibblized by disk2nib/dsk2nib.c
#   a_po_nib.hex           image a read as a .po, nibblized
# Usage: make_refs.py <dsk2nib binary> <out dir>

import os, random, subprocess, sys

TRACKS, SECTOR = 35, 256

# physical sector -> 256 byte block within the track's 4K of the image
DOS_ORDER = [0x0, 0x7, 0xE, 0x6, 0xD, 0x5, 0xC, 0x4, 0xB, 0x3, 0xA, 0x2, 0x9, 0x1, 0x8, 0xF]
PO_ORDER  = [0x0, 0x8, 0x1, 0x9, 0x2, 0xA, 0x3, 0xB, 0x4, 0xC, 0x5, 0xD, 0x6, 0xE, 0x7, 0xF]

dsk2nib, out = sys.argv[1], sys.argv[2]

def write_hex(name, data):
    with open(os.path.join(out, name), 'w') as f:
        f.write('\n'.join(f'{b:02x}' for b in data) + '\n')

def nibblize(name, dsk):
    d, n = os.path.join(out, name + '.dsk'), os.path.join(out, name + '.nib')
    with open(d, 'wb') as f:
        f.write(dsk)
    subprocess.run([dsk2nib, d, n], check=True, stdout=subprocess.DEVNULL)
    with open(n, 'rb') as f:
        return f.read()

def po_to_dsk(po):
    dsk = bytearray(len(po))
    for t in range(TRACKS):
        base = t * 16 * SECTOR
        for p in range(16):
            src = base + PO_ORDER[p] * SECTOR
            dst = base + DOS_ORDER[p] * SECTOR
            dsk[dst:dst + SECTOR] = po[src:src + SECTOR]
    return bytes(dsk)

rnd = random.Random(2026)
a = bytes(rnd.randrange(256) for _ in range(TRACKS * 16 * SECTOR))
b = bytes(rnd.randrange(256) for _ in range(TRACKS * 16 * SECTOR))

write_hex('a_dsk.hex', a)
write_hex('b_dsk.hex', b)
write_hex('a_nib.hex', nibblize('a', a))
write_hex('b_nib.hex', nibblize('b', b))
write_hex('a_po_nib.hex', nibblize('a_po', po_to_dsk(a)))
