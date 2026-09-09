#!/usr/bin/env python3
"""
Three Finnish urban morphologies as building footprints for voxel_sim.

HONEST SCOPE: these are NOT real OSM geometry. The Overpass API was unreachable
from the machine this was written on, so the layouts are hand-authored to match
the MORPHOLOGY of each place -- block pitch, street width, courtyard depth and
storey count -- because those are what actually determine whether a corridor is
flyable. Street names and exact footprints are invented.

To replace them with the real thing (5 minutes, needs network):

    # 1. Fetch. Bounding boxes below are roughly the areas modelled here.
    #    Overpass QL, paste at https://overpass-turbo.eu or curl the API:
    #      [out:json][timeout:60];
    #      way["building"](61.4470,23.8460,61.4560,23.8650);   // Hervanta
    #      out geom;
    #
    # 2. Project lat/lon to local metres and emit this loader's format:
    #      import json, math
    #      d = json.load(open('hervanta.json'))
    #      es = [e for e in d['elements'] if e.get('geometry')]
    #      lat0 = sum(p['lat'] for e in es for p in e['geometry']) / sum(len(e['geometry']) for e in es)
    #      lon0 = sum(p['lon'] for e in es for p in e['geometry']) / sum(len(e['geometry']) for e in es)
    #      mlat, mlon = 111320.0, 111320.0 * math.cos(math.radians(lat0))
    #      for e in es:
    #          t = e.get('tags', {})
    #          h = float(t.get('height', 0)) or 3.2 * float(t.get('building:levels', 4))
    #          pts = ' '.join(f"{(p['lon']-lon0)*mlon:.1f} {(p['lat']-lat0)*mlat:.1f}"
    #                         for p in e['geometry'])
    #          print(f"{h:.1f} {pts}")
    #
    # 3. ./build/voxel_sim --world osm --buildings hervanta.txt --cell 0.5

Bounding boxes for the three areas:
    Hervanta          61.4470,23.8460,61.4560,23.8650
    Tampere centre    61.4950,23.7500,61.5020,23.7750
    Helsinki centre   60.1650,24.9350,60.1720,24.9550

Format, one building per line:  height_m  x0 y0  x1 y1  x2 y2 ...   (ENU metres)
"""
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))


def rect(cx, cy, w, h, ang=0.0):
    """Rectangle centred at (cx,cy), rotated ang radians."""
    c, s = math.cos(ang), math.sin(ang)
    out = []
    for dx, dy in ((-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)):
        out += [cx + dx*c - dy*s, cy + dx*s + dy*c]
    return out


def perimeter_block(bx, by, size, depth, gap):
    """A closed perimeter block (umpikortteli): four wings round a courtyard.
    This is the defining form of Nordic city centres and it matters for flight:
    the courtyard is enclosed, so a corridor that enters one is a dead end."""
    s, d = size, depth
    return [
        rect(bx,           by - s/2 + d/2, s, d),        # south wing
        rect(bx,           by + s/2 - d/2, s, d),        # north wing
        rect(bx - s/2+d/2, by,             d, s - 2*d),  # west wing
        rect(bx + s/2-d/2, by,             d, s - 2*d),  # east wing
    ]


def write(path, rows):
    with open(path, 'w') as f:
        f.write("# height_m  x0 y0  x1 y1 ...   (ENU metres)\n")
        for h, poly in rows:
            f.write(f"{h:.1f} " + " ".join(f"{v:.1f}" for v in poly) + "\n")
    print(f"  {os.path.basename(path):<16} {len(rows):>4} buildings")


def hervanta(seed=1):
    """1970s Finnish lahio. Widely spaced concrete slab blocks, 8-10 storeys,
    large open ground between them. WIDE corridors -- the easy case, and the one
    most like flying between apartment towers rather than through a street."""
    rng = random.Random(seed)
    rows = []
    pitch = 120.0                       # block spacing, generous
    for gy in range(5):
        for gx in range(5):
            bx, by = 60 + gx*pitch, 60 + gy*pitch
            n = rng.choice((1, 2, 2, 3))
            for k in range(n):
                # slab blocks, long and thin, varied orientation
                ang = rng.choice((0.0, math.pi/2, 0.35, -0.35))
                L = rng.uniform(45, 75)
                W = rng.uniform(12, 16)
                h = rng.uniform(24, 32)          # 8-10 storeys
                ox, oy = rng.uniform(-35, 35), rng.uniform(-35, 35)
                rows.append((h, rect(bx+ox, by+oy, L, W, ang)))
    return rows


def tampere_centre(seed=2):
    """Tampere city centre. Perimeter blocks around ~80 m, streets 16-20 m,
    5-7 storeys. Tighter than Hervanta, with enclosed courtyards."""
    rng = random.Random(seed)
    rows, pitch, size, depth = [], 98.0, 80.0, 14.0
    for gy in range(6):
        for gx in range(6):
            bx, by = 70 + gx*pitch, 70 + gy*pitch
            if rng.random() < 0.10:                    # occasional open square
                continue
            for poly in perimeter_block(bx, by, size, depth, 0):
                rows.append((rng.uniform(17, 25), poly))   # 5-7 storeys
    return rows


def helsinki_centre(seed=3):
    """Helsinki centre (Kruununhaka/Kluuvi grain). Denser again: ~70 m blocks,
    12-16 m streets, 6-7 storeys. The tightest of the three, and the case where
    a 0.6 m-radius aircraft in a 14 m street has real but limited room."""
    rng = random.Random(seed)
    rows, pitch, size, depth = [], 84.0, 70.0, 15.0
    for gy in range(7):
        for gx in range(7):
            bx, by = 60 + gx*pitch, 60 + gy*pitch
            if rng.random() < 0.06:
                continue
            for poly in perimeter_block(bx, by, size, depth, 0):
                rows.append((rng.uniform(20, 27), poly))   # 6-7 storeys
    return rows


if __name__ == '__main__':
    print("writing hand-authored Finnish urban morphologies (NOT real OSM):")
    write(os.path.join(HERE, 'hervanta.txt'), hervanta())
    write(os.path.join(HERE, 'tampere_centre.txt'), tampere_centre())
    write(os.path.join(HERE, 'helsinki_centre.txt'), helsinki_centre())
    print("\nrun:  ./build/voxel_sim --world osm --buildings worlds/hervanta.txt "
          "--cell 0.5 --display")
