# Kipinä 430: twin-tube micro FPV plane

![Kipinä 430](preview/hero.png)

A 3D-printed fixed wing with a Molniya-style layout. Two plain round tubes are
the whole fuselage structure: they run from the motor to the tail and carry the
pod, the straight rectangular wing and an H-tail. It is a hobby FPV airframe
only. There is no payload bay or release mechanism, just a pod for the flight
battery and the FPV electronics.

It flies on three **SG90** servos, one per aileron and one for the elevator, as
on the original layout. The brief was "as small as possible", so the size
comes from a sweep, not a guess. See [Why 430 mm](#why-430-mm).

| | |
|---|---|
| Wingspan | 430 mm, chord 86 mm, NACA 4412 at 2° incidence |
| Length | 401 mm (prop to elevator trailing edge) |
| Main struts | 2 × carbon tube 6×5 mm, 361 mm long, 40 mm apart |
| Servos | 3 × SG90 |
| All-up weight | ~175 g (printed parts ~67 g) |
| Wing loading | 47 g/dm² |
| Stall / cruise | ~8.9 / ~13 m/s |
| Endurance | ~13 min on 2S 450 mAh (rough estimate) |
| CG | 113.8 mm behind the motor face = 24.1 mm behind the wing LE (28 % chord) |
| Static margin | ~15 % (neutral point at ~43 % chord) |

All numbers are first-order estimates from `design.py` and the CAD volumes.
Nothing here has been flight-tested yet. [REPORT.md](REPORT.md) has the full
mass breakdown, the print list and the interference check, regenerated on
every build.

| Pod, cut open | Aileron SG90 under the wing |
|---|---|
| ![Pod](preview/pod.png) | ![Aileron servo](preview/servo.png) |

## Why 430 mm

A printed plane does not scale down evenly. The electronics weigh the same at
any size, and a printed skin can't get thinner than one nozzle line. So as the
span drops, the wing loading and the stall speed climb.

`design.py` rebuilds the layout at each span, re-balances it (tail volumes, CG,
battery station) and estimates the weight. A span passes when:

- the stall speed is at most 9 m/s at CLmax 0.95, which keeps hand launches and
  FPV landings comfortable, and
- an SG90's 32.3 mm mounting tabs fit between the two wing spars. That needs a
  chord of at least about 80 mm.

| span | AUW | stall |
|---|---|---|
| 300 mm | 146 g | 11.7 m/s |
| 380 mm | 160 g | 9.7 m/s |
| 400 mm | 164 g | 9.3 m/s |
| **420 mm** | **168 g** | **9.0 m/s (first to pass)** |
| **430 mm** | | **8.9 m/s (built)** |
| 460 mm | 177 g | 8.4 m/s |

The build rounds up by 10 mm for margin. The three SG90s weigh 27 g together;
with 4 g micro servos the same sweep lands at 400 mm. Change `Kit` and `Params`
in `design.py` and rerun to try other parts.

## Parts

### Printed (`stl/`, already in print orientation)

| file | material | orientation |
|---|---|---|
| `pod.stl` | PLA/PETG | Upright, open top up. 2 walls, 15 % infill |
| `lid.stl` | PLA/PETG | Flat, lips up |
| `wing_centre.stl` | LW-PLA | On its side. 0.6 mm walls, 8 % infill |
| `wing_R.stl`, `wing_L.stl` | LW-PLA | Standing on the root rib, brim. 1 wall, 0 % infill |
| `aileron_R.stl`, `aileron_L.stl` | LW-PLA | Flat. 1 wall, 0 % infill |
| `tail_mount.stl` | PLA/PETG | Plate face down |
| `stab.stl` | LW-PLA | Flat. 2 top / 2 bottom layers, 15 % infill |
| `elevator.stl` | LW-PLA | Top face down, horn up |
| `fin_R.stl`, `fin_L.stl` | LW-PLA | Outer face down |

The tallest part is a wing panel at 187 mm, and nothing is longer than 170 mm,
so everything fits a 200 × 200 × 200 mm bed. The masses assume LW-PLA foamed to
about 0.75 g/cm³. Plain PLA works, but it adds roughly 25 g to the wing and tail.

### Bought

Everything in this table is modelled in `cad/airframe.step`, under the
`electronics` and `hardware` groups.

| item | example spec | g |
|---|---|---|
| Servos | 3 × Tower Pro SG90 with single-arm horns | 27 |
| Main struts | 2 × carbon tube 6×5 mm, cut to 361 mm (6×0.5 aluminium also fits, +6 g) | 9.6 |
| Wing spars | carbon rod 3 mm and 2 mm, 420 mm each | 6.6 |
| Motor + prop | 1404 3800 KV, 4×2.5 two-blade | 11 |
| ESC | 12 A single, BLHeli_S, about 20 × 10 × 4 mm | 3.5 |
| Flight controller | 20×20 wing FC with servo outputs (INAV) | 6 |
| Receiver | ELRS nano | 1.5 |
| FPV | nano camera (14 mm) + 25–200 mW VTX + whip antenna | 7.5 |
| Battery | 2S 450 mAh LiPo (58 × 31 × 13 mm), XT30 | 27 |
| Linkages | 1 mm carbon rod (~225 mm) with wire Z-bend ends for the elevator, 2 × 0.8 mm Z-bend wire for the ailerons, 2 micro control horns | ~1 |
| Small parts | M2 screws, hinge tape, 10 mm velcro strap | ~2 |

## Assembly

1. **Check the fits.** Tube holes are 6.2 mm and spar holes 3.2/2.2 mm for glued
   joints. Ream any tight hole with a drill bit by hand.
2. **Fit out the pod.**
   - Screw the motor to the front plate with M2 screws from inside. The slots
     take 9×9 mm, 12 mm and 16 mm patterns.
   - Glue the camera behind its window and stand the VTX board right behind it.
     The whip antenna goes up through the hole in the lid.
   - Screw the FC to the four bosses, 33 mm behind the motor face. Stick the
     ESC to the left wall and the receiver to the right wall, both about 3 mm
     above the floor so they clear the belly chamfer. The motor wires go
     through the slot beside the camera.
3. **Fit the elevator SG90.** Lay it on its side in the rear bay, between the
   two floor ribs, with its base towards the left wall and the shaft pointing
   right. The horn points straight up. Trim the horn to 13.5 mm so it clears
   the wing centre, and use the hole 11.5 mm from the shaft. Hot glue it, or
   screw through the tabs into the ribs.
4. **Join the tubes.** Thread the wing centre's saddle onto both tubes. Push the
   tubes into the pod from the rear until they bottom out in the front sleeves.
   Then slide the wing centre forward until its caps sit over the pod's rear
   sleeves, and glue everything with epoxy or thick CA.
5. **Build the wing.**
   - Slide the 3 mm and 2 mm rods through the centre section and glue both
     panels on.
   - Each aileron SG90 lies on its side in the pocket under its panel: tabs
     between the spars, shaft pointing at the tip, horn hanging down. About
     3 mm of the servo stands proud of the lower surface.
   - The servo leads run through the wire channel and down into the pod.
6. **Hinge and link the ailerons.**
   - Tape the hinge along the top surface. The bevel under the hinge line gives
     about 25° of up travel.
   - Glue a micro control horn into the slot under each aileron.
   - Link it to the servo horn (11.5 mm hole) with 0.8 mm Z-bend wire.
7. **Build the tail.** Glue the stabiliser onto the tail mount and push the fins'
   jaws onto the stabiliser tips. Hinge the elevator with tape on top. Push the
   tail mount onto the tube ends. Sight from behind to get it square to the
   wing, then glue.
8. **Fit the elevator pushrod.** It runs 13.5 mm right of the centre-line: from
   the servo horn, over the pod's rear wall and through the tail mount guide,
   to the printed elevator horn.
9. **Balance.** Strap the battery through the floor slots and slide it until the
   plane balances 24 mm behind the wing LE (113.8 mm from the motor face).
   The nominal battery centre is 85 mm from the motor face, and the travel is
   78–92 mm. The XT30 folds back over the top of the pack.

The hatch lid rests on the tubes between the sleeves. Hold it with tape or two
3 mm magnets.

## First flights

- Throws: ailerons ±12°, elevator ±15°, 30 % expo. INAV in angle mode for the
  first launch.
- Hand-launch with a firm, level throw at about 70 % throttle.
- The prop hangs below the pod. Cut the throttle before touchdown and land on
  grass, or use a prop saver.
- The fins are fixed, as on the original layout. Roll comes from the ailerons,
  and INAV coordinates the turns.

## Regenerating

```sh
pip install cadquery trimesh
python3 airframe/design.py      # sizing sweep and layout, no CAD needed
python3 airframe/build.py       # STLs, STEP, GLB, REPORT.md, interference check

# optional: PNG previews from the three.js viewer (headless Chromium)
npm install three@0.169.0 playwright
node airframe/tools/render.mjs
```

The code is split into four files:

- `design.py` holds every dimension (`Params`), the electronics (`Kit`) and the
  SG90 datasheet dimensions (`Servo`).
- `build.py` makes the printed parts.
- `components.py` models the bought parts.
- `geom.py` holds the shared solid helpers.

The build moves the wing along the tubes until the battery balances the plane
in the middle of its travel, using the CAD masses. It then checks every bought
part against every other body for overlaps.

- `cad/airframe.step` is the full assembly in flight position, in three groups:
  `printed`, `hardware` and `electronics`. Open it in Fusion, FreeCAD or
  SolidWorks.
- `viewer/index.html` is an interactive 3D viewer. Serve the `viewer/` folder
  over HTTP, for example with `npx http-server airframe/viewer`.

The coordinates are millimetres. x runs aft from the motor mounting face, y
towards the right wing tip and z up, with z = 0 on the tube centre-line.

## Known limitations

- The weights are estimates. LW-PLA density depends heavily on print
  temperature and flow. Weigh your parts and put the real numbers in
  `build.py` (`PRINT`) before trusting the CG.
- The aerodynamics are first-order: Helmbold lift slope, a Gilruth-style pod
  term and CLmax 0.95. Expect to trim.
- The SG90 model uses datasheet dimensions. Clones vary by a few tenths of a
  millimetre, so test-fit a servo in the pocket before gluing.
- The camera, FC, ESC and receiver sizes are generic. Check them against the
  parts you buy.
