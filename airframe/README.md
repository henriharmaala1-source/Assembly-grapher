# Kipinä 450: twin-tube micro FPV plane

![Kipinä 450](preview/hero.png)

A 3D-printed fixed wing with a Molniya-style layout. Two plain round tubes are
the whole fuselage structure: they run from the motor to the tail and carry the
pod, the straight rectangular wing and an H-tail. It is a hobby FPV airframe
only. There is no payload bay or release mechanism, just a pod for the flight
battery and the FPV electronics.

Four **SG90** servos give full three-axis control: one per aileron, one for the
elevator and one for the twin rudders. The brief was "as small as possible",
so the size comes from a sweep, not a guess. See [Why 450 mm](#why-450-mm).

| | |
|---|---|
| Wingspan | 450 mm, chord 90 mm, NACA 4412 at 2° incidence |
| Length | 430 mm (prop to elevator trailing edge) |
| Main struts | 2 × carbon tube 6×5 mm, 389 mm long, 40 mm apart |
| Controls | ailerons, elevator, twin rudders: 4 × SG90 |
| All-up weight | ~193 g (printed parts ~74 g) |
| Wing loading | 48 g/dm² |
| Stall / cruise | ~9.0 / ~13 m/s |
| Top speed | ~18 m/s (66 km/h) level, estimated; prop pitch speed 86 km/h |
| Endurance | ~12 min on 2S 450 mAh (rough estimate) |
| CG | 129.9 mm behind the motor face = 25.2 mm behind the wing LE (28 % chord) |
| Static margin | ~15 % (neutral point at ~43 % chord) |
| Printer | every part fits a Bambu Lab A1 mini (180 × 180 × 180 mm) |

All numbers are first-order estimates from `design.py` and the CAD volumes.
Nothing here has been flight-tested yet. [REPORT.md](REPORT.md) has the full
mass breakdown, the print list, the sizing check and the interference check,
regenerated on every build.

| Pod, cut open | Aileron SG90 under the wing |
|---|---|
| ![Pod](preview/pod.png) | ![Aileron servo](preview/servo.png) |

| Rudder linkage, from below |
|---|
| ![Tail](preview/tail.png) |

## Build process and DFMA

`process.py` turns the model into a **bill of process** ([BOP.md](BOP.md)) and a
**DFMA analysis** ([DFMA.md](DFMA.md)). The same data drives the viewer's
**Assemble** button and the video [preview/assembly.mp4](preview/assembly.mp4).

- **31 operations** in five groups:
  - 5 print jobs on the A1 mini;
  - 8 prep steps (cut carbon, make linkages, finish prints);
  - 16 assembly steps;
  - 2 setup steps.

  Each has its predecessors, tools, consumables and a time.
- **Print times are real slicer output.** They come from PrusaSlicer 2.7 on an
  A1-mini-like profile (`tools/a1mini.ini`): 7 h 22 min in total and
  72 g of filament. The wing panels take about 1 h 26 min
  each, because LW-PLA prints slowly.
- **Hand times are estimates.** They use Boothroyd–Dewhurst handling and
  insertion values plus shop times for gluing, soldering, taping and bending.
  Hands-on time is 1 h 38 min, including 30 min of INAV setup.
- **Schedule.** One builder works while one printer runs. That gives a lead
  time of **8 h 51 min**, and the critical path runs through the five
  prints in series.

| DFMA | |
|---|---|
| Parts and fasteners | 53 (theoretical minimum 26) |
| Manual assembly time | 33 min (7 min handling and insertion) |
| Design efficiency | 4% (19% counting handling and insertion only) |
| With the redesign suggestions | 45 parts, 23 min, 6% |
| Printability | no part needs support; the only wide overhang is a 16 mm bridge over the rudder servo pocket |
| Cost | about 147 EUR per airframe; the whole printed airframe is only 72 g of filament (about 3 EUR), but a first build also buys an LW-PLA and a PETG spool (about 65 EUR), so about 210 EUR up front |

Most of the assembly time goes into joints (solder, epoxy, CA, tape), not into
handling parts. The suggestions that save the most are:

- an FC with a built-in receiver;
- a camera with an integrated VTX;
- printed hinges;
- printing the tail as one piece.

DFMA.md lists each one with its trade-off.

## Why 450 mm

A printed plane does not scale down evenly. The electronics weigh the same at
any size, and a printed skin can't get thinner than one nozzle line. So as the
span drops, the wing loading and the stall speed climb.

`design.py` rebuilds the layout at each span, re-balances it (tail volumes, CG,
battery station) and estimates the weight. A span passes when:

- the stall speed is at most 9 m/s at CLmax 0.95, which keeps hand launches and
  FPV landings comfortable, and
- an SG90's 32.3 mm mounting tabs fit between the two wing spars. That needs a
  chord of at least about 80 mm.

| span | AUW (estimate) | stall |
|---|---|---|
| 300 mm | 156 g | 12.1 m/s |
| 380 mm | 171 g | 10.0 m/s |
| 420 mm | 179 g | 9.3 m/s |
| **440 mm** | **185 g** | **9.0 m/s (first to pass)** |
| **450 mm** | **193 g (CAD)** | **8.96 m/s (built)** |
| 460 mm | 192 g | 8.7 m/s |

The quick estimate is a few grams light, so the build rounds up by 10 mm. The
CAD weights confirm that 450 mm passes, and 440 mm fails at 9.08 m/s.

Yaw control costs about 11 g: a fourth SG90, two printed rudders and the
linkage. Without rudders (`Params.rudders = False`) the sweep lands at 420 mm.
Change `Kit` and `Params` in `design.py` and rerun to try other parts.

## Yaw control

Each fin carries a rudder aft of the elevator hinge line. The rudders are 14.6 mm
chord by 39 mm tall, about a third of the fin area, and are hinged with tape on
the outboard face.

- **Servo.** The rudder SG90 lies on its side in the wing centre section,
  between the tubes. It sits in a small printed fairing on top of the wing so
  that its horn, pointing down into the pod, clears the elevator servo and
  pushrod underneath.
- **Pushrod.** A 1 mm carbon rod, about 240 mm long, runs from the servo horn
  (9 mm hole) 13 mm left of the centre-line. It goes through a slot in the pod's
  rear wall and the wing saddle, back to the tail.
- **Bellcrank.** A printed 90° bellcrank pivots on an M2 screw under the tail
  mount. The pushrod drives its left arm, and its rear arm moves side to side.
- **Joiners.** Two 0.8 mm steel wires, about 86 mm each, run from the
  bellcrank's rear arm to a horn tab under each rudder. The wires run below
  the fins and the tail mount, so both rudders always move together.

## Parts

### Printed (`stl/`, already in print orientation)

| file | material | on the bed, mm | orientation |
|---|---|---|---|
| `pod.stl` | PLA/PETG | 168 × 49 × 40 | Upright, open top up. 2 walls, 15 % infill |
| `lid.stl` | PLA/PETG | 90 × 43 × 3 | Flat, lips up |
| `wing_centre.stl` | LW-PLA | 90 × 24 × 100 | On its side. 0.5 mm walls, 5 % infill |
| `wing_R.stl`, `wing_L.stl` | LW-PLA | 90 × 12 × 175 | Standing on the root rib, brim. 1 wall, 0 % infill |
| `aileron_R.stl`, `aileron_L.stl` | LW-PLA | 22 × 158 × 6 | Flat. 1 wall, 0 % infill |
| `tail_mount.stl` | PLA/PETG | 27 × 49 × 9 | Plate face down |
| `stab.stl` | LW-PLA | 26 × 174 × 2 | Flat. 2 top / 2 bottom layers, 15 % infill |
| `elevator.stl` | LW-PLA | 14 × 163 × 8 | Top face down, horn up |
| `fin_R.stl`, `fin_L.stl` | LW-PLA | 26 × 39 × 9 | Outer face down |
| `rudder_R.stl`, `rudder_L.stl` | PLA/PETG | 15 × 44 × 6 | Outer face down, wire flange up |
| `bellcrank.stl` | PLA/PETG | 16 × 19 × 2 | Flat, solid |

Every wing piece is a printed NACA 4412 section: a closed one-wall LW-PLA shell
with the spar holes, servo pocket and hinge gap built in. The wing centre and both
panels plus the ailerons come to 36 g of filament.

![Wing sections cut from the STLs](preview/wing_sections.png)

The masses assume LW-PLA foamed to about 0.75 g/cm³. Plain PLA works, but it
adds roughly 25 g to the wing and tail. The rudders and bellcrank are plain
PLA or PETG because their horns take the linkage loads.

### Printing on a Bambu Lab A1 mini

Everything fits the A1 mini's 180 × 180 × 180 mm volume. The tallest parts are
the wing panels at 175 mm. `Params.bed` holds the build volume, and the model
adapts to it in two ways:

- the centre section widens so the panels fit under the build height, and
- the stabiliser's span is capped at the bed width, with its chord widened to
  keep the same area.

Every build checks each part against the volume (see the print list in
REPORT.md). For a bigger printer, set `bed` to its volume and the centre
section shrinks back to 56 mm.

Five plates cover the whole airframe:

| plate | parts | material |
|---|---|---|
| 1 | pod, lid, tail mount, both rudders, bellcrank | PETG or PLA |
| 2 | stabiliser, elevator, both ailerons | LW-PLA |
| 3 | wing centre, both fins | LW-PLA |
| 4 | right wing panel | LW-PLA |
| 5 | left wing panel | LW-PLA |

Plates 1, 2 and 3 mix different wall and infill settings, so set them per
object in Bambu Studio.

- **Tall wing panels.** The A1 mini moves its bed front to back, which shakes
  tall, thin parts. Turn each panel so its 90 mm chord runs front to back, add
  an 8–10 mm brim, and print at reduced speed (Silent mode).
- **LW-PLA** is not a Bambu filament, so make a custom filament profile. A
  starting point: about 240 °C with the flow at 55–60 %, little or no
  retraction. Print a test cube, weigh it and adjust the flow until the density
  is near 0.75 g/cm³. The hotend reaches these temperatures without upgrades.
- **Plain PLA** prints with the stock profile if you want to skip LW-PLA for a
  first test airframe.

### Electronics

Everything below is modelled in `cad/airframe.step` (the `electronics` and
`hardware` groups) and in the CG calculation. The pod bays are sized for these
dimensions, so check sizes before substituting parts.

| part | spec | requirements | g |
|---|---|---|---|
| Servos | 4 × Tower Pro SG90: 4.8–6 V, ~1.8 kg·cm, 0.1 s/60° | 22.8 × 12.2 × 22.7 mm body, 32.3 mm across the tabs | 36 |
| Motor | 1404 outrunner, 3000–3800 KV | Rated for 2S; ≥ 193 g static thrust on a 4" prop (1:1). 9×9, 12 or 16 mm mounting | 9 |
| Prop | 4" two-blade, 2.4–2.5" pitch (Gemfan 4024 class) | Hub to match the motor shaft | 2 |
| ESC | 12 A single, BLHeli_S or AM32, 2S | About 20 × 10 × 4 mm; full-throttle draw is about 7.5 A | 3.5 |
| Flight controller | INAV-supported wing FC, 20×20 M2 mounting | Board ≤ 27 × 27 mm; ≥ 5 outputs (motor + 4 servos); analog OSD (AT7456E); 2S input; 5 V BEC ≥ 3 A for the servos (or add a separate UBEC); UART for the receiver | 6 |
| Receiver | ExpressLRS 2.4 GHz nano | CRSF to an FC UART | 1.5 |
| Camera | analog nano, 14 mm (Caddx Ant / RunCam Nano class) | 14 × 14 mm body, ≤ 12 mm deep | 3.3 |
| VTX | 5.8 GHz analog, 25–200 mW | ≤ 20 × 20 mm, 2S or 5 V input; whip antenna through the lid | 4.2 |
| Battery | 2S 450 mAh LiPo, ≥ 30C, XT30 | ≤ 58 × 31 × 13 mm | 27 |

Some electrical numbers from the model (`design.py`), all first-order:

- **Cruise:** about 1.9 A (14 W), which gives about 12 minutes on the 450 mAh
  pack with 20 % held in reserve.
- **Full throttle:** about 7.5 A at 1:1 thrust. That is about 17C from the pack
  and under two thirds of the ESC's rating.
- **Servo supply:** four SG90s can together draw well over an amp when they
  stall or buzz. Size the 5 V supply for at least 3 A.

GPS is not included. An M10 module adds about 5 g, which raises the stall
speed by about 0.1 m/s, just past the sizing limit. That is fine once you know
the plane; otherwise build the wing at 460 mm.

### Also needed

| item | spec | g |
|---|---|---|
| Main struts | 2 × carbon tube 6×5 mm, cut to 389 mm (6×0.5 aluminium also fits, +8 g) | 10.4 |
| Wing spars | carbon rod 3 mm and 2 mm, 440 mm each | 6.9 |
| Pushrods | 2 × 1 mm carbon rod (~245 mm) with wire Z-bend ends: elevator and rudder | 0.6 |
| Wire | 0.8 mm steel: 2 aileron links, 2 rudder joiners (~90 mm each) | ~1 |
| Small parts | 2 micro control horns, M2 screws (one for the bellcrank), hinge tape, 10 mm velcro strap | ~2 |

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
4. **Fit the rudder SG90** before the wing centre goes on.
   - Press it into the pocket under the centre section from below: shaft
     pointing left, tabs between the spars. The top of the servo sits in the
     fairing.
   - Trim its horn to 11 mm, point it straight down and hook the rudder
     pushrod into the 9 mm hole.
5. **Join the tubes.** Thread the wing centre's saddle onto both tubes, feeding
   the rudder pushrod through the slot in the saddle and the pod's rear wall.
   Push the tubes into the pod from the rear until they bottom out in the front
   sleeves. Then slide the wing centre forward until its caps sit over the
   pod's rear sleeves, and glue everything with epoxy or thick CA.
6. **Build the wing.**
   - Slide the 3 mm and 2 mm rods through the centre section and glue both
     panels on.
   - Each aileron SG90 lies on its side in the pocket under its panel: tabs
     between the spars, shaft pointing at the tip, horn hanging down. About
     3 mm of the servo stands proud of the lower surface.
   - All servo leads run through the wire channel and down into the pod.
7. **Hinge and link the ailerons.**
   - Tape the hinge along the top surface. The bevel under the hinge line gives
     about 25° of up travel.
   - Glue a micro control horn into the slot under each aileron.
   - Link it to the servo horn (11.5 mm hole) with 0.8 mm Z-bend wire.
8. **Build the tail.**
   - Glue the stabiliser onto the tail mount and push the fins' jaws onto the
     stabiliser tips.
   - Hinge the elevator with tape on top.
   - Hinge each rudder to the back of its fin with tape on the outboard face.
     The horn tab goes at the bottom.
   - Push the tail mount onto the tube ends. Sight from behind to get it square
     to the wing, then glue.
9. **Fit the elevator pushrod.** It runs 13.5 mm right of the centre-line: from
   the servo horn, over the pod's rear wall and through the tail mount guide,
   to the printed elevator horn.
10. **Fit the rudder linkage.**
    - Screw the bellcrank loosely under the tail mount with an M2 screw, so it
      turns freely.
    - Hook the rudder pushrod into its left arm.
    - Bend both joiner wires with an L at each end. Drop one end into the
      bellcrank's rear arm and the other into the flange under each rudder.
    - Adjust the wire lengths so both rudders sit straight with the servo
      centred.
11. **Balance.** Strap the battery through the floor slots and slide it until
    the plane balances 25 mm behind the wing LE (129.9 mm from the motor face).
    The nominal battery centre is 90 mm from the motor face, and the travel is
    78–101 mm. The XT30 folds back over the top of the pack.

The hatch lid rests on the tubes between the sleeves. Hold it with tape or two
3 mm magnets.

## First flights

- **Throws:** ailerons ±12°, elevator ±15°, rudders ±20° (set with the servo
  endpoints), 30 % expo.
- **INAV setup:** use the Airplane mixer with a rudder output, and fly angle
  mode for the first launch.
- **Launch:** hand-launch with a firm, level throw at about 70 % throttle.
- **Landing:** the prop hangs below the pod. Cut the throttle before touchdown
  and land on grass, or use a prop saver.
- **Rudder:** use it to coordinate turns and hold heading in crosswinds. Check
  the rudder direction on the bench, and reverse the servo in INAV if needed.

## Regenerating

```sh
pip install cadquery trimesh
python3 airframe/design.py      # sizing sweep and layout, no CAD needed
python3 airframe/build.py       # STLs, STEP, GLB, REPORT.md, sizing + interference checks

python3 airframe/process.py     # BOP.md, DFMA.md, viewer/process.json (slices with prusa-slicer if installed)

# optional: PNG previews and the assembly video from the three.js viewer (headless Chromium)
npm install three@0.169.0 playwright
node airframe/tools/render.mjs
node airframe/tools/render.mjs --video --ffmpeg /path/to/ffmpeg
```

The code is split into four files:

- `design.py` holds every dimension (`Params`), the electronics (`Kit`) and the
  SG90 datasheet dimensions (`Servo`).
- `build.py` makes the printed parts.
- `components.py` models the bought parts.
- `geom.py` holds the shared solid helpers.

Each build:

- moves the wing along the tubes until the battery balances the plane in the
  middle of its travel, using the CAD masses,
- checks the stall speed with the CAD weight,
- checks every bought part against every other body for overlaps, and
- checks every printed part against the printer's build volume.

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
