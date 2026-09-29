# Patria TRACKX, exterior 3D model

![Patria TRACKX](preview/hero.png)

An outside-only, full-scale model of the Patria TRACKX, the light tracked
armoured carrier shown at DSEI in September 2025. It is generated from
`build.py` (CadQuery), so every dimension is a number at the top of the file.

| | published | model |
|---|---|---|
| Length | just over 7 m | 7.06 m |
| Width | under 3 m | 2.99 m over the guards (mirrors 3.3 m) |
| Height | 2 m to the roof plate | 2.00 m (hatches 2.06 m) |
| Tracks | 56 cm rubber | 56 cm |
| Belly clearance | 55 cm | 55 cm |
| Running gear | front sprocket, rear idler, 6 dual rubber-tyred road wheels and 2 return rollers a side | the same |

## Where the shape comes from

Published numbers set the size. Three reference photos set the shape: the
front on snow, the rear at DSEI, and the front in a forest. Sizes are scaled
off the pictures against the published width, so they are estimates.

**From the photos**, checked by rendering the model from each photo's camera
angle and comparing them side by side (`tools/render.mjs match-*`):

- **Plan:** the upper hull is full width over the tracks from the rear to the
  bumper. The windscreen sits in its front face between two long angled
  corners that carry the crew door windows; the tracks sit just inboard of the
  hull sides.
- **Front:** a wide, low windscreen with three wipers; a wide glacis with two
  louvre panels and rubber latches; headlights in angular housings at its
  outer corners; a black bumper whose ends drop and sweep back over the
  tracks; tow shackles and rounded corners on the lower plate; mirrors on
  outrigger frames.
- **Rear:** two corner blocks with sloped tops and light clusters (red lamp on
  the left of each), the troop door recessed between them and hinged on the
  left, curved rubber flaps hanging behind the tracks, tow shackles.
- **Sides:** grey box-section guards along the track tops, a slotted rail
  where the upper side leans in, bolt pads in pairs.
- **Running gear:** a raised front sprocket and rear idler with the level top
  run just under the guards, and the lower run climbing at each end. Six dual
  road wheels with dished rims and no bolts, and chunky cleats.

**Left off:** the roof weapon station, the pintle gun and the drone jammer,
antennas, camouflage and markings, and everything inside.

## Files

Generated into `out/`:

- `trackx.step`: assembly of 30 named, coloured bodies (hull, glass,
  fittings, bolts, bumper, guards, flaps, mirrors, lights and their housings,
  tow shackles, rear lights, and for each side the track, guide ridge, tyres,
  rims, hubs, return rollers, sprocket and idler, and suspension arms)
- `trackx.glb`: the same model for viewers (copied to `viewer/`)
- `trackx_1to1.stl`: one mesh, millimetres, z up
- `trackx_1to43.stl`: the same at 1:43, 164 × 70 × 51 mm, which fits the
  180 mm bed of a Bambu Lab A1 mini. Small parts such as cleats and bolts are
  finer than a 0.4 mm nozzle prints, so expect a soft detail level.

`preview/` holds stills rendered from the viewer.

## Regenerating

```sh
pip install cadquery trimesh
python3 trackx/build.py --scale 43   # any --scale N writes trackx_1toN.stl
npm install three@0.169.0 playwright
node trackx/tools/render.mjs --three node_modules/three   # stills
```

`viewer/index.html` is the interactive viewer (orbit, six views, hide the
tracks to see the hull, hover a part to name it). Serve the `viewer/` folder
over HTTP to open it locally.

Coordinates are millimetres: x forward, y left, z up, ground at z = 0.

## Sources

- [Patria: TRACKX product page](https://www.patriagroup.com/products-and-services/protected-mobility/patria-trackx)
- [Patria: TRACKX launched at DSEI UK](https://www.patriagroup.com/newsroom/news/2025/patria-trackx-the-ultimate-all-terrain-tracked-vehicle-launched-at-dsei-uk-in-london)
- [Janes: Patria unveils TRACKX](https://www.janes.com/osint-insights/defence-news/land/dsei-2025-patria-unveils-trackx-tracked-all-terrain-vehicle)
- [Defense News: Patria launches light tracked APC](https://www.defensenews.com/global/europe/2025/09/09/finlands-patria-launches-light-tracked-apc-as-successor-to-m113/)
- [EDR Magazine: the FAMOUS concept vehicle it grew out of](https://www.edrmagazine.eu/patria-unveils-the-famous-concept-vehicle)
- Three reference photos supplied by the requester (not stored in the repository)
