# Patria TRACKX, exterior 3D model

![Patria TRACKX](preview/hero.png)

An outside-only, full-scale model of the Patria TRACKX, the light tracked
armoured carrier shown at DSEI in September 2025. It is generated from
`build.py` (CadQuery), so every dimension is a number at the top of the file.

| | published | model |
|---|---|---|
| Length | just over 7 m | 7.20 m |
| Width | under 3 m | 2.95 m |
| Height | 2 m to the roof plate | 2.00 m (hatches 2.12 m) |
| Tracks | 56 cm rubber | 56 cm |
| Belly clearance | 55 cm | 55 cm |
| Running gear | front sprocket, rear idler, 6 dual rubber-tyred road wheels and 2 return rollers a side | the same |

## Where the shape comes from

I could read the search summaries of the makers' and press pages, but not open
the pages or their photos. So the model follows the published numbers and the
described layout, and the rest is my estimate.

**From the sources:** driver and commander at the front behind a large
armoured windscreen; two side crew doors; a rear troop door hinged on the
left; a forward-opening commander's roof hatch; transmission at the front;
front drive sprocket, rear idler; an almost flat underside.

**Estimated:** windscreen rake and size, bonnet height and front plate angle,
door, window and hatch sizes and positions, wheel, sprocket and idler
diameters, the track cleat pattern, the sloped roof edges, and the colour.

**Left off:** the roof weapon station and pintle gun, the drone jammer,
antennas, mirrors, and everything inside.

If you have photos, send them and I will match the windscreen, bonnet and
hatch layout to them.

## Files

Generated into `out/`:

- `trackx.step`: assembly of 21 named, coloured bodies (hull, glass, fittings,
  lights, tow eyes, and for each side the track, guide ridge, tyres, rims,
  hubs, return rollers, sprocket and suspension arms)
- `trackx.glb`: the same model for viewers (copied to `viewer/`)
- `trackx_1to1.stl`: one mesh, millimetres, z up
- `trackx_1to43.stl`: the same at 1:43, 167 × 69 × 49 mm, which fits the
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
