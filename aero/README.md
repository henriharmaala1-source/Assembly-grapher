# Air-resistance and efficiency optimizer

A small, dependency-free tool with two models:

- **`airframe`**: the Kipinä twin-boom pusher. It looks for the lowest drag at
  cruise.
- **`quad`**: a typical 5-inch FPV quad. It looks for the longest flight time
  at a relaxed cruise, while staying an ordinary FPV quad (see
  [What the quad model will not do](#what-the-quad-model-will-not-do)).

It uses handbook formulas, not CFD or thrust-stand data. That makes it good for
ranking design changes and seeing what matters, but not for exact numbers.

```sh
python3 aero/optimize.py airframe                 # Kipinä drag at its 13 m/s cruise
python3 aero/optimize.py airframe --free span     # let the span grow too (the brief fixes it)
python3 aero/optimize.py airframe --only fairings,boattail
python3 aero/optimize.py quad                     # quad flight time at 40 km/h
python3 aero/optimize.py quad --kmh 60            # ... at 60 km/h
python3 aero/optimize.py quad --fix motor         # keep the stock motors
```

Each run prints a summary and writes `aero/REPORT_<model>.md`, which has:

- the result before and after;
- the changes in the optimum;
- **one change at a time**: each parameter moved alone to its best value,
  ranked by how much it helps;
- the drag build-up, item by item;
- every parameter's range and what limits it.

## Results

### Typical 5-inch FPV quad, at 40 km/h ([REPORT_quad.md](REPORT_quad.md))

The starting point is an ordinary 6S freestyle build:

- 2306 1750 KV motors;
- 5.1 × 4.3 tri-blade props;
- a 6S 1100 mAh LiPo;
- analog video;
- 504 g all-up, thrust-to-weight 7.8 at mid-pack.

The model gives it 10.0 min at a steady 40 km/h and 9.6 min in a hover.

The optimum keeps the frame, the 6S battery and the analog kit. It changes
the props, motors and battery, and roughly doubles the flight time to
**19.7 min** (17.4 min in a hover):

| | typical build | optimised |
|---|---|---|
| Props | 5.1 × 4.3 × 3 | 5.1 × 3.0 × 2 |
| Motors | 2306 1750 KV | 1804 2027 KV (2004s give 19.0 min with a little more thrust) |
| Battery | 6S 1100 mAh LiPo, 186 g | 6S ~2300 mAh LiPo, 383 g |
| All-up weight | 504 g | 637 g |
| Thrust-to-weight | 7.8 | 5.0, the lower limit |

That is what a long-range 5-inch cruiser looks like. The optimizer spends
the thrust margin on flight time until it reaches the lower limit.

On their own, the changes rank like this:

| change | flight time on its own |
|---|---|
| A bigger pack (6S ~2300 mAh) | +37 % |
| Smaller, lighter motors (1804) | +14 % |
| Two-blade props | +10 % |
| Lower pitch (3.0 inch) | +9 % |

- **Keeping the stock 2306 motors** (`--fix motor`) still reaches 16.3 min.
  The props and the battery do most of the work.
- **Li-ion does not fit.** A 21700 pack can't supply the current for
  thrust-to-weight 5 within its rating on a quad under 700 g. Li-ion builds
  run lower thrust-to-weight on bigger frames.
- **Drag matters little at these speeds.** At 40 km/h the body's drag takes
  about 6 W of the 116 W. At 60 km/h it is about 26 W of 148 W, and the
  optimum is the same build (7.9 → 17.0 min). A printed canopy never earns
  its 10 g.

### What the quad model will not do

The brief was to make a typical FPV drone better without it turning into an
interceptor. So the quad model rewards flight time at a relaxed cruise and
nothing else:

- It has no speed, acceleration or payload objective, and it doesn't estimate
  top speed.
- Thrust-to-weight must stay between **5 and 12**, the band ordinary freestyle
  quads fly with. That is enough to fly and recover normally, but not a
  racer's or an interceptor's punch.
- All-up weight stays at or under 700 g, what a 5-inch frame and props are
  built for.

In practice the optimum sits on the *lower* thrust-to-weight limit.
Efficiency pulls the design away from the high-power, high-speed end, not
towards it.

### Kipinä, at its 13 m/s cruise ([REPORT_airframe.md](REPORT_airframe.md))

The span is fixed by the brief: the smallest plane that passes the stall
limit. With that fixed, the rest of the sizing is pinned too:

- a higher aspect ratio would push the stall over the limit, and leave no room
  for the SG90 tabs;
- a longer tail arm would move the wing aft and make the lid too long for the
  bed.

The drag to win is in the details. Together they cut the drag by about a
fifth (32 → 26 gf, endurance 13.6 → 16.4 min at this speed):

| change (none of these is in the CAD yet) | drag saved on its own |
|---|---|
| Round or chamfer the pod's top front edge (the front of the lid). It is the only sharp edge left on the nose. 6–8 mm does it; a 10 mm 45° chamfer does nearly as well and prints without support | 7.5 % |
| Run the rudder joiner wires in a groove under the stabiliser: 171 mm of 0.8 mm wire across the flow | 4.0 % |
| Printed fairings over the four servo horns and the aileron servo bumps | 3.4 % |
| Taper the pod's rear 30 mm at 12° towards the motor mount (the ESC moves forward) | 2.3 % |
| Lay the VTX whip back 60° | 2.2 % |

- **Letting the span grow** (`--free span`): the optimizer stretches the wing
  to 580 mm at aspect ratio 7 for the same area, and the drag falls by 24 %
  instead of 19.5 %. That is a bigger plane, against the brief.
- **Compared with `design.py`**: the build-up here counts more drag than the
  quick estimate (22.9 cm² against 18.4 cm²), mostly small parts that the
  quick estimate lumps together.

## How it works

- **`dragkit.py`** has the drag formulas:
  - skin friction on a flat plate (laminar, turbulent, or mixed) times a form
    factor for wings and bodies (Raymer);
  - pressure drag of a blunt front against how round its edges are
    (Hoerner);
  - base drag behind a square-cut rear;
  - rods, wires and strips across the flow.
- **`models/airframe.py`** builds the airframe from `airframe/design.py`.
  Changing the aspect ratio, the tail arm or the span re-sizes the tail,
  re-balances the plane and re-estimates its weight, exactly as the sizing
  loop does. The total drag includes the induced drag of carrying the weight.
  Every point must:
  - keep the stall at or under 8.95 m/s with the CAD weight;
  - leave room for the SG90 tabs in the wing;
  - fit the lid and the rear pod half on the bed.
- **`models/quad.py`** builds the quad from its parts:
  - weight from each part;
  - body drag at the cruise tilt that the drag itself sets;
  - rotor power in forward flight: momentum theory with Glauert's inflow,
    plus the props' profile power;
  - motor losses (winding loss from the motor constant, iron loss with rpm),
    ESC and battery losses;
  - full-throttle thrust and currents, from the motor model and the pack
    sagging under load.

  The prop coefficients are fitted to typical 5-inch FPV props, so flight
  times are good to perhaps ±15 %.
- **`optimize.py`** first moves each parameter alone over its whole range.
  Then it searches all free parameters together: one grid step up and down on
  each, then pairs of parameters moved at once, so it can slide along a limit
  (for example, less pitch with more KV at the same thrust-to-weight). It
  keeps any move that helps without breaking a limit, and halves the step
  when nothing does.

In the reports, the **kind** column says whether a change is a parameter of
the CAD, a what-if the CAD doesn't build yet, or a part you choose.

## Adding a model

A model is a module in `aero/models/` with:

- `NAME`, `TITLE`, `OBJECTIVE` (what is optimised), `MAXIMIZE`, `BASE_LABEL`
  (what the starting design is called), `NOTE`, `METHOD` and `SPEED` (m/s);
- if it has a drag build-up, `AREA_UNIT` (a label and a scale from m²) and
  `ITEMS_NOTE`;
- optionally `DEFAULT_FIX`: parameters held unless `--free` names them;
- `params()`, which returns a list of `model.Param`: the starting value, the
  range, the step, the unit, what limits it, and its kind;
- `evaluate(x, v)`, which returns a `model.Result`: the objective, the drag
  items, the numbers to show and any limits broken;
- `speed_text(v)` and `value_text(value)` for the report.

## Limits

- The coefficients come from handbooks and typical-prop fits, not
  measurements. Expect about ±30 % on the airframe's drag totals (its
  Reynolds numbers are low: 50 000 to 300 000) and ±15 % on the quad's flight
  times. The ranking and the differences between versions are more reliable.
  Check a real prop, motor and battery combination on a thrust stand or in
  eCalc before buying.
- The optimizer finds a local optimum on each parameter's grid. The paired
  moves, and a restart from the best single change, cover the cases here.
- It only knows the limits written into each model. A change can look good
  here and still be a bad idea for structure, cost or use, so read the
  "limit" column before acting on one.
