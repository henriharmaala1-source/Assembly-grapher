# Air-resistance optimizer

A small, dependency-free tool that estimates the air resistance (drag) of the
Kipinä airframe and searches its design parameters for the lowest drag. It
uses handbook drag formulas, not CFD: good for ranking design changes and
seeing what matters, not for exact numbers.

```sh
python3 aero/optimize.py airframe                 # Kipinä, at its cruise speed
python3 aero/optimize.py airframe --speed 18      # at 18 m/s
python3 aero/optimize.py airframe --fix span,aspect_ratio   # keep the sizing, tune the details
python3 aero/optimize.py airframe --only fairings,boattail
```

Each run prints a summary and writes `aero/REPORT_<model>.md`:

- the drag before and after;
- the changes in the optimum;
- **one change at a time**: each parameter moved alone to its best value,
  ranked by how much drag it saves;
- the full drag build-up, item by item;
- every parameter's range and what limits it.

## Results

### Kipinä, at its 13 m/s cruise ([REPORT_airframe.md](REPORT_airframe.md))

The sizing is already at its optimum. The span sits on the stall limit, and a
higher aspect ratio would leave no room for the SG90 tabs. A longer tail arm
would move the wing aft and make the lid too long for the bed. The drag to win
is in the details, and together they cut the drag by about a fifth
(32 → 26 gf, endurance 13.6 → 16.4 min at this speed):

| change (none of these is in the CAD yet) | drag saved on its own |
|---|---|
| Round or chamfer the pod's top front edge (the front of the lid). It is the only sharp edge left on the nose. 6–8 mm does it; a 10 mm 45° chamfer does nearly as well and prints without support | 7.5 % |
| Run the rudder joiner wires in a groove under the stabiliser: 171 mm of 0.8 mm wire across the flow | 4.0 % |
| Printed fairings over the four servo horns and the aileron servo bumps | 3.4 % |
| Taper the pod's rear 30 mm at 12° towards the motor mount (the ESC moves forward) | 2.3 % |
| Lay the VTX whip back 60° | 2.2 % |

The build-up here counts more drag than `design.py`'s quick estimate
(22.9 cm² against 18.4 cm²), mostly small parts that the quick estimate
lumps together. At 18 m/s the parasite drag matters more, and a slightly
longer tail arm (3.0 chords) also starts to pay.

## How it works

- **`dragkit.py`** has the drag formulas:
  - skin friction on a flat plate (laminar, turbulent, or mixed) times a form
    factor for wings and bodies (Raymer);
  - pressure drag of a blunt front against how round its edges are
    (Hoerner);
  - base drag behind a square-cut rear;
  - rods, wires and strips across the flow.
- **`models/airframe.py`** builds the airframe from `airframe/design.py`, so
  changing the span, the aspect ratio or the tail arm re-sizes the tail,
  re-balances the plane and re-estimates its weight, exactly as the sizing loop
  does. The total drag includes the induced drag of carrying the weight.
  Every point must keep the stall at or under 9 m/s with the CAD weight and
  leave room for the SG90 tabs in the wing.
- **`optimize.py`** first moves each parameter alone over its whole range,
  then runs a coordinate search over all free parameters together. The search
  tries one step up and one step down on each parameter and keeps any move that
  lowers the drag without breaking a limit, then halves the step.

Parameters marked **what-if** in the reports are not built in the CAD yet.
They show what a change would be worth before anyone models it.

## Adding a model

A model is a module in `aero/models/` with:

- `NAME`, `TITLE`, `NOTE`, `METHOD`, `SPEED` (m/s) and `AREA_UNIT`
  (a label and a scale from m²);
- `params()`, which returns a list of `model.Param`: the value as built, the
  range, the step, the unit and what limits it;
- `evaluate(x, v)`, which returns a `model.Result`: the drag items, the drag
  in newtons, any limits broken, and the numbers to show;
- `speed_text(v)` and `force_text(n)` for the report.

## Limits

- The coefficients come from wind-tunnel handbooks. At the airframe's low
  Reynolds numbers (50 000 to 300 000), expect about ±30 % on the totals and
  more on the small items. The ranking and the differences between versions are more reliable.
- The optimizer finds a local optimum on each parameter's grid. It also
  restarts from the best single change, which covers the simple cases here.
- It only knows the limits written into each model. A change can look good
  here and still be a bad idea for structure, cost or use, so read the
  "limit" column before acting on one.
