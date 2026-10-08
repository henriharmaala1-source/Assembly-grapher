# Assembly-grapher

- `docs/`: build log for the AI companion computer on an analog FPV quad (GitHub Pages).
- [`airframe/`](airframe/README.md): **Kipinä 485**, a 485 mm, 3D-printed twin-boom
  FPV pusher (the prop turns between the booms, so the nose camera has a clear
  view) with ailerons, elevator and twin rudders on four SG90 servos,
  a Matek F405-WMN flight controller and a Hobbywing XRotor Micro 30A ESC,
  generated from parametric CadQuery code (STLs, STEP assembly with detailed
  electronics, sizing report). Every part prints on a Bambu Lab A1 mini. It
  also has a bill of process with its precedence graph, an assembly animation
  and a DFMA analysis. An optimized variant (`--variant optimized`) builds in
  the efficiency study's airfoil and drag fixes. A single-boom variant
  (`--variant single-boom`) puts that plane on one carbon tube with the motor
  behind the tail. A comparison page shows any two of the three side by side,
  with what their weight differences cost in flight
  ([airframe/variants/](airframe/variants/index.html)).
- [`frames/mark4-v2-10/`](frames/mark4-v2-10/README.md): 2D drawings of the Mark4 V2 10" FPV
  frame, taken exactly from its STEP model: one DXF profile per part, dimensioned A4
  sheets with hole tables, and a check that the eight standoff holes line up through
  every plate (within 0.07 mm), with notes for printing it.
- [`aero/`](aero/README.md): drag and efficiency tools. An efficiency study
  optimizes the Kipinä wing's airfoil with NeuralFoil (an XFOIL-trained model)
  within what the printed wing needs, then compares the whole plane before and
  after, with charts. Cross-checks rerun it through XFOIL, 2D CFD (OpenFOAM) and
  AeroSandbox. A handbook optimizer ranks design changes for the
  airframe and for typical 5-inch and 7-inch FPV quads (stock propulsion,
  scored on cruise power, with no speed objective).
