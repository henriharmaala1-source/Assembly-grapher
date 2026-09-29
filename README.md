# Assembly-grapher

- `docs/`: build log for the AI companion computer on an analog FPV quad (GitHub Pages).
- [`airframe/`](airframe/README.md): **Kipinä 485**, a 485 mm, 3D-printed twin-boom
  FPV pusher (the prop turns between the booms, so the nose camera has a clear
  view) with ailerons, elevator and twin rudders on four SG90 servos,
  a Matek F405-WMN flight controller and a Hobbywing XRotor Micro 30A ESC,
  generated from parametric CadQuery code (STLs, STEP assembly with detailed
  electronics, sizing report). Every part prints on a Bambu Lab A1 mini. It
  also has a bill of process with its precedence graph, an assembly animation
  and a DFMA analysis.
- [`aero/`](aero/README.md): a simple air-resistance optimizer for the airframe:
  a handbook drag build-up, a ranking of single design changes and a search for
  the lowest drag within the design's limits.
