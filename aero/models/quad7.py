"""A typical 7-inch long-range FPV quad: aerodynamic model for aero/optimize.py.

Same model as quad.py (the airframe's aerodynamics only; the propulsion stays
stock), on an ordinary 7-inch long-range build: a 7-inch frame with 6 mm arms,
2806.5 1300 KV motors, 7 x 4 x 3 props, a 6S 3000 mAh LiPo and a GPS on a
short mast. It is scored at 100 km/h, a fast cruise for this class.
"""
from __future__ import annotations

from models.quad import Build, QuadModel

SEVEN_INCH = Build(
    name="quad7", title="Typical 7-inch long-range FPV quad, aerodynamics", speed_kmh=100,
    parts="7-inch frame with 6 mm arms, 2806.5 1300 KV, 7 x 4 x 3 props, 6S 3000 mAh LiPo, GPS, analog video",
    mass=980.0, tw=(3.0, 8.0), auw_max=1400.0,
    prop=(7.0, 4.0, 3), kv=1300.0, km=0.0278, iron_10k=5.9, bell=(33.0, 20.0),
    cells=6, pack_ah=3.0, pack_r=0.019, avionics_w=8.0,
    battery=(140.0, 44.0, 42.0), stack=(35.0, 35.0), plate=50.0 * 120.0, canopy=50.0 * 50.0,
    arm=(16.0, 6.0), arm_len=150.0, arm_exposed=120.0, arm_widths=(12, 22),
    extras_g={"canopy": 15.0, "fairing": 60.0, "sleeves": 20.0, "wedges": 6.0},
    other_cda=1.0 * 25 * 8 + 1.1 * 3 * 30, other_note="GPS on a mast",
)

MODEL = QuadModel(SEVEN_INCH)
