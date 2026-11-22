# bbanalyzer – blackbox log analyser

Turns an iNav or Betaflight blackbox log into a single HTML report: power and battery health,
efficiency and air resistance, motor balance, vibration (frame resonance vs motor noise) and a
PID/tune review with reconstructed step responses and a plain-language scorecard.

## Install

```bash
cd tools/bbanalyzer
python3 -m pip install -r requirements.txt
```

pip may print `Invalid script entry point ... bb2csv` for orangebox. That only concerns an
optional command-line script; the library itself installs and works.

## Use

```bash
# analyse a log (raw .TXT/.BBL from the SD card, or a CSV export)
python3 -m bbanalyzer analyze LOG00004.TXT --mass 1.1 --capacity 2200 --prop 7

# several logs in one file? list them, then pick one with --log-index
python3 -m bbanalyzer list LOG00004.TXT

# A/B test (props, frame change, tune): compare the JSON metrics of two flights
python3 -m bbanalyzer compare before.json after.json
```

`analyze` writes `<log>_report.html` (self-contained, figures embedded) and `<log>_report.json`
(metrics for comparisons) and prints the scorecard items that need attention.

| Option | Needed for |
|---|---|
| `--mass` kg (all-up) | coast-down drag, power model, hover g/W |
| `--prop` inches | induced-power model → drag area, best speeds |
| `--capacity` mAh | C-rate, % of pack used |
| `--eta` (default 0.55) | absolute CdA from the power curve (coast-down needs no η) |
| `--motor-range LO,HI` | CSV logs (no header), e.g. `1080,2000` |
| `--blades`, `--poles` | harmonic lines on the noise map when the log carries eRPM |

## What it calculates

| Area | Output | Method |
|---|---|---|
| Integrity | logging gaps, decoder artifacts trimmed, unit sanity | dt anomalies; identical linearly ramping motors |
| Power & battery | Wh, mAh, Wh/km, C-rate, sag, time above 25/50/100 A, **pack + wiring resistance**, **current-sensor clipping** | V×I integration; regression of fast voltage dips on fast current changes after removing the 2 s trend; repeated flat tops at the maximum |
| Hover | power, current, motor output, g/W, rough thrust-to-weight | hover phase detection |
| Efficiency | **power vs speed curve, best-endurance and best-range speed** | steady level flight binned by speed, fitted with P = a·W·vᵢ(v) + b + c·v³ (momentum-theory induced power + fixed + parasitic) |
| Air resistance | **CdA from coast-down** (no efficiency assumption) and **CdA/η from the power curve**; both together give a measured η | D = −(m·v·dv/dt + m·g·dh/dt)/v |
| Motors | outputs per position, **front/rear, left/right and diagonal thrust splits** (sticks centred), saturation, high-frequency motor noise | Quad X order M1 RR, M2 FR, M3 RL, M4 FL |
| Vibration | raw vs filtered gyro spectra, filter attenuation, D-term noise, **noise vs throttle map**, peaks classified **structural (frame/arm resonance) vs motor/prop** | Welch spectra; peaks tracked across load bins; eRPM-based Campbell lines when available |
| PID / tune | **step response per axis** (rise, overshoot, settling, steady state, 50 % response time), tracking error, P/I/D/F contributions, **I-term standing offset** (mechanical bias), worst tracking moments with the terms plotted, tune settings from the header, **scorecard** | Wiener deconvolution of setpoint → gyro over 2 s windows (PID-Analyzer approach) |

## Fly the log so it can answer the questions

The analyses need specific flight segments. A 3–4 minute "test card" gets everything:

1. **Hover 15 s** – hover power, g/W, thrust-to-weight.
2. **Level legs at 4–6 constant speeds, 8–10 s each, out and back** – power curve, best range speed. Out-and-back cancels wind (ground speed stands in for airspeed unless iNav logs a wind estimate).
3. **Coast-down**: from a fast level leg, throttle to zero for 2–3 s – drag area with no efficiency guess. If you never go below the coast threshold the drag is reported as a lower bound.
4. **Stick snaps on each axis for ~20 s** – step response. More, varied inputs = a cleaner response.
5. **A slow throttle sweep (climb)** – spreads the noise map across throttle so frame resonance and motor noise can be told apart. Cruising at one throttle all flight makes them inseparable, and the report says so.

## Validation

`tests/` builds a synthetic flight in raw iNav log units with planted ground truth and checks the
whole pipeline (`python3 -m pytest -q`). Current results against the truth:

| Planted | Recovered |
|---|---|
| CdA 0.0120 m² | coast-down 0.0121, power curve 0.0122 |
| drag-work efficiency η 0.55 | 0.54–0.55 |
| pack resistance 25 mΩ | ±20 % |
| roll step overshoot 16.3 % (ζ = 0.5) | 15–17 % |
| front − rear thrust split −3.0 % | −3.0 % |
| 92 Hz arm resonance | classified structural; motor tone tracked 125→297 Hz, classified motor/prop |
| 340 ms logging gap, decoder artifact, clipped current sensor | all detected |

Real-log validation: unit scaling matches a Blackbox Explorer readout (vbat, amperage, baro,
velocity, acceleration). The full pipeline still needs checking against real flights with a
reference (e.g. an inline watt meter for current, a tap test for frame modes).

## Limitations

- Airspeed = ground speed unless the log has a wind estimate. Fly out-and-back.
- Current-derived numbers are only as good as the FC current-sensor calibration.
- Battery resistance includes connectors, solder joints and wiring.
- Thrust-to-weight from hover output assumes thrust ∝ output²; treat it as rough.
- Scorecard thresholds are rules of thumb for multirotors, not hard limits. Compare tunes of the
  same aircraft rather than chasing absolute numbers.
- Motor positions assume Betaflight/iNav Quad X numbering. Other layouts need a mapping in `config.py`.
