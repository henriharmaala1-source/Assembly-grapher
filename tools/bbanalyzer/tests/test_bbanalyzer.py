"""End-to-end and unit tests against a synthetic flight with known ground truth."""

import json
import math

import numpy as np
import pandas as pd
import pytest

from bbanalyzer import Aircraft, analysis
from bbanalyzer.aero import induced_velocity
from bbanalyzer.loader import _convert, from_dataframe
from bbanalyzer.pid import step_metrics
from bbanalyzer.report import build
from synth import TRUTH, make_flight, power_model


@pytest.fixture(scope="module")
def result():
    raw, hdr = make_flight(seed=1)
    log = from_dataframe(raw, hdr, "synthetic")
    ac = Aircraft(mass_kg=TRUTH["mass_kg"], capacity_mah=1600, prop_diameter_in=TRUTH["prop_in"])
    log, res, phases = analysis.run(log, ac)
    return log, res, phases, ac


def theory_overshoot(zeta):
    return 100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2))


# ---------------------------------------------------------------- integrity

def test_artifact_trimmed(result):
    log, *_ = result
    assert log.trimmed_s == pytest.approx(2.0, abs=0.05)


def test_logging_gap_found(result):
    log, *_ = result
    g = log.gaps
    assert len(g) == 1
    assert g["gap_ms"].iloc[0] == pytest.approx(TRUTH["gap_ms"], abs=5)


# ---------------------------------------------------------------- power / battery

def test_current_clipping_detected(result):
    _, res, *_ = result
    assert res["power"]["clipping"]["suspect"]
    assert res["power"]["clipping"]["max_a"] == pytest.approx(TRUTH["clip_a"], abs=0.5)


def test_internal_resistance(result):
    _, res, *_ = result
    r = res["power"]["internal_resistance"]["r_mohm"]
    assert r == pytest.approx(TRUTH["r_ohm"] * 1000, rel=0.2)


def test_hover_power(result):
    _, res, *_ = result
    assert res["power"]["hover"]["power_w"] == pytest.approx(power_model(np.array([0.0]))[0], rel=0.08)


# ---------------------------------------------------------------- aero

def test_coast_down_cda(result):
    _, res, *_ = result
    cd = res["aero"]["coast"]
    assert cd["available"] and not cd["lower_bound"]
    assert cd["cda_m2"] == pytest.approx(TRUTH["cda_m2"], rel=0.1)


def test_power_curve_cda_and_eta(result):
    _, res, *_ = result
    m = res["aero"]["curve"]["model"]
    assert m["cda_m2_assumed_eta"] == pytest.approx(TRUTH["cda_m2"], rel=0.1)
    assert res["aero"]["eta_measured"] == pytest.approx(TRUTH["eta"], abs=0.05)


def test_best_range_speed(result):
    _, res, *_ = result
    v = np.linspace(2, 25, 2000)
    true_best = v[np.argmin(power_model(v) / v)]
    assert res["aero"]["curve"]["best_range_speed"] == pytest.approx(true_best, abs=1.5)


# ---------------------------------------------------------------- motors

def test_front_rear_split(result):
    _, res, *_ = result
    s = res["motors"]["splits_centred"]
    assert s["front_minus_rear"]["mean_pct"] == pytest.approx(TRUTH["front_minus_rear_pct"], abs=0.3)
    assert abs(s["left_minus_right"]["mean_pct"]) < 0.5
    assert abs(s["diagM1M4_minus_diagM2M3"]["mean_pct"]) < 0.5


# ---------------------------------------------------------------- PID

def test_step_response_roll(result):
    _, res, *_ = result
    st = res["pid"]["axes"]["roll"]["step"]
    fn, zeta = TRUTH["axes"]["roll"]
    assert st["overshoot_pct"] == pytest.approx(theory_overshoot(zeta), abs=4)
    rise = (1 - 0.4167 * zeta + 2.917 * zeta ** 2) / (2 * math.pi * fn) * 1000
    assert st["rise_ms"] == pytest.approx(rise, rel=0.2)
    assert 0.95 < st["steady_state"] < 1.06


def test_step_response_damped_axes(result):
    _, res, *_ = result
    for ax in ("pitch", "yaw"):
        st = res["pid"]["axes"][ax]["step"]
        assert st["overshoot_pct"] < 8
        assert 0.95 < st["steady_state"] < 1.07


def test_i_term_offset(result):
    _, res, *_ = result
    axes = res["pid"]["axes"]
    assert axes["pitch"]["i_offset_ratio"] > 0.5
    assert axes["roll"]["i_offset_ratio"] < 0.3


# ---------------------------------------------------------------- vibration

def test_resonance_classified_structural(result):
    _, res, *_ = result
    classes = res["vibration"]["peak_classes"]
    structural = [p for p in classes if p["class"].startswith("structural")]
    assert any(abs(p["freq_hz"] - TRUTH["resonance_hz"]) < 5 for p in structural)
    motor = [p for p in classes if p["class"].startswith("motor")]
    assert motor and max(p["corr_with_x"] for p in motor) > 0.9


# ---------------------------------------------------------------- scorecard / output

def test_scorecard_flags(result):
    _, res, *_ = result
    sc = {(r["area"], r["axis"]): r["status"] for r in res["scorecard"]}
    assert sc[("I-term offset", "pitch")] == "bad"
    assert sc[("Front/rear split", "motors")] in ("check", "bad")
    assert any(a == "Frame resonance" for a, _ in sc)


def test_report_and_json(result, tmp_path):
    log, res, phases, ac = result
    html = build(log, res, phases, ac)
    for section in ("Tune &amp; airframe scorecard", "PID response", "Vibration", "Power &amp; battery",
                    "Efficiency &amp; air resistance", "data:image/png;base64"):
        assert section in html
    data = json.loads(analysis.to_json(res))
    assert data["aero"]["coast"]["cda_m2"] > 0


# ---------------------------------------------------------------- units

def test_step_metrics_on_ideal_second_order():
    fs, fn, z = 2000.0, 10.0, 0.4
    wn = 2 * math.pi * fn
    t = np.arange(int(0.5 * fs)) / fs
    wd = wn * math.sqrt(1 - z * z)
    y = 1 - np.exp(-z * wn * t) * (np.cos(wd * t) + z / math.sqrt(1 - z * z) * np.sin(wd * t))
    m = step_metrics(t, y)
    assert m["overshoot_pct"] == pytest.approx(theory_overshoot(z), abs=0.5)
    assert m["steady_state"] == pytest.approx(1.0, abs=0.01)


def test_induced_velocity_limits():
    vh = 7.0
    assert induced_velocity(np.array([0.0]), vh)[0] == pytest.approx(vh)
    assert induced_velocity(np.array([60.0]), vh)[0] == pytest.approx(vh * vh / 60.0, rel=0.02)


def test_csv_unit_suffix_is_respected():
    raw = pd.DataFrame({"time (us)": [0, 1000, 2000], "vbat (V)": [24.0, 23.9, 23.8],
                        "amperage (A)": [10.0, 11.0, 12.0], "vbat_extra": [1, 2, 3]})
    df, _ = _convert(raw, {})
    assert df["V"].iloc[0] == pytest.approx(24.0)
    assert df["I"].iloc[2] == pytest.approx(12.0)


def test_raw_units_are_scaled():
    raw = pd.DataFrame({"time": [0, 1000], "vbat": [2129, 2128], "amperage": [11462, 11000],
                        "navVel[0]": [3214, 0], "navVel[1]": [0, 0], "BaroAlt": [1532, 1532]})
    df, _ = _convert(raw, {})
    assert df["V"].iloc[0] == pytest.approx(21.29)       # values from a real Blackbox Explorer readout
    assert df["I"].iloc[0] == pytest.approx(114.62)
    assert df["speed"].iloc[0] == pytest.approx(32.14)
    assert df["alt"].iloc[0] == pytest.approx(15.32)


def test_no_trim_on_real_looking_data():
    raw, hdr = make_flight(seed=2)
    raw = raw[raw["time"] >= 0].reset_index(drop=True)           # drop the planted artifact
    log = from_dataframe(raw, hdr)
    assert log.trimmed_s == 0.0
