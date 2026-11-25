"""Betaflight, iNav and ArduPilot inputs: format handling and the same planted truth through each."""

import numpy as np
import pytest

from bbanalyzer import Aircraft, analysis, ardupilot
from bbanalyzer.loader import firmware_family, from_dataframe, list_logs, load
from synth import TRUTH, make_flight, to_betaflight

import ap_synth


def _ac():
    return Aircraft(mass_kg=TRUTH["mass_kg"], capacity_mah=1600, prop_diameter_in=TRUTH["prop_in"])


# ---------------------------------------------------------------- detection

def test_family_from_firmware_string():
    assert firmware_family({"Firmware revision": "INAV 7.1.2 (b1e5e3a6) MATEKH743"}) == "inav"
    assert firmware_family({"Firmware revision": "Betaflight 4.5.1 (77d01ba3b) STM32H743"}) == "betaflight"
    assert firmware_family({"Firmware revision": "ArduCopter V4.5.7 (2a3dc4b7)"}) == "ardupilot"


def test_family_from_field_names():
    assert firmware_family({}, ["time", "navVel[0]"]) == "inav"
    assert firmware_family({}, ["time", "setpoint[0]", "vbatLatest"]) == "betaflight"
    assert firmware_family({}, ["time"]) == "unknown"


# ---------------------------------------------------------------- Betaflight

@pytest.fixture(scope="module")
def bf_result():
    raw, hdr = to_betaflight(*make_flight(seed=4))
    log = from_dataframe(raw, hdr, "synthetic-bf")
    return analysis.run(log, _ac())


def test_betaflight_fields(bf_result):
    log, res, _ = bf_result
    assert log.family == "betaflight"
    for col in ("V", "I", "sp0", "gy0", "gr0", "speed", "vn", "vz", "alt", "m0"):
        assert log.has(col), col
    assert log.df["V"].max() == pytest.approx(25.0, abs=0.3)          # vbatLatest in 0.01 V


def test_betaflight_gps_staircase_removed(bf_result):
    log, *_ = bf_result
    v = log.df["speed"].to_numpy()
    fs = log.fs
    held = np.mean(np.diff(v[np.isfinite(v)]) == 0)
    assert held < 0.5                                   # held GPS would repeat ~99 % of rows
    assert v.max() == pytest.approx(20.0, abs=0.3)
    assert fs == pytest.approx(1000, rel=0.01)


def test_betaflight_pipeline(bf_result):
    _, res, _ = bf_result
    assert res["power"]["internal_resistance"]["r_mohm"] == pytest.approx(TRUTH["r_ohm"] * 1000, rel=0.2)
    assert res["aero"]["coast"]["cda_m2"] == pytest.approx(TRUTH["cda_m2"], rel=0.15)
    s = res["motors"]["splits_centred"]["front_minus_rear"]["mean_pct"]
    assert s == pytest.approx(TRUTH["front_minus_rear_pct"], abs=0.3)
    assert res["pid"]["axes"]["pitch"]["i_offset_ratio"] > 0.5


def test_inav_family_on_reference_flight():
    raw, hdr = make_flight(seed=1)
    hdr = dict(hdr, **{"Firmware revision": "INAV 7.1.2 (b1e5e3a6) MATEKH743"})
    assert from_dataframe(raw, hdr).family == "inav"


# ---------------------------------------------------------------- ArduPilot

@pytest.fixture(scope="module")
def ap_files(tmp_path_factory):
    d = tmp_path_factory.mktemp("ap")
    ap_synth.write_bin(d / "00000042.BIN")
    ap_synth.write_bin(d / "clean.BIN", garbage_at=None)
    ap_synth.write_text(d / "00000042.log")
    return d


@pytest.fixture(scope="module")
def ap_result(ap_files):
    return analysis.run(ap_files / "00000042.BIN", _ac())


def test_ardupilot_sniffing(ap_files):
    assert ardupilot.sniff(ap_files / "00000042.BIN") == "bin"
    assert ardupilot.sniff(ap_files / "00000042.log") == "text"
    assert list_logs(ap_files / "00000042.BIN")[0]["firmware"].startswith("ArduCopter")


def test_ardupilot_scaled_fields(ap_files):
    msgs, _ = ardupilot.read(ap_files / "clean.BIN")
    att = msgs["ATT"].iloc[0]                         # 'c'/'C' fields are centi-units in the file
    assert att["Roll"] == pytest.approx(1.25) and att["Pitch"] == pytest.approx(-3.5)
    assert att["Yaw"] == pytest.approx(123.4)
    assert set(msgs["PARM"]["Name"]) >= {"FRAME_CLASS", "MOT_PWM_MIN", "SERVO1_FUNCTION"}


def test_ardupilot_matches_pymavlink(ap_files):
    DFReader = pytest.importorskip("pymavlink.DFReader")
    msgs, _ = ardupilot.read(ap_files / "clean.BIN")
    r = DFReader.DFReader_binary(str(ap_files / "clean.BIN"))
    got = {}
    while (m := r.recv_match(type=["ATT", "BAT", "XKF1", "RCOU"])) is not None:
        got.setdefault(m.get_type(), []).append(m.to_dict())
    for name, rows in got.items():
        df = msgs[name]
        assert len(df) == len(rows)
        for col in rows[0]:
            if col != "mavpackettype":
                assert np.allclose([r_[col] for r_ in rows], df[col].to_numpy(float)), (name, col)


def test_ardupilot_text_equals_binary(ap_files):
    txt, _ = ardupilot.read(ap_files / "00000042.log")
    binm, _ = ardupilot.read(ap_files / "clean.BIN")
    for name in ("RATE", "BAT", "XKF1", "RCOU"):
        n = len(txt[name])
        assert n > 10
        for col in txt[name].columns:
            assert np.allclose(txt[name][col].to_numpy(float), binm[name][col].to_numpy(float)[:n],
                               rtol=1e-5, atol=1e-3), (name, col)


def test_ardupilot_log_metadata(ap_result):
    log, *_ = ap_result
    assert log.family == "ardupilot"
    assert log.firmware.startswith("ArduCopter V4.5.7")
    assert log.headers["flight_modes"][0][1] == "Loiter"
    assert log.headers["ATC_RAT_RLL_P"] == pytest.approx(0.135)
    assert any("corrupt" in n for n in log.notes)            # planted garbage skipped, parsing continued
    assert len(log.gaps) == 1 and log.gaps["gap_ms"].iloc[0] == pytest.approx(TRUTH["gap_ms"], abs=5)
    # the second EKF core carries wrong velocities on purpose: only core 0 may be used
    assert np.nanmax(log.df["speed"]) == pytest.approx(20.0, abs=0.1)


def test_ardupilot_pipeline(ap_result):
    _, res, _ = ap_result
    assert res["power"]["internal_resistance"]["r_mohm"] == pytest.approx(TRUTH["r_ohm"] * 1000, rel=0.2)
    assert res["power"]["fc_impedance_mohm"] == pytest.approx(TRUTH["r_ohm"] * 1000, rel=0.01)
    assert res["power"]["clipping"]["suspect"]
    assert res["aero"]["coast"]["cda_m2"] == pytest.approx(TRUTH["cda_m2"], rel=0.1)
    assert res["aero"]["eta_measured"] == pytest.approx(TRUTH["eta"], abs=0.05)
    s = res["motors"]["splits_centred"]
    assert s["front_minus_rear"]["mean_pct"] == pytest.approx(TRUTH["front_minus_rear_pct"], abs=0.3)
    assert abs(s["left_minus_right"]["mean_pct"]) < 0.5
    st = res["pid"]["axes"]["roll"]["step"]
    assert st["overshoot_pct"] == pytest.approx(16.3, abs=4)
    assert res["pid"]["axes"]["pitch"]["i_offset_ratio"] > 0.5
    structural = [p for p in res["vibration"]["peak_classes"] if p["class"].startswith("structural")]
    assert any(abs(p["freq_hz"] - TRUTH["resonance_hz"]) < 5 for p in structural)
    assert "ATC_RAT_RLL_P" in res["pid"]["header"]


def test_ardupilot_load_via_cli_path(ap_files, tmp_path):
    from bbanalyzer.__main__ import main
    out = tmp_path / "r.html"
    rc = main(["analyze", str(ap_files / "00000042.BIN"), "--mass", "1.1", "--prop", "7", "--out", str(out)])
    assert rc in (0, None) and out.exists() and "ArduCopter" in out.read_text(encoding="utf-8")
