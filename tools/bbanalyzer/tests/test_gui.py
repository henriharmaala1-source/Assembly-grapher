"""GUI smoke test: build the window, analyse a log through it, check the tabs are filled."""

import pytest

tk = pytest.importorskip("tkinter")


@pytest.fixture()
def root():
    try:
        r = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available")
    r.withdraw()
    yield r
    r.destroy()


def test_gui_analyses_a_log(root, tmp_path, monkeypatch):
    from bbanalyzer import app as app_mod
    from bbanalyzer import gui
    from synth import make_flight

    monkeypatch.setattr(app_mod, "SETTINGS", tmp_path / "settings.json")
    monkeypatch.setattr(gui, "_save_settings", lambda s: None)
    log = tmp_path / "flight.csv"
    make_flight(seed=3)[0].to_csv(log, index=False)

    win = gui.App(root, [str(log)])
    for k, v in {"mass_kg": "1.1", "prop_diameter_in": "7", "capacity_mah": "1600",
                 "blades": "2", "motor_range": "1080,2000", "log_index": ""}.items():
        win.vars[k].set(v)
    win.open_browser.set(False)
    ac, idx, mr = win._aircraft()
    win._worker([str(log)], ac, idx, mr)          # run synchronously
    win._poll()

    r = win.results[str(log.resolve())]
    assert "res" in r and r["report"].exists()
    assert len(win.tree.get_children()) > 5          # checklist filled
    assert win.plot_keys                              # plots available
    assert any(k == "Drag area (coast)" for k, _ in r["headline"])
