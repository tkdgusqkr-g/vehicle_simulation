import numpy as np
import pytest

import vsim
from vsim import profiles


def test_profile_from_a_lap_drives_the_mpc(scenario, tmp_path):
    rec = vsim.run_scenario(scenario("autocross_auto"), duration=45)
    done = profiles.laps(rec)
    assert len(done) >= 2
    s, v = profiles.from_recording(rec)
    assert s[0] == 0.0 and np.all(np.diff(s) == 1.0)
    assert s[-1] > 340.0  # the autocross centreline is about 350 m
    assert 5.0 < v.min() < v.max() < 30.0
    path = tmp_path / "profile.csv"
    profiles.write_csv(path, s, v)

    # The tracking MPC follows the recorded speeds instead of its own profile.
    with vsim.Simulation(scenario("autocross_mpc"), overrides={"driver.speed_profile": str(path)}) as sim:
        sim.run(6.0)
        here = sim.get("drv.mpc.s")
        assert abs(sim.get("drv.mpc.v_ref") - np.interp(here, s, v)) < 0.05


def test_profile_from_a_logged_run(scenario, tmp_path):
    pytest.importorskip("mcap.reader")
    with vsim.Simulation(scenario("autocross_auto"), log_dir=tmp_path) as sim:
        sim.run(45)
        run_dir = sim.run_dir
    out = tmp_path / "from_log.csv"
    assert profiles.main([str(run_dir) + "/log.mcap", "-o", str(out), "--lap", "2"]) == 0
    rows = np.loadtxt(out, delimiter=",", comments="#")
    assert rows.shape[1] == 2 and rows.shape[0] > 340
