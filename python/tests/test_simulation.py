import json
import pathlib
import time

import numpy as np
import pytest

import vsim


def test_version():
    assert vsim.__version__
    assert isinstance(vsim.git_commit, str)


def test_run_scenario_returns_recording(scenario):
    rec = vsim.run_scenario(scenario("sine_steer"), duration=10.0)
    assert len(rec["t"]) == 1001
    assert rec["t"][-1] == pytest.approx(10.0)
    for name, values in rec.items():
        assert np.all(np.isfinite(values)), name
    # The speed hold settles near 40 km/h.
    assert rec["veh.chassis.vx"][-1] == pytest.approx(40 / 3.6, abs=0.3)


def test_deterministic(scenario):
    a = vsim.run_scenario(scenario("sine_steer"), duration=12.0)
    b = vsim.run_scenario(scenario("sine_steer"), duration=12.0)
    assert a.keys() == b.keys()
    for name in a:
        np.testing.assert_array_equal(a[name], b[name])


def test_faster_than_real_time(scenario):
    sim = vsim.Simulation(scenario("sine_steer"), record=False)
    start = time.perf_counter()
    sim.run(30.0)
    factor = 30.0 / (time.perf_counter() - start)
    print(f"real-time factor: {factor:.0f}x")
    assert factor > 20.0


def test_overrides_accept_python_types(scenario):
    sim = vsim.Simulation(
        scenario("straight_accel"),
        overrides={"vehicle.mass_kg": 400, "control.power_limit_kw": 40.5, "driver.type": "external"},
    )
    sim.set_driver_command(throttle=1.0)
    sim.run(1.0)
    assert sim.get("drv.cmd.throttle") == 1.0
    assert sim.get("pt.power_mech") <= 40.5e3 * 1.01
    with pytest.raises(TypeError):
        vsim.Simulation(scenario("straight_accel"), overrides={"vehicle.mass_kg": [1]})


def test_heavier_car_accelerates_slower(scenario):
    light = vsim.run_scenario(scenario("straight_accel"), duration=3.0)
    heavy = vsim.run_scenario(
        scenario("straight_accel"), duration=3.0, overrides={"vehicle.mass_kg": 400}
    )
    assert heavy["veh.chassis.vx"][-1] < light["veh.chassis.vx"][-1]


def test_reset_restarts(scenario):
    with vsim.Simulation(scenario("sine_steer")) as sim:
        sim.run(5.0)
        x = sim.get("veh.chassis.x")
        sim.reset()
        assert sim.time == 0.0
        sim.run(5.0)
        assert sim.get("veh.chassis.x") == x


def test_errors_are_python_exceptions(scenario):
    with pytest.raises(IndexError):
        vsim.Simulation(scenario("sine_steer")).get("no.such.signal")
    with pytest.raises(ValueError):
        vsim.Simulation(scenario("sine_steer"), overrides={"driver.type": "telepathy"})
    with pytest.raises(RuntimeError):
        vsim.Simulation("does/not/exist.yaml")


def test_log_dir_writes_readable_mcap(scenario, tmp_path):
    mcap_reader = pytest.importorskip("mcap.reader")
    with vsim.Simulation(scenario("sine_steer"), log_dir=tmp_path) as sim:
        sim.run(2.0)
        run_dir = sim.run_dir
    assert run_dir is not None

    run_dir = pathlib.Path(run_dir)
    assert run_dir.parent == tmp_path
    meta = json.loads((run_dir / "meta.json").read_text(encoding="utf-8"))
    assert meta["scenario"] == "sine_steer"
    assert meta["seed"] == 1

    times = []
    with open(run_dir / "log.mcap", "rb") as f:
        for _schema, channel, message in mcap_reader.make_reader(f).iter_messages():
            assert channel.topic == "signals"
            record = json.loads(message.data)
            assert "veh.chassis.vx" in record
            times.append(record["t"])
    assert len(times) == 201
    assert times == sorted(times)
