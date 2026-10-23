import math

import numpy as np
import pytest

import vsim
from vsim import correlate

# An open-loop drive: launch, a left-right steer, coast, brake (two-track model for speed).
DRIVE = {"vehicle.model": "two_track", "initial.speed_mps": "8", "time.t_end": "12", "logging.signals": "*",
         "driver.steer": "{type: table, t: [0, 2, 3, 4, 5, 7, 12], values: [0, 0, 40, 0, -40, 0, 0]}",
         "driver.throttle": "{type: table, t: [0, 0.5, 6, 6.5, 12], values: [0, 0.7, 0.7, 0, 0]}",
         "driver.brake": "{type: table, t: [0, 9, 9.3, 10.5, 11, 12], values: [0, 0, 0.5, 0.5, 0, 0]}"}

# The "measured" log's own names and units.
MAPPING = {"time": "time_s",
           "inputs": {"steer_deg": {"to": "steer"}, "apps_pct": {"to": "throttle", "scale": 0.01},
                      "brake_frac": "brake"},
           "channels": {"speed_kph": {"signal": "veh.chassis.speed", "scale": 1 / 3.6},
                        "yaw_dps": {"signal": "veh.chassis.yaw_rate", "scale": math.pi / 180},
                        "ay": "veh.chassis.ay"}}


def measured_log(scenario, **truth):
    """A run of the car with `truth` parameters, written the way a logger would."""
    rec = vsim.run_scenario(scenario("replay"), overrides={**DRIVE, **truth})
    return {"time_s": rec["t"] + 100.0, "steer_deg": np.degrees(rec["drv.cmd.steer"]),
            "apps_pct": 100.0 * rec["drv.cmd.throttle"], "brake_frac": rec["drv.cmd.brake"],
            "speed_kph": 3.6 * rec["veh.chassis.speed"], "yaw_dps": np.degrees(rec["veh.chassis.yaw_rate"]),
            "ay": rec["veh.chassis.ay"]}


def test_replay_reproduces_the_same_car(scenario):
    log = measured_log(scenario)
    rec = correlate.replay(scenario("replay"), log, MAPPING, {"vehicle.model": "two_track"})
    assert rec["t"][-1] == pytest.approx(12.0, abs=0.02)
    for signal, m in correlate.compare(rec, log, MAPPING).items():
        assert m["nrmse"] < 0.01, signal
        assert m["correlation"] > 0.999, signal
        # The logged commands are the driver's 100 Hz steps; replayed, they are held once
        # more: about one driver period late, a flat cross-correlation peak.
        assert abs(m["lag_s"]) <= 0.05, signal


def test_identify_mass_and_drag(scenario, tmp_path):
    log = measured_log(scenario, **{"vehicle.mass_kg": "305", "vehicle.aero.cda_m2": "1.6"})
    nominal = correlate.compare(correlate.replay(scenario("replay"), log, MAPPING, {"vehicle.model": "two_track"}),
                                log, MAPPING)
    result = correlate.identify(scenario("replay"), log, MAPPING,
                                {"vehicle.mass_kg": (280, 250, 330), "vehicle.aero.cda_m2": (1.3, 0.8, 2.0)},
                                fit=(0, 9), overrides={"vehicle.model": "two_track"})
    assert result["values"]["vehicle.mass_kg"] == pytest.approx(305, rel=0.02)
    assert result["values"]["vehicle.aero.cda_m2"] == pytest.approx(1.6, rel=0.04)
    costs = [c for _, c in result["history"]]
    assert costs[-1] < 0.2 * costs[0]
    # Checked on the part of the log it did not fit (braking after 9 s).
    assert result["check"]["veh.chassis.speed"]["nrmse"] < 0.5 * nominal["veh.chassis.speed"]["nrmse"]


def test_metrics_find_a_delay():
    t = np.arange(0.0, 10.0, 0.01)
    measured = np.sin(2.0 * t) + 0.3 * np.sin(5.3 * t)
    late = np.sin(2.0 * (t - 0.12)) + 0.3 * np.sin(5.3 * (t - 0.12))
    m = correlate.metrics(t, late, t, measured)
    assert m["lag_s"] == pytest.approx(0.12, abs=0.011)
    same = correlate.metrics(t, measured, t, measured)
    assert same["rmse"] == 0.0 and same["correlation"] == pytest.approx(1.0)


def test_noise_from_a_standstill_record():
    rate, n = 200.0, 400_000  # 33 minutes
    rng = np.random.default_rng(3)
    white = 0.02  # [unit/sqrt(Hz)]
    walk = 2e-3   # [unit*sqrt(Hz)]: dominates the white noise beyond about 17 s
    y = white * math.sqrt(rate) * rng.standard_normal(n) + np.cumsum(walk / math.sqrt(rate) * rng.standard_normal(n))
    found = correlate.identify_noise(y, rate)
    assert found["noise_density"] == pytest.approx(white, rel=0.1)
    assert found["random_walk"] == pytest.approx(walk, rel=0.35)
    assert found["bias_instability"] > 0.0


def test_command_line(scenario, tmp_path, capsys):
    log = measured_log(scenario, **{"vehicle.mass_kg": "300"})
    csv = tmp_path / "run.csv"
    names = list(log)
    with open(csv, "w", encoding="utf-8") as f:
        f.write(",".join(names) + "\n")
        for row in zip(*(log[k] for k in names)):
            f.write(",".join(f"{v:.9g}" for v in row) + "\n")
    import yaml
    mapping = tmp_path / "run.map.yaml"
    mapping.write_text(yaml.safe_dump(MAPPING))
    report = tmp_path / "report.md"
    assert correlate.main(["replay", "--scenario", str(scenario("replay")), "--log", str(csv), "--map", str(mapping),
                           "--set", "vehicle.model=two_track", "--report", str(report)]) == 0
    assert "`veh.chassis.speed`" in report.read_text(encoding="utf-8")

    still = tmp_path / "still.csv"
    rng = np.random.default_rng(1)
    with open(still, "w", encoding="utf-8") as f:
        f.write("t,ax\n")
        for i, v in enumerate(0.01 * rng.standard_normal(20000)):
            f.write(f"{i / 100.0},{v}\n")
    capsys.readouterr()
    assert correlate.main(["allan", "--log", str(still), "--column", "ax"]) == 0
    out = yaml.safe_load(capsys.readouterr().out)
    assert out["ax"]["noise_density"] == pytest.approx(0.001, rel=0.15)  # 0.01 at 100 Hz


def test_mapping_is_checked():
    with pytest.raises(ValueError):
        correlate.load_mapping({"inputs": {"x": {"to": "gear"}}})
