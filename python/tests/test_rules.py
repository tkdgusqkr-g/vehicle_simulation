import copy
import pathlib

from vsim import rules

PROJECT = pathlib.Path(__file__).resolve().parents[2] / "templates" / "projects" / "example_fs_ev"


def test_example_vehicle_meets_the_fsae_limits():
    vehicle, control = rules.load_project(PROJECT)
    failed = [c for c in rules.check(vehicle, control) if not c.ok]
    assert not failed, failed


def test_violations_are_reported():
    vehicle, control = rules.load_project(PROJECT)
    bad = copy.deepcopy(vehicle)
    bad["wheelbase_m"] = 1.50
    bad["track_rear_m"] = 0.8
    bad["battery"]["cells_series"] = 150      # 630 V, 21.4 per segment
    bad_control = dict(control, power_limit_kw=100)
    failed = {c.topic for c in rules.check(bad, bad_control) if not c.ok}
    assert {"wheelbase", "track ratio (smaller / larger)", "tractive system voltage",
            "tractive power limit", "segments split evenly"} <= failed


def test_cli_exit_codes(capsys):
    assert rules.main([str(PROJECT)]) == 0
    assert "checks passed" in capsys.readouterr().out
    assert rules.main([str(PROJECT / "missing")]) == 2


def test_builder_values_override_vehicle_yaml(tmp_path):
    project = tmp_path / "car"
    (project / "vehicle").mkdir(parents=True)
    (project / "vehicle" / "vehicle.yaml").write_text((PROJECT / "vehicle" / "vehicle.yaml").read_text(encoding="utf-8"), encoding="utf-8")
    (project / "vehicle" / "builder.yaml").write_text("wheelbase_m: 1.50\nwheel: {unloaded_radius_m: 0.25}\n", encoding="utf-8")
    vehicle, _ = rules.load_project(project)
    assert vehicle["wheelbase_m"] == 1.50
    assert vehicle["wheel"]["unloaded_radius_m"] == 0.25
    assert vehicle["wheel"]["rim_diameter_in"] == 10       # the rest of the map stays
    failed = {c.topic for c in rules.check(vehicle, {}) if not c.ok}
    assert "wheelbase" in failed


def test_pedal_sensor_and_plausibility_rules():
    vehicle, control = rules.load_project(PROJECT)
    bad = copy.deepcopy(vehicle)
    apps = next(s for s in bad["sensors"] if s["type"] == "pedal")
    apps["ch2_v"] = apps["ch1_v"]                       # identical tracks
    bad["sensors"] = [s for s in bad["sensors"] if s["type"] != "brake_pressure"]
    bad_control = dict(control, apps={"max_deviation": 0.2}, bspc={"apps_on": 0.3})
    failed = {c.topic for c in rules.check(bad, bad_control) if not c.ok}
    assert failed == {"APPS: different transfer functions", "BSE: brake system encoder", "APPS plausibility",
                      "brake / APPS plausibility"}
