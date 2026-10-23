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
