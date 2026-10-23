"""Formula SAE / Formula Student rule checks on a vsim vehicle project.

    python -m vsim.rules templates/projects/example_fs_ev [--json]

Checks the dimensional and electrical limits that the vehicle files can
show (FSAE EV rules as of 2024/2025; verify against the current rule book
of your event: limits and numbering change from year to year). Geometry
from CAD (wing positions, envelope) is checked by the vehicle builder.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys
from dataclasses import asdict, dataclass

import yaml


@dataclass
class Check:
    topic: str
    ok: bool
    value: str
    limit: str


def _load(path: pathlib.Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _overlay(base: dict, top: dict) -> dict:
    """Maps merge key by key; anything else (lists, scalars) in top replaces."""
    out = dict(base)
    for key, value in top.items():
        out[key] = _overlay(out[key], value) if isinstance(value, dict) and isinstance(out.get(key), dict) else value
    return out


def load_project(project: pathlib.Path) -> tuple[dict, dict]:
    vehicle = _load(project / "vehicle" / "vehicle.yaml")
    # Values measured by the vehicle builder override vehicle.yaml, as in vsim-core.
    builder = project / "vehicle" / "builder.yaml"
    if builder.exists():
        vehicle = _overlay(vehicle, _load(builder))
    control_file = project / "control" / "vcu.yaml"
    control = _load(control_file) if control_file.exists() else {}
    return vehicle, control


def check(vehicle: dict, control: dict) -> list[Check]:
    checks: list[Check] = []

    def add(topic: str, ok: bool, value: str, limit: str) -> None:
        checks.append(Check(topic, bool(ok), value, limit))

    wheelbase = float(vehicle.get("wheelbase_m", 0.0))
    add("wheelbase", wheelbase >= 1.525, f"{wheelbase * 1000:.0f} mm", ">= 1525 mm")

    front, rear = float(vehicle.get("track_front_m", 0.0)), float(vehicle.get("track_rear_m", 0.0))
    if front > 0 and rear > 0:
        ratio = min(front, rear) / max(front, rear)
        add("track ratio (smaller / larger)", ratio >= 0.75, f"{ratio * 100:.1f} %", ">= 75 %")

    wheel = vehicle.get("wheel", {}) or {}
    rim = wheel.get("rim_diameter_in")
    if rim is not None:
        add("wheel diameter", float(rim) >= 8.0, f'{float(rim):.0f}"', '>= 8"')

    battery = vehicle.get("battery", {}) or {}
    series = int(battery.get("cells_series", 0))
    parallel = int(battery.get("cells_parallel", 1))
    v_max = float(battery.get("cell_voltage_max_v", 4.2))
    if series:
        add("tractive system voltage", series * v_max <= 600.0, f"{series * v_max:.0f} V", "<= 600 V DC")
        segments = int(battery.get("segments", 0))
        if segments:
            per = series / segments
            capacity_ah = float(battery.get("cell_capacity_ah", 0.0)) * parallel
            ocv = battery.get("ocv_v") or [3.7]
            mean_cell_v = sum(ocv) / len(ocv)
            energy_mj = per * mean_cell_v * capacity_ah * 3600 / 1e6
            add("accumulator segment voltage", per * v_max <= 120.0, f"{per * v_max:.0f} V", "<= 120 V")
            add("accumulator segment energy", energy_mj <= 6.0, f"{energy_mj:.1f} MJ", "<= 6 MJ")
            add("segments split evenly", series % segments == 0, f"{series} / {segments}", "whole cells per segment")
        else:
            add("accumulator segments given", False, "battery.segments missing", "segment count")

    power = control.get("power_limit_kw")
    if power is not None:
        add("tractive power limit", float(power) <= 80.0, f"{float(power):.0f} kW", "<= 80 kW")
    else:
        add("tractive power limit set", False, "control.power_limit_kw missing", "<= 80 kW")

    brakes = vehicle.get("brakes", {}) or {}
    add("brakes on both axles", float(brakes.get("front_gain_nm_per_bar", 0)) > 0
        and float(brakes.get("rear_gain_nm_per_bar", 0)) > 0, "front and rear", "acts on all four wheels")
    return checks


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("project", type=pathlib.Path, help="vehicle project directory (contains vehicle/vehicle.yaml)")
    parser.add_argument("--json", action="store_true", help="print the checks as JSON")
    args = parser.parse_args(argv)
    if not (args.project / "vehicle" / "vehicle.yaml").exists():
        print(f"error: {args.project / 'vehicle' / 'vehicle.yaml'} not found", file=sys.stderr)
        return 2
    vehicle, control = load_project(args.project)
    checks = check(vehicle, control)
    if args.json:
        print(json.dumps([asdict(c) for c in checks], ensure_ascii=False, indent=2))
    else:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(errors="replace")
        width = max(len(c.topic) for c in checks)
        for c in checks:
            print(f"{'PASS' if c.ok else 'FAIL'}  {c.topic:<{width}}  {c.value:<14} {c.limit}")
        failed = sum(not c.ok for c in checks)
        print(f"\n{len(checks) - failed} / {len(checks)} checks passed (FSAE EV limits; confirm with your rule book)")
    return 0 if all(c.ok for c in checks) else 1


if __name__ == "__main__":
    sys.exit(main())
