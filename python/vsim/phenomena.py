"""Phenomenon detection and cause isolation (design doc §13.3).

    python -m vsim.phenomena templates/projects/example_fs_ev/scenarios/acceleration.yaml
    python -m vsim.phenomena <scenario> --isolate --signals vcu.vx_est sens.imu.ax --report out.md
    python -m vsim.phenomena <scenario> --isolate without      # all real but one, per category

Runs a scenario with every realism category real and lists what happened:
safety cuts (APPS, brake plausibility), traction control, wheel speed
dropouts, GNSS outages, battery limits, CAN timeouts, bus-off and bus
overload, and the dominant frequencies of the
sensor channels (vibration, aliasing). With --isolate it also runs the
scenario all-ideal and once per realism category (only that one real, or with
--isolate without: all but that one), and shows how far each run moves the
chosen signals from the ideal run: which non-ideality causes what.

From Python:

    from vsim import phenomena
    rec = phenomena.run(scenario, realism="real")
    for e in phenomena.events(rec): print(e)
    table = phenomena.isolate(scenario, ["vcu.vx_est"])
"""

import argparse
import fnmatch
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import run_scenario
from .testing import Series, spectral_peak, spectrum

CATEGORIES = ["environment", "sensors", "actuators", "electrical", "comms", "variation"]


def run(scenario, *, realism="real", duration=None, overrides=None):
    """Runs a scenario with every realism category set to `realism` ("real" or
    "ideal"), or per category from a dict; returns the recording."""
    ov = dict(overrides or {})
    for category in CATEGORIES:
        mode = realism.get(category, "ideal") if isinstance(realism, dict) else realism
        ov[f"realism.{category}"] = mode
    ov.setdefault("logging.signals", "*")
    return run_scenario(scenario, duration=duration, overrides=ov)


@dataclass
class Event:
    t: float          # start [s]
    kind: str
    detail: str
    duration: float = 0.0  # [s]; 0 for an instant

    def __str__(self):
        span = f" for {self.duration:.3f} s" if self.duration > 0 else ""
        return f"{self.t:8.3f} s  {self.kind:<22} {self.detail}{span}"


def _intervals(t, cond):
    """(start, end) times of the runs where cond is true."""
    cond = np.asarray(cond, dtype=bool)
    if cond.size == 0:
        return []
    edges = np.diff(cond.astype(np.int8))
    starts = list(np.flatnonzero(edges == 1) + 1)
    ends = list(np.flatnonzero(edges == -1) + 1)
    if cond[0]:
        starts.insert(0, 0)
    if cond[-1]:
        ends.append(cond.size - 1)
    return [(float(t[a]), float(t[b])) for a, b in zip(starts, ends)]


def _match(rec, pattern):
    return sorted(k for k in rec if k != "t" and fnmatch.fnmatchcase(k, pattern))


def events(rec, *, min_duration=0.0):
    """What happened in a run, in time order."""
    # The first row is logged before any block has run (no VCU output, no
    # GNSS fix yet): leave it out.
    rec = {k: np.asarray(v)[1:] for k, v in rec.items()}
    t = rec["t"]
    out = []

    def add(name, kind, detail, cond):
        for a, b in _intervals(t, cond):
            if b - a >= min_duration:
                out.append(Event(a, kind, detail, b - a))

    if "vcu.apps_implausible" in rec:
        add("vcu.apps_implausible", "apps_implausible", "accelerator sensors disagree or out of range: torque cut",
            rec["vcu.apps_implausible"] > 0.5)
    if "vcu.bspc" in rec:
        add("vcu.bspc", "brake_plausibility", "braking with the accelerator pressed: torque cut",
            rec["vcu.bspc"] > 0.5)
    for name in _match(rec, "vcu.motor.*.tc"):
        motor = name.split(".")[2]
        tc = rec[name]
        for a, b in _intervals(t, tc < 0.95):
            if b - a >= min_duration:
                window = (t >= a) & (t <= b)
                out.append(Event(a, "traction_control", f"motor {motor}: torque down to {tc[window].min():.0%}",
                                 b - a))
    speed = rec.get("veh.chassis.speed")
    for name in _match(rec, "sens.*.omega"):
        sensor = name.split(".")[1]
        if f"sens.{sensor}.pulses" in rec and speed is not None:
            add(name, "wheel_speed_dropout", f"{sensor} reads 0 while moving", (rec[name] == 0.0) & (speed > 0.5))
    for name in _match(rec, "sens.*.valid"):
        add(name, "gnss_outage", f"{name.split('.')[1]} has no fix", rec[name] < 0.5)
    for name in _match(rec, "can.*.timeout"):
        add(name, "can_timeout", f"{name[4:-8]}: no frame within its timeout", rec[name] > 0.5)
    for name in _match(rec, "can.*.bus_off"):
        add(name, "can_bus_off", f"node {name[4:-8]} is bus-off", rec[name] > 0.5)
    for name in _match(rec, "can.*.load"):
        add(name, "can_bus_load", f"bus {name[4:-5]} above 80 % load", rec[name] > 0.8)
    # Electrical network (design doc §12.3).
    for name in _match(rec, "elec.*.resets"):
        ecu = name[5:-7]
        if f"elec.{ecu}.on" in rec:
            add(name, "ecu_reset", f"{ecu} without supply (reset or booting)", rec[f"elec.{ecu}.on"] < 0.5)
    for name in _match(rec, "elec.*.blown"):
        add(name, "fuse_blown", f"fuse {name[5:-6]} blown", rec[name] > 0.5)
    for name in _match(rec, "elec.*.chatter"):
        add(name, "contactor_chatter", f"{name[5:-8]} opens and closes repeatedly", rec[name] > 0.5)
    for name in _match(rec, "elec.*.violation"):
        add(name, "precharge_violation", f"{name[5:-10]}: AIR closed below the precharge rule", rec[name] > 0.5)
    for name in _match(rec, "elec.*.close_ratio"):
        pcc = name[5:-12]
        if f"elec.{pcc}.state" in rec:
            add(name, "precharge_fail", f"{pcc}: precharge timed out", rec[f"elec.{pcc}.state"] > 3.5)
    for name in _match(rec, "elec.*.overload"):
        add(name, "wire_overheat", f"wire {name[5:-9]} above its rated temperature", rec[name] > 0.5)
    for name in _match(rec, "elec.*.limited"):
        add(name, "dcdc_limit", f"{name[5:-8]} at its output current limit", rec[name] > 0.5)
    if "bat.overloaded" in rec:
        add("bat.overloaded", "battery_limit", "battery at its current or voltage limit", rec["bat.overloaded"] > 0.5)
    out.sort(key=lambda e: e.t)
    return out


def peaks(rec, patterns=("sens.*.a?", "sens.*.g?"), *, fmin=2.0, fmax=None, prominence=10.0, start=0.0):
    """Dominant spectral peak of each matching signal from time `start` on,
    when it stands out (amplitude above `prominence` times the median of its
    spectrum): [(signal, frequency [Hz], amplitude)]. Spectra are taken at the
    logging rate, so log at the sensors' rate (logging.rate_hz) to see them."""
    t = np.asarray(rec["t"])
    keep = t >= start
    found = []
    for pattern in patterns:
        for name in _match(rec, pattern):
            s = Series(t[keep], np.asarray(rec[name], dtype=float)[keep])
            f, amp = spectrum(s)
            hi = fmax if fmax is not None else f[-1]
            freq, a = spectral_peak(s, fmin, hi)
            band = (f >= fmin) & (f <= hi)
            floor = float(np.median(amp[band])) if np.any(band) else 0.0
            if a > prominence * floor and a > 0.0:
                found.append((name, freq, a))
    return found


def _rms_difference(a, b):
    n = min(len(a), len(b))
    d = np.asarray(a[:n], dtype=float) - np.asarray(b[:n], dtype=float)
    return float(np.sqrt(np.mean(d * d))) if n else math.nan


def isolate(scenario, signals, *, duration=None, overrides=None, categories=CATEGORIES, mode="only"):
    """RMS deviation of each signal from the all-ideal run with all categories
    real and, per category, with only that one real (mode "only": what it
    causes on its own) or with all but that one real (mode "without": what
    disappears without it): {"all": {signal: rms}, "<category>": {...}, ...}."""
    if mode not in ("only", "without"):
        raise ValueError("mode must be 'only' or 'without'")
    ideal = run(scenario, realism="ideal", duration=duration, overrides=overrides)
    missing = [s for s in signals if s not in ideal]
    if missing:
        raise KeyError(f"not recorded: {', '.join(missing)}")
    table = {}
    runs = [("all", "real")]
    for c in categories:
        runs.append((c, {c: "real"} if mode == "only" else {o: "real" for o in CATEGORIES if o != c}))
    for label, realism in runs:
        rec = run(scenario, realism=realism, duration=duration, overrides=overrides)
        table[label] = {s: _rms_difference(rec[s], ideal[s]) for s in signals}
    return table


def markdown(scenario, found_events, found_peaks, table=None):
    lines = [f"# Phenomena: {Path(str(scenario)).name}", "", "## Events (all categories real)", ""]
    if found_events:
        lines += ["| t [s] | event | detail | duration [s] |", "|---:|---|---|---:|"]
        lines += [f"| {e.t:.3f} | {e.kind} | {e.detail} | {e.duration:.3f} |" for e in found_events]
    else:
        lines.append("No events.")
    lines += ["", "## Spectral peaks", ""]
    if found_peaks:
        lines += ["| signal | frequency [Hz] | amplitude |", "|---|---:|---:|"]
        lines += [f"| {n} | {f:.2f} | {a:.4g} |" for n, f, a in found_peaks]
    else:
        lines.append("No dominant peaks.")
    if table:
        signals = list(next(iter(table.values())).keys())
        lines += ["", "## Cause isolation (RMS deviation from the all-ideal run)", "",
                  "| real | " + " | ".join(signals) + " |", "|---|" + "---:|" * len(signals)]
        for label, row in table.items():
            lines.append(f"| {label} | " + " | ".join(f"{row[s]:.4g}" for s in signals) + " |")
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m vsim.phenomena",
                                     description="Detect phenomena in a scenario run and trace them to their cause.")
    parser.add_argument("scenario")
    parser.add_argument("--duration", type=float, help="seconds to run (default: the scenario's t_end)")
    parser.add_argument("--isolate", nargs="?", const="only", choices=["only", "without"],
                        help="rerun per realism category to find the cause: with only that category real "
                             "(default) or with all but that one")
    parser.add_argument("--signals", nargs="+", default=["vcu.vx_est"], help="signals compared by --isolate")
    parser.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE", help="scenario overrides")
    parser.add_argument("--report", help="write a Markdown report here")
    args = parser.parse_args(argv)
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    overrides = dict(item.split("=", 1) for item in args.set)

    rec = run(args.scenario, realism="real", duration=args.duration, overrides=overrides)
    found_events = events(rec)
    found_peaks = peaks(rec)
    print(f"Events ({len(found_events)}):")
    for e in found_events:
        print("  " + str(e))
    print("Spectral peaks:")
    for name, f, a in found_peaks:
        print(f"  {name:<28} {f:8.2f} Hz  {a:.4g}")
    table = None
    if args.isolate:
        table = isolate(args.scenario, args.signals, duration=args.duration, overrides=overrides, mode=args.isolate)
        print(f"Cause isolation (RMS deviation from the all-ideal run; per category: {args.isolate} it real):")
        print("  " + "real".ljust(12) + "".join(s.rjust(20) for s in args.signals))
        for label, row in table.items():
            print("  " + label.ljust(12) + "".join(f"{row[s]:20.4g}" for s in args.signals))
    if args.report:
        Path(args.report).write_text(markdown(args.scenario, found_events, found_peaks, table), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
