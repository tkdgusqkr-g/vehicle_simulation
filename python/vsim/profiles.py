"""Speed profiles for the MPC driver's tracking mode (driver.speed_profile).

A profile is "s_m, v_mps" rows along the track centreline (track.s), taken
from one lap of a recording: a lap driven by hand, by the racing MPC, or a
logged run of the real car converted to these two signals.

    python -m vsim.profiles runs/<run>/log.mcap -o profiles/autocross.csv
    python -m vsim.profiles scenarios/autocross_mpc.yaml --set driver.mode=race --duration 45 -o race.csv

The lap is the fastest valid one unless --lap picks another (1 = first).
"""

import argparse
import json
import math
import pathlib
import sys

import numpy as np

SIGNALS = ("track.s", "veh.chassis.speed", "track.last_lap", "track.last_lap_valid")


def read_mcap(path, signals=SIGNALS):
    """Reads signals (plus "t") from a vsim log.mcap into numpy arrays."""
    from mcap.reader import make_reader

    columns = {name: [] for name in ("t",) + tuple(signals)}
    with open(path, "rb") as f:
        for _schema, _channel, message in make_reader(f).iter_messages(topics=["signals"]):
            record = json.loads(message.data)
            for name, values in columns.items():
                values.append(record.get(name, math.nan))
    return {name: np.asarray(values, dtype=float) for name, values in columns.items()}


def laps(recording):
    """Completed laps as (start time, end time, lap time, valid), in driving order."""
    t = np.asarray(recording["t"], dtype=float)
    last = np.asarray(recording["track.last_lap"], dtype=float)
    valid = np.asarray(recording.get("track.last_lap_valid", np.ones_like(last)), dtype=float)
    out = []
    for i in range(1, len(t)):
        if last[i] > 0.0 and last[i] != last[i - 1]:
            out.append((t[i] - last[i], t[i], float(last[i]), bool(valid[i] > 0.5)))
    return out


def from_recording(recording, lap="best", step=1.0):
    """The speed along the centreline over one lap: arrays (s, v) on a regular grid of `step` m."""
    done = laps(recording)
    if not done:
        raise ValueError("the recording has no completed lap (track.last_lap)")
    if lap == "best":
        candidates = [x for x in done if x[3]] or done
        start, end, _time, _valid = min(candidates, key=lambda x: x[2])
    else:
        index = int(lap)
        if not 1 <= index <= len(done):
            raise ValueError(f"lap {index}: the recording has laps 1..{len(done)}")
        start, end, _time, _valid = done[index - 1]
    t = np.asarray(recording["t"], dtype=float)
    pick = (t >= start) & (t <= end)
    s = np.asarray(recording["track.s"], dtype=float)[pick]
    v = np.asarray(recording["veh.chassis.speed"], dtype=float)[pick]
    if s.size < 2:
        raise ValueError("the lap has too few samples")
    # track.s already lies in [0, length): a lap that wraps past the centreline's origin needs
    # nothing more than averaging into bins.
    grid = np.arange(0.0, float(np.max(s)) + step, step)
    sums = np.zeros_like(grid)
    counts = np.zeros_like(grid)
    bins = np.clip(np.round(s / step).astype(int), 0, grid.size - 1)
    np.add.at(sums, bins, v)
    np.add.at(counts, bins, 1.0)
    have = counts > 0
    speed = np.interp(grid, grid[have], sums[have] / counts[have])
    return grid, speed


def write_csv(path, s, v):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("# s_m, v_mps (vsim speed profile)\n")
        for a, b in zip(s, v):
            f.write(f"{a:.2f}, {b:.3f}\n")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m vsim.profiles", description=__doc__.split("\n\n")[0])
    parser.add_argument("source", help="a vsim log.mcap, or a scenario .yaml to run first")
    parser.add_argument("-o", "--output", required=True, help="CSV file to write")
    parser.add_argument("--lap", default="best", help="best (fastest valid, default) or a lap number from 1")
    parser.add_argument("--step", type=float, default=1.0, help="grid step along the track [m]")
    parser.add_argument("--duration", type=float, help="seconds to run a scenario (default: its t_end)")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="scenario override")
    args = parser.parse_args(argv)

    source = pathlib.Path(args.source)
    if source.suffix in (".yaml", ".yml"):
        import vsim

        overrides = dict(item.split("=", 1) for item in args.set)
        recording = vsim.run_scenario(source, duration=args.duration, overrides=overrides)
    else:
        recording = read_mcap(source)
    s, v = from_recording(recording, lap=args.lap, step=args.step)
    write_csv(args.output, s, v)
    print(f"{args.output}: {len(s)} points, {s[-1]:.0f} m, {np.min(v) * 3.6:.0f}-{np.max(v) * 3.6:.0f} km/h")
    return 0


if __name__ == "__main__":
    sys.exit(main())
