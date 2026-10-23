"""Correlation with measured data (design doc §13.4, R-26..R-29).

Replays a measured run's driver inputs through the simulator, compares the
car's response channel by channel, identifies parameters that make them
agree, and identifies sensor noise from a standstill record.

A mapping file connects the log's columns to the simulator:

    time: time_s                      # time column of the log [s]
    inputs:                           # replayed as the driver (maneuver profiles)
      steer_deg: {to: steer}          # steering wheel angle [deg]
      apps_pct: {to: throttle, scale: 0.01}
      brake_bar: {to: brake, scale: 0.0167}
    channels:                         # compared: measured x scale + offset = signal
      speed_kph: {signal: veh.chassis.speed, scale: 0.27778}
      yaw_dps: {signal: veh.chassis.yaw_rate, scale: 0.0174533}

    python -m vsim.correlate replay --scenario scenarios/replay.yaml --log run3.csv --map run3.map.yaml
    python -m vsim.correlate identify --scenario scenarios/replay.yaml --log run3.csv --map run3.map.yaml \\
        --param vehicle.mass_kg=280:250:330 --param vehicle.aero.cda_m2=1.3:0.8:2.0 --fit 0 20
    python -m vsim.correlate allan --log still.csv --column ax --scale 9.81

Logs are CSV with a header row, or a vsim log.mcap (a run compared with
another, e.g. before and after a change).
"""

import argparse
import math
import pathlib
import sys
import tempfile

import numpy as np
import yaml

INPUTS = ("steer", "throttle", "brake")


# --- Logs and mappings --------------------------------------------------------------------

def read_log(path, columns=None):
    """Columns of a CSV (header row) or a vsim log.mcap as float arrays."""
    path = pathlib.Path(path)
    if path.suffix == ".mcap":
        from .profiles import read_mcap
        return read_mcap(path, tuple(c for c in (columns or []) if c != "t"))
    with open(path, encoding="utf-8") as f:
        header = [h.strip().strip('"') for h in f.readline().split(",")]
        rows = [line.split(",") for line in f if line.strip()]
    data = {}
    for i, name in enumerate(header):
        data[name] = np.array([float(r[i]) if i < len(r) and r[i].strip() else math.nan for r in rows])
    return data


def load_mapping(mapping):
    """A mapping file (path) or dict, normalised: time, inputs {column: (to, scale, offset)},
    channels {column: (signal, scale, offset)}."""
    if not isinstance(mapping, dict):
        mapping = yaml.safe_load(pathlib.Path(mapping).read_text(encoding="utf-8")) or {}

    def entry(value, key):
        if isinstance(value, str):
            return value, 1.0, 0.0
        return value[key], float(value.get("scale", 1.0)), float(value.get("offset", 0.0))

    inputs = {col: entry(v, "to") for col, v in (mapping.get("inputs") or {}).items()}
    for col, (to, _, _) in inputs.items():
        if to not in INPUTS:
            raise ValueError(f"inputs.{col}: 'to' must be one of {', '.join(INPUTS)}")
    channels = {col: entry(v, "signal") for col, v in (mapping.get("channels") or {}).items()}
    return {"time": mapping.get("time", "t"), "inputs": inputs, "channels": channels}


def measured(log, mapping):
    """Measured channels in simulator units: {signal: (t, values)}."""
    m = load_mapping(mapping)
    t = np.asarray(log[m["time"]], dtype=float)
    out = {}
    for col, (signal, scale, offset) in m["channels"].items():
        if col not in log:
            raise KeyError(f"log has no column '{col}'")
        out[signal] = (t - t[0], scale * np.asarray(log[col], dtype=float) + offset)
    return out


# --- Replay -------------------------------------------------------------------------------

def replay_overrides(log, mapping, directory):
    """Overrides that replay the log's inputs (written to `directory`/inputs.csv)."""
    m = load_mapping(mapping)
    t = np.asarray(log[m["time"]], dtype=float)
    t = t - t[0]
    columns = {"t": t}
    for col, (to, scale, offset) in m["inputs"].items():
        if col not in log:
            raise KeyError(f"log has no column '{col}'")
        columns[to] = scale * np.asarray(log[col], dtype=float) + offset
    path = pathlib.Path(directory) / "inputs.csv"
    names = list(columns)
    with open(path, "w", encoding="utf-8") as f:
        f.write(",".join(names) + "\n")
        for row in zip(*(columns[n] for n in names)):
            f.write(",".join(f"{v:.9g}" for v in row) + "\n")
    overrides = {"driver.type": "maneuver", "time.t_end": f"{t[-1]:.6f}"}
    for to in INPUTS:
        if to in columns:
            overrides.update({f"driver.{to}.type": "file", f"driver.{to}.path": str(path.resolve()),
                              f"driver.{to}.column": to, f"driver.{to}.time_column": "t"})
    speed = measured(log, mapping).get("veh.chassis.speed")
    if speed is not None:
        overrides["initial.speed_mps"] = f"{max(0.0, float(speed[1][0])):.6f}"
    return overrides


def replay(scenario, log, mapping, overrides=None):
    """Runs the scenario with the log's inputs; returns the recording."""
    from . import Simulation

    with tempfile.TemporaryDirectory() as tmp:
        ov = replay_overrides(log, mapping, tmp)
        ov.update({str(k): str(v) for k, v in (overrides or {}).items()})
        ov.setdefault("logging.signals", "*")
        with Simulation(str(scenario), overrides=ov) as sim:
            sim.run(None)
            return {k: np.asarray(v, dtype=float) for k, v in sim.recording().items()}


# --- Metrics ------------------------------------------------------------------------------

def metrics(t_sim, v_sim, t_meas, v_meas, window=None, max_lag=0.5):
    """RMSE, NRMSE (by the measured range), peak error, correlation, the delay of the
    simulation behind the measurement [s] and the final value difference, on the
    simulation's time base within the overlap (and `window`)."""
    t_sim, v_sim = np.asarray(t_sim, float), np.asarray(v_sim, float)
    t_meas, v_meas = np.asarray(t_meas, float), np.asarray(v_meas, float)
    lo, hi = max(t_sim[0], t_meas[0]), min(t_sim[-1], t_meas[-1])
    if window:
        lo, hi = max(lo, window[0]), min(hi, window[1])
    keep = (t_sim >= lo) & (t_sim <= hi)
    t, s = t_sim[keep], v_sim[keep]
    good = np.isfinite(v_meas)
    m = np.interp(t, t_meas[good], v_meas[good])
    if len(t) < 3:
        raise ValueError("too little overlap between the simulation and the log")
    e = s - m
    span = float(np.ptp(m)) or 1.0
    corr = float(np.corrcoef(s, m)[0, 1]) if np.std(s) > 0 and np.std(m) > 0 else math.nan
    # Delay: the shift of the simulation that best lines it up with the measurement.
    dt = float(np.median(np.diff(t)))
    lags = int(max_lag / dt)
    sd, md = s - s.mean(), m - m.mean()
    best, lag = -math.inf, 0
    for k in range(-lags, lags + 1):
        a, b = (sd[k:], md[:len(md) - k]) if k >= 0 else (sd[:k], md[-k:])
        if len(a) > 10:
            c = float(np.dot(a, b)) / len(a)
            if c > best:
                best, lag = c, k
    return {"rmse": float(np.sqrt(np.mean(e * e))), "nrmse": float(np.sqrt(np.mean(e * e))) / span,
            "peak_error": float(np.max(np.abs(e))), "correlation": corr, "lag_s": lag * dt,
            "final_diff": float(s[-1] - m[-1]), "n": int(len(t))}


def compare(recording, log, mapping, window=None):
    """Metrics per mapped signal: {signal: metrics}."""
    out = {}
    t_sim = recording["t"]
    for signal, (t, v) in measured(log, mapping).items():
        if signal not in recording:
            raise KeyError(f"the simulation did not log '{signal}'")
        out[signal] = metrics(t_sim, recording[signal], t, v, window)
    return out


# --- Identification -----------------------------------------------------------------------

def _residuals(recording, meas, window):
    parts = []
    t_sim = recording["t"]
    for signal, (t, v) in meas.items():
        lo, hi = max(t_sim[0], t[0]), min(t_sim[-1], t[-1])
        if window:
            lo, hi = max(lo, window[0]), min(hi, window[1])
        keep = (t_sim >= lo) & (t_sim <= hi)
        m = np.interp(t_sim[keep], t, v)
        scale = float(np.std(m)) or 1.0
        parts.append((recording[signal][keep] - m) / scale / math.sqrt(max(1, keep.sum())))
    return np.concatenate(parts)


def identify(scenario, log, mapping, params, *, fit=None, overrides=None, iterations=10, progress=None):
    """Parameters that make the replay match the log (Levenberg-Marquardt, forward
    difference Jacobian, bounds by clipping). `params`: {key: (start, low, high)}.
    Fits over `fit` = (t0, t1) (all if None); the rest of the log is for checking.
    Returns {"values", "cost", "history", "fit", "check" (metrics on the rest, or None)}."""
    meas = measured(log, mapping)
    keys = list(params)
    x = np.array([float(params[k][0]) for k in keys])
    lo = np.array([float(params[k][1]) for k in keys])
    hi = np.array([float(params[k][2]) for k in keys])
    if np.any(lo >= hi) or np.any(x < lo) or np.any(x > hi):
        raise ValueError("each parameter needs low < high and its start between them")
    scale = (hi - lo) / 10.0

    def run(values):
        ov = dict(overrides or {})
        ov.update({k: f"{v:.9g}" for k, v in zip(keys, values)})
        return replay(scenario, log, mapping, ov)

    def cost_of(rec):
        r = _residuals(rec, meas, fit)
        return r, float(r @ r)

    rec = run(x)
    r, cost = cost_of(rec)
    history = [(x.copy(), cost)]
    lam = 1e-2
    for _ in range(iterations):
        jac = np.empty((len(r), len(keys)))
        for j in range(len(keys)):
            step = np.zeros(len(keys))
            step[j] = 0.1 * scale[j]  # 1 % of the range
            xj = np.clip(x + step, lo, hi)
            if xj[j] == x[j]:
                xj = np.clip(x - step, lo, hi)
            rj, _ = cost_of(run(xj))
            jac[:, j] = (rj - r) / ((xj[j] - x[j]) / scale[j])
        a, g = jac.T @ jac, jac.T @ r
        improved = False
        for _ in range(6):
            delta = -np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-12), g) * scale
            x_new = np.clip(x + delta, lo, hi)
            rec_new = run(x_new)
            r_new, cost_new = cost_of(rec_new)
            if cost_new < cost:
                x, r, cost, rec = x_new, r_new, cost_new, rec_new
                lam = max(lam / 3.0, 1e-6)
                improved = True
                break
            lam *= 4.0
        history.append((x.copy(), cost))
        if progress:
            progress(len(history) - 1, dict(zip(keys, map(float, x))), cost)
        if not improved or history[-2][1] - cost < 1e-4 * history[-2][1]:
            break
    check = None
    if fit:
        t_end = float(rec["t"][-1])
        rest = (fit[1], t_end) if fit[1] < t_end - 0.5 else None
        if rest:
            check = compare(rec, log, mapping, rest)
    return {"values": dict(zip(keys, map(float, x))), "cost": cost, "history": history,
            "fit": compare(rec, log, mapping, fit), "check": check, "recording": rec}


# --- Sensor noise from a standstill record --------------------------------------------------

def allan_deviation(values, rate, points=40):
    """Overlapping Allan deviation: (taus, deviations) at log-spaced cluster sizes."""
    y = np.asarray(values, dtype=float)
    y = y[np.isfinite(y)]
    tau0 = 1.0 / rate
    x = np.concatenate([[0.0], np.cumsum(y) * tau0])
    sizes = np.unique(np.logspace(0, math.log10(max(1, len(y) // 9)), points).astype(int))
    taus, devs = [], []
    for m in sizes:
        d = x[2 * m:] - 2.0 * x[m:-m] + x[:-2 * m]
        tau = m * tau0
        taus.append(tau)
        devs.append(math.sqrt(float(np.mean(d * d)) / (2.0 * tau * tau)))
    return np.array(taus), np.array(devs)


def identify_noise(values, rate):
    """Sensor channel parameters from a standstill record (vsim units per second):
    noise_density [unit/sqrt(Hz)] from the -1/2 slope, bias_instability [unit] from
    the floor (Allan minimum / 0.664), random_walk [unit sqrt(Hz)] from a +1/2
    slope at long times (0 if the record does not reach one)."""
    taus, devs = allan_deviation(values, rate)
    short = taus <= 10.0 * taus[0]
    noise = float(np.median(devs[short] * np.sqrt(taus[short])))
    floor = int(np.argmin(devs))
    walk = 0.0
    if floor < len(taus) - 3:
        tail_t, tail_d = taus[-4:], devs[-4:]
        slope = np.polyfit(np.log(tail_t), np.log(tail_d), 1)[0]
        if slope > 0.3:
            walk = float(np.median(tail_d * np.sqrt(3.0 / tail_t)))
    return {"noise_density": noise, "bias_instability": float(devs[floor] / 0.664),
            "bias_tau_s": float(taus[floor] / 1.89), "random_walk": walk}


# --- Report and command line ----------------------------------------------------------------

def report(results, title="Correlation"):
    lines = [f"# {title}", "", "| signal | RMSE | NRMSE | peak | correlation | delay [s] | final diff |",
             "|---|---|---|---|---|---|---|"]
    for signal, m in results.items():
        lines.append(f"| `{signal}` | {m['rmse']:.4g} | {100 * m['nrmse']:.1f} % | {m['peak_error']:.4g} | "
                     f"{m['correlation']:.3f} | {m['lag_s']:+.3f} | {m['final_diff']:+.4g} |")
    return "\n".join(lines) + "\n"


def _param(text):
    key, _, spec = text.partition("=")
    parts = [float(p) for p in spec.split(":")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--param key=start:low:high")
    return key, tuple(parts)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m vsim.correlate", description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("replay", "identify"):
        p = sub.add_parser(name)
        p.add_argument("--scenario", required=True)
        p.add_argument("--log", required=True)
        p.add_argument("--map", required=True)
        p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
        p.add_argument("--report")
    sub.choices["identify"].add_argument("--param", action="append", type=_param, required=True)
    sub.choices["identify"].add_argument("--fit", nargs=2, type=float, metavar=("T0", "T1"))
    sub.choices["identify"].add_argument("--iterations", type=int, default=10)
    a = sub.add_parser("allan")
    a.add_argument("--log", required=True)
    a.add_argument("--column", required=True)
    a.add_argument("--time", default="t")
    a.add_argument("--scale", type=float, default=1.0)
    args = parser.parse_args(argv)

    if args.command == "allan":
        log = read_log(args.log)
        t = log[args.time]
        rate = 1.0 / float(np.median(np.diff(t)))
        result = identify_noise(args.scale * log[args.column], rate)
        print(yaml.safe_dump({args.column: {k: float(f"{v:.4g}") for k, v in result.items()}}, sort_keys=False), end="")
        return 0

    log = read_log(args.log)
    overrides = dict(s.split("=", 1) for s in args.set)
    if args.command == "replay":
        text = report(compare(replay(args.scenario, log, args.map, overrides), log, args.map), "Replay")
    else:
        result = identify(args.scenario, log, args.map, dict(args.param), fit=args.fit, overrides=overrides,
                          iterations=args.iterations,
                          progress=lambda i, v, c: print(f"iteration {i}: cost {c:.4g} {v}", file=sys.stderr))
        text = "# Identification\n\n" + "\n".join(f"- `{k}` = {v:.6g}" for k, v in result["values"].items())
        text += "\n\n" + report(result["fit"], "Fit window")
        if result["check"]:
            text += "\n" + report(result["check"], "Check (outside the fit window)")
    if args.report:
        pathlib.Path(args.report).write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
