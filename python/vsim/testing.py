"""Test runner (design doc §13.2): scenario + checks -> pass/fail and a report.

    python -m vsim.testing templates/projects/example_fs_ev/tests --report report.md

A test file (*.test.yaml):

    test: acceleration_75m
    description: FS acceleration under 4 s
    scenario: ../scenarios/acceleration.yaml   # relative to this file
    overrides: {vehicle.mass_kg: 300}           # optional
    duration_s: 8                               # optional; default time.t_end
    seeds: [1, 2, 3]                            # optional; one run per seed
    checks:
      - name: 75 m time
        expr: "final(track.last_lap) < 4.0"

Expressions use signal names directly (track.last_lap, veh.chassis.vx, ...)
and these functions:

    max min mean std rms abs        statistics / element-wise absolute value
    final(x) initial(x)             last / first value
    at(x, t)                        value at time t
    window(x, t0, t1)               part of a signal between two times
    time_to(x, value)               first time x reaches value (inf if never)
    rise_time(x, target, lo, hi)    time from lo*target to hi*target (0.1, 0.9)
    overshoot(x, target)            (max(x) - target) / |target|
    settle_time(x, target, tol)     last time |x - target| > tol*|target| (0.05)
    count(cond)                     how often cond becomes true
    duration(cond)                  total time cond is true
    any(cond) all(cond)
    changes(x)                      values x changes to, in order (e.g. lap times)
    peak_freq(x, f0, f1)            frequency [Hz] of the largest spectral peak in [f0, f1]
    peak_amp(x, f0, f1)             its amplitude (same unit as x)

with + - * /, comparisons, and/or/not, and [index] on changes(). `t` is
time. Expressions are interpreted, never passed to eval().
"""

import argparse
import ast
import json
import math
import sys
import time as clock
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from ._vsim import Simulation


class Series:
    """A signal: values at times."""

    def __init__(self, t, v):
        self.t = np.asarray(t, dtype=float)
        self.v = np.asarray(v)

    def _binary(self, other, op):
        if isinstance(other, Series):
            return Series(self.t, op(self.v, other.v))
        return Series(self.t, op(self.v, other))

    def __add__(self, o): return self._binary(o, np.add)
    def __radd__(self, o): return self._binary(o, lambda a, b: b + a)
    def __sub__(self, o): return self._binary(o, np.subtract)
    def __rsub__(self, o): return self._binary(o, lambda a, b: b - a)
    def __mul__(self, o): return self._binary(o, np.multiply)
    def __rmul__(self, o): return self._binary(o, lambda a, b: b * a)
    def __truediv__(self, o): return self._binary(o, np.divide)
    def __rtruediv__(self, o): return self._binary(o, lambda a, b: b / a)
    def __neg__(self): return Series(self.t, -self.v)
    def __lt__(self, o): return self._binary(o, np.less)
    def __le__(self, o): return self._binary(o, np.less_equal)
    def __gt__(self, o): return self._binary(o, np.greater)
    def __ge__(self, o): return self._binary(o, np.greater_equal)
    def __eq__(self, o): return self._binary(o, np.equal)
    def __ne__(self, o): return self._binary(o, np.not_equal)
    __hash__ = None


class CheckError(Exception):
    """A check expression that cannot be evaluated."""


def _values(x):
    return x.v if isinstance(x, Series) else np.asarray(x)


def _crossing_time(s, value):
    v = s.v.astype(float)
    hits = np.flatnonzero(v >= value)
    if hits.size == 0:
        return math.inf
    i = hits[0]
    if i == 0:
        return float(s.t[0])
    f = (value - v[i - 1]) / (v[i] - v[i - 1]) if v[i] != v[i - 1] else 1.0
    return float(s.t[i - 1] + f * (s.t[i] - s.t[i - 1]))


def _rising_edges(cond):
    v = _values(cond).astype(bool)
    return int(np.count_nonzero(v[1:] & ~v[:-1]) + (1 if v.size and v[0] else 0))


def _window(s, t0, t1):
    mask = (s.t >= t0) & (s.t <= t1)
    return Series(s.t[mask], s.v[mask])


def _changes(s):
    v = s.v
    idx = np.flatnonzero(v[1:] != v[:-1]) + 1
    return [float(x) for x in v[idx]]


def _settle_time(s, target, tol=0.05):
    outside = np.flatnonzero(np.abs(s.v - target) > tol * abs(target))
    return float(s.t[outside[-1]]) if outside.size else float(s.t[0])


def spectrum(s):
    """One-sided amplitude spectrum of a uniformly sampled signal: (f [Hz], amplitude).

    Mean removed, Hann window (amplitudes corrected for its gain), so a sine
    of amplitude A reads A at its frequency.
    """
    v = np.asarray(_values(s), dtype=float)
    t = np.asarray(s.t, dtype=float)
    if v.size < 8:
        raise CheckError("spectrum: too few samples")
    dt = float(np.median(np.diff(t)))
    window = np.hanning(v.size)
    amp = 2.0 * np.abs(np.fft.rfft((v - v.mean()) * window)) / window.sum()
    return np.fft.rfftfreq(v.size, dt), amp


def spectral_peak(s, f0=0.0, f1=math.inf):
    """(frequency, amplitude) of the largest spectral peak between f0 and f1 [Hz]."""
    f, amp = spectrum(s)
    band = (f >= f0) & (f <= f1) & (f > 0.0)
    if not np.any(band):
        raise CheckError(f"spectral peak: no frequencies in [{f0}, {f1}] Hz")
    i = int(np.argmax(np.where(band, amp, -1.0)))
    return float(f[i]), float(amp[i])


def _duration(cond):
    v = _values(cond).astype(bool)
    if v.size < 2:
        return 0.0
    dt = np.diff(cond.t)
    return float(np.sum(dt[v[:-1]]))


FUNCTIONS = {
    "max": lambda x: float(np.max(_values(x))),
    "min": lambda x: float(np.min(_values(x))),
    "mean": lambda x: float(np.mean(_values(x))),
    "std": lambda x: float(np.std(_values(x))),
    "rms": lambda x: float(np.sqrt(np.mean(np.square(_values(x).astype(float))))),
    "abs": lambda x: Series(x.t, np.abs(x.v)) if isinstance(x, Series) else abs(x),
    "final": lambda x: float(x.v[-1]),
    "initial": lambda x: float(x.v[0]),
    "at": lambda x, t: float(np.interp(t, x.t, x.v.astype(float))),
    "window": _window,
    "time_to": _crossing_time,
    "rise_time": lambda x, target, lo=0.1, hi=0.9: _crossing_time(x, hi * target) - _crossing_time(x, lo * target),
    "overshoot": lambda x, target: (float(np.max(x.v)) - target) / abs(target),
    "settle_time": _settle_time,
    "count": _rising_edges,
    "duration": _duration,
    "any": lambda c: bool(np.any(_values(c))),
    "all": lambda c: bool(np.all(_values(c))),
    "changes": _changes,
    "peak_freq": lambda x, f0=0.0, f1=math.inf: spectral_peak(x, f0, f1)[0],
    "peak_amp": lambda x, f0=0.0, f1=math.inf: spectral_peak(x, f0, f1)[1],
}

_COMPARE = {
    ast.Lt: lambda a, b: a < b, ast.LtE: lambda a, b: a <= b, ast.Gt: lambda a, b: a > b,
    ast.GtE: lambda a, b: a >= b, ast.Eq: lambda a, b: a == b, ast.NotEq: lambda a, b: a != b,
}
_BINARY = {
    ast.Add: lambda a, b: a + b, ast.Sub: lambda a, b: a - b,
    ast.Mult: lambda a, b: a * b, ast.Div: lambda a, b: a / b,
}


def _dotted_name(node):
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if not isinstance(node, ast.Name):
        raise CheckError("only signal names may contain dots")
    parts.append(node.id)
    return ".".join(reversed(parts))


class Evaluator:
    """Evaluates a check expression against a recording ({"t": ..., name: ...})."""

    def __init__(self, recording):
        self.recording = recording
        self.t = recording["t"]

    def signal(self, name):
        if name == "t":
            return Series(self.t, self.t)
        if name not in self.recording:
            raise CheckError(f"unknown signal '{name}'")
        return Series(self.t, self.recording[name])

    def evaluate(self, expr):
        try:
            tree = ast.parse(expr, mode="eval")
        except SyntaxError as e:
            raise CheckError(f"syntax error: {e.msg}") from None
        return self._eval(tree.body)

    def parts(self, expr):
        """(left, right) values when the expression is a single comparison."""
        tree = ast.parse(expr, mode="eval").body
        if isinstance(tree, ast.Compare) and len(tree.ops) == 1:
            return self._eval(tree.left), self._eval(tree.comparators[0])
        return None

    def _eval(self, node):
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float, bool)):
            return node.value
        if isinstance(node, ast.Name):
            if node.id in ("inf", "nan"):
                return float(node.id)
            return self.signal(node.id)
        if isinstance(node, ast.Attribute):
            return self.signal(_dotted_name(node))
        if isinstance(node, ast.UnaryOp):
            value = self._eval(node.operand)
            if isinstance(node.op, ast.USub):
                return -value
            if isinstance(node.op, ast.Not):
                return not _truth(value)
        if isinstance(node, ast.BinOp) and type(node.op) in _BINARY:
            return _BINARY[type(node.op)](self._eval(node.left), self._eval(node.right))
        if isinstance(node, ast.BoolOp):
            values = [_truth(self._eval(v)) for v in node.values]
            return all(values) if isinstance(node.op, ast.And) else any(values)
        if isinstance(node, ast.Compare):
            left = self._eval(node.left)
            result = True
            for op, comparator in zip(node.ops, node.comparators):
                if type(op) not in _COMPARE:
                    raise CheckError("unsupported comparison")
                right = self._eval(comparator)
                outcome = _COMPARE[type(op)](left, right)
                result = result and outcome if not isinstance(outcome, Series) else outcome
                left = right
            return result
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in FUNCTIONS:
            if node.keywords:
                raise CheckError("keyword arguments are not supported")
            args = [self._eval(a) for a in node.args]
            try:
                return FUNCTIONS[node.func.id](*args)
            except (TypeError, ValueError, IndexError) as e:
                raise CheckError(f"{node.func.id}(): {e}") from None
        if isinstance(node, ast.Subscript) and isinstance(node.slice, ast.Constant):
            value = self._eval(node.value)
            if isinstance(value, list) and isinstance(node.slice.value, int):
                try:
                    return value[node.slice.value]
                except IndexError:
                    raise CheckError(f"index {node.slice.value} out of range ({len(value)} values)") from None
        raise CheckError(f"unsupported expression: {ast.unparse(node)}")


def _truth(value):
    if isinstance(value, Series):
        return bool(np.all(value.v))
    return bool(value)


def _fmt(value):
    if isinstance(value, Series):
        return f"<signal, {value.v.size} samples>"
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


@dataclass
class CheckResult:
    name: str
    expr: str
    passed: bool
    detail: str


@dataclass
class RunResult:
    seed: object  # int, or None for the scenario's own seed
    checks: list
    sim_time: float
    wall_time: float
    error: str = ""

    @property
    def passed(self):
        return not self.error and all(c.passed for c in self.checks)


@dataclass
class TestResult:
    name: str
    file: str
    description: str
    runs: list = field(default_factory=list)
    error: str = ""
    skipped: str = ""  # why it did not run (a requirement missing); not a failure

    @property
    def passed(self):
        return bool(self.skipped) or (not self.error and bool(self.runs) and all(r.passed for r in self.runs))

    @property
    def status(self):
        return "SKIP" if self.skipped else ("PASS" if self.passed else "FAIL")


# What a test can require (`requires: [ngspice]`): name -> check returning "" or why not.
def _ngspice_missing():
    from . import ngspice_available
    return "" if ngspice_available() else "ngspice shared library not found"


REQUIREMENTS = {"ngspice": _ngspice_missing}


def load_test(path):
    path = Path(path)
    spec = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    for key in ("scenario", "checks"):
        if key not in spec:
            raise ValueError(f"{path}: missing '{key}'")
    for check in spec["checks"]:
        if "expr" not in check:
            raise ValueError(f"{path}: every check needs an 'expr'")
    return spec


def run_test(path, *, log_dir=None):
    path = Path(path)
    try:
        spec = load_test(path)
    except (OSError, ValueError, yaml.YAMLError) as e:
        return TestResult(path.stem, str(path), "", error=str(e))
    result = TestResult(spec.get("test", path.name.removesuffix(".test.yaml")), str(path),
                        spec.get("description", ""))
    for need in spec.get("requires") or []:
        check = REQUIREMENTS.get(str(need))
        why = check() if check else f"unknown requirement '{need}'"
        if why:
            result.skipped = why
            return result
    scenario = (path.parent / spec["scenario"]).resolve()
    seeds = spec.get("seeds") or [None]
    for seed in seeds:
        overrides = {str(k): v for k, v in (spec.get("overrides") or {}).items()}
        overrides.setdefault("logging.signals", "*")
        if seed is not None:
            overrides["seed"] = int(seed)
        started = clock.perf_counter()
        try:
            with Simulation(str(scenario), overrides=overrides, log_dir=log_dir) as sim:
                sim.run(spec.get("duration_s"))
                recording = sim.recording()
                sim_time = sim.time
        except Exception as e:  # scenario errors are test failures, not crashes
            result.runs.append(RunResult(seed, [], 0.0, 0.0, error=str(e)))
            continue
        wall = clock.perf_counter() - started
        evaluator = Evaluator(recording)
        checks = []
        for check in spec["checks"]:
            expr = str(check["expr"])
            name = check.get("name", expr)
            try:
                passed = _truth(evaluator.evaluate(expr))
                parts = evaluator.parts(expr)
                detail = f"{_fmt(parts[0])} vs {_fmt(parts[1])}" if parts else ""
            except CheckError as e:
                passed, detail = False, f"error: {e}"
            checks.append(CheckResult(name, expr, passed, detail))
        result.runs.append(RunResult(seed, checks, sim_time, wall))
    return result


def discover(paths):
    files = []
    for p in map(Path, paths):
        if p.is_dir():
            files += sorted(p.rglob("*.test.yaml"))
        else:
            files.append(p)
    return files


def markdown_report(results):
    passed = sum(r.passed for r in results)
    lines = ["# vsim test report", "", f"**{passed} / {len(results)} tests passed**", "",
             "| Test | Result | Seeds | Sim time | Wall time |", "|---|---|---|---|---|"]
    for r in results:
        sim_time = sum(run.sim_time for run in r.runs)
        wall = sum(run.wall_time for run in r.runs)
        lines.append(f"| {r.name} | {r.status} | {len(r.runs)} | {sim_time:.1f} s | {wall:.2f} s |")
    for r in results:
        lines += ["", f"## {r.name} — {r.status}", ""]
        if r.description:
            lines += [r.description, ""]
        lines.append(f"`{r.file}`")
        if r.skipped:
            lines += ["", f"Skipped: {r.skipped}"]
            continue
        if r.error:
            lines += ["", f"Error: {r.error}"]
            continue
        for run in r.runs:
            lines += ["", f"Seed {run.seed}:" if run.seed is not None else "Scenario seed:", ""]
            if run.error:
                lines.append(f"Run failed: {run.error}")
                continue
            lines += ["| Check | Result | Values | Expression |", "|---|---|---|---|"]
            for c in run.checks:
                lines.append(f"| {c.name} | {'PASS' if c.passed else 'FAIL'} | {c.detail} | `{c.expr}` |")
    return "\n".join(lines) + "\n"


def json_report(results):
    return json.dumps([{
        "test": r.name, "file": r.file, "passed": r.passed, "skipped": r.skipped, "error": r.error,
        "runs": [{"seed": run.seed, "passed": run.passed, "error": run.error, "sim_time_s": run.sim_time,
                  "wall_time_s": run.wall_time,
                  "checks": [{"name": c.name, "expr": c.expr, "passed": c.passed, "detail": c.detail}
                             for c in run.checks]} for run in r.runs]} for r in results], indent=2,
        ensure_ascii=False)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m vsim.testing", description="Run vsim test files.")
    parser.add_argument("paths", nargs="+", help="*.test.yaml files or directories")
    parser.add_argument("--report", help="write a Markdown report here")
    parser.add_argument("--json", help="write a JSON report here")
    parser.add_argument("-k", dest="keyword", help="only test files whose name contains this")
    parser.add_argument("--log-dir", help="also write run outputs (MCAP logs) here")
    args = parser.parse_args(argv)
    for stream in (sys.stdout, sys.stderr):
        # Check names may be Korean; a cp1252 console must not crash the run.
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")

    missing = [p for p in args.paths if not Path(p).exists()]
    if missing:
        print(f"not found: {', '.join(missing)}", file=sys.stderr)
        return 2
    files = discover(args.paths)
    if not files:
        print("no *.test.yaml files found", file=sys.stderr)
        return 2
    if args.keyword:
        files = [f for f in files if args.keyword in f.name]
    results = []
    for f in files:
        result = run_test(f, log_dir=args.log_dir)
        results.append(result)
        print(f"{result.status}  {result.name}" + (f" ({result.skipped})" if result.skipped else ""))
        if result.error:
            print(f"      {result.error}")
        for run in result.runs:
            if run.error:
                print(f"      seed {run.seed if run.seed is not None else '-'}: {run.error}")
            for c in run.checks:
                if not c.passed:
                    print(f"      seed {run.seed if run.seed is not None else '-'}: {c.name}: {c.detail or c.expr}")
    passed = sum(r.passed for r in results)
    print(f"\n{passed} / {len(results)} tests passed")
    if args.report:
        Path(args.report).write_text(markdown_report(results), encoding="utf-8")
    if args.json:
        Path(args.json).write_text(json_report(results), encoding="utf-8")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
