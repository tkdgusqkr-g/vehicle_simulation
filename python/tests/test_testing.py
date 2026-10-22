"""The test runner: expression evaluation, safety, and running test files."""

import math
import textwrap

import numpy as np
import pytest

from vsim import testing
from vsim.testing import CheckError, Evaluator


@pytest.fixture
def recording():
    t = np.linspace(0.0, 10.0, 1001)
    step = np.where(t >= 1.0, 1.0 - np.exp(-(t - 1.0) / 0.5), 0.0)  # first-order step at t = 1
    laps = np.where(t < 4, 0.0, np.where(t < 8, 3.5, 3.2))
    return {"t": t, "veh.speed": 2.0 * t, "resp.y": step, "lap.last": laps,
            "flag.on": ((t > 2) & (t < 3)) | ((t > 5) & (t < 5.5))}


def value(recording, expr):
    return Evaluator(recording).evaluate(expr)


def test_statistics_and_time_functions(recording):
    assert value(recording, "final(veh.speed)") == pytest.approx(20.0)
    assert value(recording, "max(window(veh.speed, 2, 4))") == pytest.approx(8.0)
    assert value(recording, "at(veh.speed, 2.5)") == pytest.approx(5.0)
    assert value(recording, "time_to(veh.speed, 7)") == pytest.approx(3.5)
    assert value(recording, "time_to(veh.speed, 100)") == math.inf
    rise = 0.5 * (math.log(0.9) - math.log(0.1))  # 10 -> 90 % of a first-order step
    assert value(recording, "rise_time(resp.y, 1)") == pytest.approx(rise, abs=0.02)
    assert value(recording, "overshoot(resp.y, 1)") < 0.01
    assert value(recording, "settle_time(resp.y, 1, 0.05)") == pytest.approx(1 + 0.5 * math.log(20), abs=0.02)
    assert value(recording, "count(flag.on)") == 2
    assert value(recording, "duration(flag.on)") == pytest.approx(1.5, abs=0.03)
    assert value(recording, "changes(lap.last)") == [3.5, 3.2]
    assert value(recording, "changes(lap.last)[1] < 3.3") is True
    assert value(recording, "count(veh.speed > 10)") == 1
    assert value(recording, "abs(-2) + 1") == 3


def test_comparisons_report_both_sides(recording):
    evaluator = Evaluator(recording)
    assert evaluator.parts("final(veh.speed) < 30") == (pytest.approx(20.0), 30)
    assert evaluator.evaluate("final(veh.speed) < 30 and not final(veh.speed) > 25")


@pytest.mark.parametrize("expr", [
    "__import__('os').system('true')",
    "open('x')",
    "veh.speed.__class__",
    "(lambda: 1)()",
    "max(x for x in [1])",
    "final(veh.speed)[0]",
    "nope.signal + 1",
    "max(veh.speed, key=1)",
])
def test_rejects_anything_else(recording, expr):
    with pytest.raises(CheckError):
        value(recording, expr)


def write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(textwrap.dedent(text), encoding="utf-8")
    return path


def test_runs_a_test_file(tmp_path, scenario):
    path = write(tmp_path, "accel.test.yaml", f"""\
        test: accel
        scenario: {scenario('straight_accel')}
        duration_s: 2
        seeds: [1, 2]
        checks:
          - name: moving
            expr: "final(veh.chassis.vx) > 5"
          - name: impossible
            expr: "final(veh.chassis.vx) > 500"
          - name: broken
            expr: "final(no.such.signal) > 1"
        """)
    result = testing.run_test(path)
    assert not result.passed
    assert [run.seed for run in result.runs] == [1, 2]
    checks = result.runs[0].checks
    assert [c.passed for c in checks] == [True, False, False]
    assert "unknown signal" in checks[2].detail
    report = testing.markdown_report([result])
    assert "FAIL" in report and "moving" in report


def test_main_exit_codes_and_reports(tmp_path, scenario):
    write(tmp_path, "ok.test.yaml", f"""\
        scenario: {scenario('straight_accel')}
        duration_s: 1
        checks:
          - expr: "final(drv.cmd.throttle) == 1"
        """)
    report = tmp_path / "report.md"
    data = tmp_path / "report.json"
    assert testing.main([str(tmp_path), "--report", str(report), "--json", str(data)]) == 0
    assert "1 / 1 tests passed" in report.read_text(encoding="utf-8")
    write(tmp_path, "bad.test.yaml", "scenario: nowhere.yaml\nchecks: [{expr: '1 > 0'}]\n")
    assert testing.main([str(tmp_path)]) == 1
    assert testing.main([str(tmp_path / "empty")]) == 2
