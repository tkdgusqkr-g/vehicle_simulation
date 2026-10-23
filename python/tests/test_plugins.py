"""Python plugins (design doc §11): the same law as the C++ and FMU examples,
errors that point at the plugin, and the run record."""

import json
import textwrap

import numpy as np
import pytest

import vsim
from vsim.plugin import Block


def run(scenario, overrides, duration=4.0, **kw):
    return vsim.run_scenario(scenario, duration=duration, overrides={"logging.signals": "*", **overrides}, **kw)


def test_python_and_native_plugins_give_the_same_run(scenario):
    py = run(scenario("skidpad_auto"), {"components.tv_py.enabled": "true"})
    native = run(scenario("skidpad_auto"), {"components.tv_native.enabled": "true"})
    assert np.array_equal(py["veh.chassis.yaw_rate"], native["veh.chassis.yaw_rate"])
    assert np.max(np.abs(py["tv.shift"])) > 0.1


def write_plugin(tmp_path, body):
    file = tmp_path / "my_plugin.py"
    file.write_text(textwrap.dedent(body))
    return str(file)


def test_plugin_from_a_file_with_unit_conversion(scenario, tmp_path):
    file = write_plugin(tmp_path, """
        from vsim.plugin import Block

        class Speedo(Block):
            period = 0.05
            inputs = {"v": "km/h"}
            outputs = ["v_out"]

            def init(self, params):
                self.offset = params["offset"]

            def step(self, t, u):
                return [u["v"] + self.offset]
    """)
    rec = run(scenario("acceleration"), {"components.speedo.type": "python",
                                         "components.speedo.module": f"{file}:Speedo",
                                         "components.speedo.params.offset": "1",
                                         "components.speedo.ports.in.v": "veh.chassis.vx"}, duration=2.0)
    t, v, out = rec["t"], rec["veh.chassis.vx"], rec["plugin.speedo.v_out"]
    i = np.searchsorted(t, 1.5)                   # the plugin runs at 1.5 s (every 0.05 s) ...
    assert out[i + 1] == pytest.approx(v[i] * 3.6 + 1.0, rel=1e-12)  # ... and its output is logged after the step
    assert out[i + 40] == out[i + 1]              # held until the next run (1 ms logging)


def test_plugin_errors_name_the_plugin(scenario, tmp_path):
    file = write_plugin(tmp_path, """
        from vsim.plugin import Block

        class Broken(Block):
            inputs = ["veh.chassis.vx"]
            outputs = {"y": "-"}

            def step(self, t, u):
                if t > 0.5:
                    raise ValueError("sensor table out of range")
                return {"y": 1.0}
    """)
    with pytest.raises(RuntimeError, match="broken.*sensor table out of range"):
        run(scenario("acceleration"), {"components.broken.type": "python",
                                       "components.broken.module": f"{file}:Broken"}, duration=1.0)
    with pytest.raises(Exception, match="has no 'Nope'"):
        run(scenario("acceleration"), {"components.broken.type": "python",
                                       "components.broken.module": f"{file}:Nope"}, duration=0.1)


def test_plugins_are_recorded_with_the_run(scenario, tmp_path):
    with vsim.Simulation(scenario("skidpad_auto"), overrides={"components.tv_py.enabled": "true"},
                         log_dir=str(tmp_path)) as sim:
        sim.run(0.5)
    run_dir = next(tmp_path.iterdir())
    meta = json.loads((run_dir / "meta.json").read_text())
    p = meta["plugins"]["tv_py"]
    assert p["type"] == "python" and p["file"].endswith("tv_py.py") and len(p["sha256"]) == 64
    assert p["name"] == "TorqueVectoring" and p["version"] == "1.0"
    summary = json.loads((run_dir / "summary.json").read_text())
    assert summary["plugins"]["tv_py"]["calls"] == 50


def test_block_base_class():
    b = Block()
    assert b.period == 0.01 and b.init({}) is None
    with pytest.raises(NotImplementedError):
        b.step(0.0, {})
