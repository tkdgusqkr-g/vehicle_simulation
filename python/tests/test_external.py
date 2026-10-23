import os
import pathlib
import sys

import numpy as np
import pytest

import vsim

REPO = pathlib.Path(__file__).resolve().parents[2]


def external_vcu(**extra):
    spec = {"type": "external", "slot": "vcu", "period": 0.001,
            "command": [sys.executable, "ecus/simple_vcu.py"],
            "ports": {"in": {"pedal": "drv.cmd.throttle"},
                      "out": {f"tq_{c}": f"pt.motor.{c}.torque_cmd" for c in ("fl", "fr", "rl", "rr")}}}
    spec.update(extra)
    import json
    return json.dumps(spec)  # YAML flow accepts JSON


def test_external_vcu_drives_the_car(scenario, monkeypatch):
    monkeypatch.setenv("PYTHONPATH", str(REPO / "python") + os.pathsep + os.environ.get("PYTHONPATH", ""))
    with vsim.Simulation(scenario("acceleration"), overrides={"components.vcu": external_vcu(),
                                                              "logging.signals": "*"}) as sim:
        sim.run(2.0)
        rec = sim.recording()
        plugins = sim.plugins() if hasattr(sim, "plugins") else None
    torque = np.asarray(rec["pt.motor.rl.torque_cmd"])
    assert torque.max() == pytest.approx(21.0)  # full pedal from the start
    assert np.asarray(rec["veh.chassis.speed"])[-1] > 15.0  # 2 s of full power


def test_external_program_that_never_connects_fails_cleanly(scenario):
    spec = external_vcu(command=[sys.executable, "-c", "import time; time.sleep(0.2)"], connect_timeout_s=5)
    with pytest.raises(Exception, match="exited|connect"):
        vsim.Simulation(scenario("acceleration"), overrides={"components.vcu": spec})
