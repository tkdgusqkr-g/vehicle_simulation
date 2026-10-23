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


CAN_ECU = r'''
import struct
from vsim.external import Ecu, Frame

ecu = Ecu("can_vcu", inputs={"pedal": "-"}, outputs={"rx_inverter": "-"})
state = {"rx": 0, "k": 0}

def step(t, inputs, frames):
    state["rx"] += sum(1 for f in frames if f.id == 512)       # INV_FL_Status
    state["k"] += 1
    out = []
    if state["k"] % 10 == 1:                                    # VCU_TorqueCmd every 10 ms
        raw = int(round(21.0 * inputs["pedal"] / 0.01))         # 0.01 N m per bit, int16 x 4
        out.append(Frame(bus=0, id=256, data=struct.pack("<4h", raw, raw, raw, raw)))
    return {"rx_inverter": float(state["rx"])}, out

ecu.run(step)
'''


def test_external_vcu_on_the_can_bus(scenario, tmp_path, monkeypatch):
    """The VCU is firmware on the virtual bus: torque reaches the inverters only as its frames."""
    monkeypatch.setenv("PYTHONPATH", str(REPO / "python") + os.pathsep + os.environ.get("PYTHONPATH", ""))
    script = tmp_path / "can_vcu.py"
    script.write_text(CAN_ECU)
    import json
    component = json.dumps({"type": "external", "slot": "vcu", "period": 0.001, "can": ["can_main"],
                            "command": [sys.executable, str(script)],
                            "ports": {"in": {"pedal": "drv.cmd.throttle"}, "out": {"rx_inverter": "ext.rx_inverter"}}})
    overrides = {"can.enabled": True, "can.fidelity": "timing", "can.nodes.0.external": "vcu",
                 "components.vcu": component, "logging.signals": "*"}
    with vsim.Simulation(scenario("acceleration"), overrides=overrides) as sim:
        sim.run(1.0)
        rec = sim.recording()
        summary = json.loads(sim.can_summary()) if hasattr(sim, "can_summary") else {}
    assert np.asarray(rec["can.VCU_TorqueCmd.TorqueRL"]).max() == pytest.approx(21.0)
    assert np.asarray(rec["pt.motor.rl.torque_cmd"]).max() == pytest.approx(21.0)
    assert np.asarray(rec["veh.chassis.speed"])[-1] > 8.0
    assert np.asarray(rec["ext.rx_inverter"])[-1] > 50       # INV_FL_Status every 10 ms (and the car's others)
    if summary:
        assert summary["outside"]["external"] == ["vcu"]
