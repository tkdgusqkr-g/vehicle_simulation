"""Electrical network (design doc §12.3): the example car's operating point,
the run summary, and the electrical events the phenomenon report shows."""

import json

import vsim

ON = {"electrical.enabled": "true"}


def test_operating_point_of_the_example_car(scenario):
    op = vsim.electrical_operating_point(scenario("acceleration"), ON)
    assert op["converged"]
    nodes, signals = op["nodes"], op["signals"]
    pack = nodes["ACC_F"] - nodes["ACC-"]
    assert 560 < pack < 590  # 140 cells at 90 % charge
    assert abs((nodes["INV_DC+"] - nodes["INV_DC-"]) - pack) < 0.5  # precharged, AIRs closed
    assert abs(nodes["GLV"] - 13.5) < 0.05  # DC-DC
    assert signals["elec.PCC.state"] == 3
    assert signals["elec.AIR_P.closed"] == 1 and signals["elec.PRE_K.closed"] == 0
    assert signals["elec.VCU.on"] == 1
    # Pedal released: APPS1 at 0.5 V, APPS2 at 4.5 V at the VCU's ADC (through the
    # wiring; the 100 kOhm pull-down loads the wiper by about 0.5 %).
    assert abs(signals["elec.sensor.apps.ch1"] - 0.5) < 0.03
    assert abs(signals["elec.sensor.apps.ch2"] - 4.5) < 0.03


def test_operating_point_follows_the_scenario(scenario):
    op = vsim.electrical_operating_point(scenario("acceleration"), {**ON, "initial.lv_soc": 0.05,
                                                                    "electrical.faults.0.type": "open",
                                                                    "electrical.faults.0.target": "DCDC"})
    assert op["nodes"]["GLV"] < 12.5  # the LV battery alone, nearly empty
    assert op["signals"]["elec.DCDC.on"] == 0


def test_summary_in_the_run_folder(scenario, tmp_path):
    with vsim.Simulation(scenario("acceleration"), overrides=ON, log_dir=str(tmp_path)) as sim:
        sim.run(2.0)
    run_dir = next(tmp_path.iterdir())
    elec = json.loads((run_dir / "summary.json").read_text())["electrical"]
    assert elec["level"] == "E2" and elec["solver"]["not_converged"] == 0
    hv = elec["wires"]["HV_P"]
    assert hv["i_max_a"] > 100 and hv["utilization"] > 1  # bursts above the continuous rating
    assert hv["temp_max_c"] < 30  # but far too short to heat a 16 mm2 cable
    assert elec["dcdc"]["DCDC"]["margin"] > 0
    assert elec["ecus"]["VCU"]["resets"] == 0
    assert elec["lv_budget"]["p_mean_w"] > 50


def test_phenomena_see_electrical_trouble(scenario):
    from vsim import phenomena
    rec = phenomena.run(scenario("acceleration"), duration=4.5,
                        overrides={**ON, "electrical.initial": "off",
                                   "electrical.faults.0.type": "open", "electrical.faults.0.target": "PRE_R"})
    fail = [e for e in phenomena.events(rec) if e.kind == "precharge_fail"]
    assert fail and 4.0 <= fail[0].t < 4.1  # the controller's 4 s timeout
    rec = phenomena.run(scenario("acceleration"), duration=2.0,
                        overrides={**ON, "electrical.faults.0.type": "short", "electrical.faults.0.target": "COOL",
                                   "electrical.faults.0.r_ohm": "0.02", "electrical.faults.0.at_s": "1.0"})
    events = phenomena.events(rec)
    assert any(e.kind == "fuse_blown" and "F_COOL" in e.detail and 1.0 <= e.t < 1.2 for e in events)
