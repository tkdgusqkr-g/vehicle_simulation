"""V-08: CAN frame lengths against an independent implementation of the
classic frame (bit stream, CRC-15, bit stuffing), and the run summary."""

import json
import random

import vsim


def reference_bits(can_id, data, extended=False):
    bits = [0]                                               # SOF
    if extended:
        bits += [(can_id >> (28 - i)) & 1 for i in range(11)]
        bits += [1, 1]                                       # SRR, IDE
        bits += [(can_id >> (17 - i)) & 1 for i in range(18)]
        bits += [0, 0, 0]                                    # RTR, r1, r0
    else:
        bits += [(can_id >> (10 - i)) & 1 for i in range(11)]
        bits += [0, 0, 0]                                    # RTR, IDE, r0
    bits += [(len(data) >> (3 - i)) & 1 for i in range(4)]
    for byte in data:
        bits += [(byte >> (7 - i)) & 1 for i in range(8)]
    crc = 0
    for b in bits:
        nxt = b ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if nxt:
            crc ^= 0x4599
    bits += [(crc >> (14 - i)) & 1 for i in range(15)]
    stuffed, run, last = 0, 0, None
    for b in bits:
        if b == last:
            run += 1
        else:
            run, last = 1, b
        if run == 5:
            stuffed += 1
            last, run = 1 - b, 1
    return len(bits) + stuffed + 13


def test_frame_bits_match_an_independent_implementation():
    rng = random.Random(8)
    for _ in range(500):
        extended = rng.random() < 0.3
        can_id = rng.getrandbits(29 if extended else 11)
        data = [rng.choice([0, 0xFF, rng.getrandbits(8)]) for _ in range(rng.randint(0, 8))]
        assert vsim.can_frame_bits(can_id, data, extended)["nominal"] == reference_bits(can_id, data, extended)


def test_can_summary_in_the_run_folder(scenario, tmp_path):
    with vsim.Simulation(scenario("acceleration"), overrides={"can.enabled": "true", "can.fidelity": "timing"},
                         log_dir=str(tmp_path)) as sim:
        sim.run(1.0)
    run_dir = next(tmp_path.iterdir())
    can = json.loads((run_dir / "summary.json").read_text())["can"]
    assert can["fidelity"] == "timing"
    assert 0.05 < can["buses"]["can_main"]["load_mean"] < 0.15
    torque = can["messages"]["VCU_TorqueCmd"]
    assert torque["delivered"] >= 99 and 0.1 < torque["latency_ms"]["mean"] < 1.0
    assert (run_dir / "can_main.asc").exists()


def test_phenomena_see_can_trouble(scenario):
    from vsim import phenomena
    rec = phenomena.run(scenario("acceleration"), duration=2.0,
                        overrides={"can.enabled": "true", "can.fidelity": "faults",
                                   "can.faults.0.type": "bus_off", "can.faults.0.node": "INV_FL",
                                   "can.faults.0.at_s": "1.0", "can.faults.0.until_s": "1.3"})
    kinds = {(e.kind, e.detail.split(" ")[0]) for e in phenomena.events(rec)}
    assert ("can_bus_off", "node") in kinds
    found = [e for e in phenomena.events(rec) if e.kind == "can_timeout"]
    assert found and found[0].detail.startswith("INV_FL_Status") and 1.02 < found[0].t < 1.05
