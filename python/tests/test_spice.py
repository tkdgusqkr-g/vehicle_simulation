import re
import shutil
import subprocess

import pytest

import vsim


def ngspice_op(netlist, tmp_path):
    """Node voltages of a netlist's operating point from the ngspice program."""
    deck = netlist.replace("\n.op\n", "\n.control\nop\nprint all\n.endc\n")
    path = tmp_path / "net.cir"
    path.write_text(deck)
    out = subprocess.run(["ngspice", "-b", str(path)], capture_output=True, text=True, timeout=60).stdout
    return {m.group(1): float(m.group(2)) for m in re.finditer(r"^(n_\w+) = (\S+)$", out, re.M)}


def net(name):
    return ("n_" + "".join(c if c.isalnum() else ("_P" if c == "+" else "_N" if c == "-" else "_")
                           for c in name)).lower()


def test_export_lists_every_part(scenario):
    text = vsim.spice_netlist(scenario("acceleration"), {"electrical.enabled": True})
    assert text.startswith("* acceleration\n")
    assert "\n.op\n.end\n" in text
    assert "not exported" not in text
    # Contacts as their state: AIRs closed when active, precharge relay open.
    assert re.search(r"^RAIR_P_\d+ \S+ \S+ 0\.0003 ; contact closed$", text, re.M)
    assert re.search(r"^RPRE_K_\d+ .* ; contact open$", text, re.M)
    assert "n_ACC_P" in text and "n_ACC_N" in text  # distinct nets for ACC+ and ACC-


@pytest.mark.skipif(shutil.which("ngspice") is None, reason="ngspice not installed")
def test_ngspice_agrees_with_the_simulator(scenario, tmp_path):
    overrides = {"electrical.enabled": True}
    ours = vsim.electrical_operating_point(scenario("acceleration"), overrides)["nodes"]
    theirs = ngspice_op(vsim.spice_netlist(scenario("acceleration"), overrides), tmp_path)
    assert len(theirs) >= len(ours)
    # The HV side floats against the chassis: compare it against ACC-.
    hv = ("ACC", "TS", "INV_DC", "PRE_MID", "DIS_MID")
    for name, v in ours.items():
        ref_ours = ours["ACC-"] if name.startswith(hv) else 0.0
        ref_theirs = theirs[net("ACC-")] if name.startswith(hv) else 0.0
        assert theirs[net(name)] - ref_theirs == pytest.approx(v - ref_ours, abs=0.1), name
