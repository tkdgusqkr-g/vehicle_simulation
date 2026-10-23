"""Phenomenon detection, spectra and cause isolation (design doc §10.6)."""

import numpy as np
import pytest

from vsim import phenomena
from vsim.testing import Evaluator, Series, spectral_peak


def test_spectral_peak_reads_frequency_and_amplitude():
    t = np.arange(0.0, 4.0, 0.002)
    v = 2.0 * np.sin(2 * np.pi * 7.0 * t) + 0.3 * np.sin(2 * np.pi * 40.0 * t) + 5.0
    freq, amp = spectral_peak(Series(t, v), 1.0, 100.0)
    assert freq == pytest.approx(7.0, abs=0.3)
    assert amp == pytest.approx(2.0, rel=0.05)
    rec = {"t": t, "imu.az": v}
    assert Evaluator(rec).evaluate("peak_freq(imu.az, 20, 100)") == pytest.approx(40.0, abs=0.3)
    assert Evaluator(rec).evaluate("peak_amp(imu.az, 20, 100)") == pytest.approx(0.3, rel=0.05)


def test_events_find_a_sensor_fault_and_its_torque_cut(scenario):
    rec = phenomena.run(scenario("acceleration"), duration=1.5,
                        overrides={"vehicle.sensors.5.faults.0.channel": "ch2",
                                   "vehicle.sensors.5.faults.0.type": "open",
                                   "vehicle.sensors.5.faults.0.at_s": "1.0"})
    found = phenomena.events(rec)
    cuts = [e for e in found if e.kind == "apps_implausible"]
    assert len(cuts) == 1
    assert cuts[0].t == pytest.approx(1.1, abs=0.002)
    assert any(e.kind == "traction_control" for e in found)  # 4WD launch
    assert all(e.t > 0.0 for e in found)


def test_vibration_aliasing_shows_as_a_peak(scenario):
    # PH-11: motor-order vibration (about 1 kHz at 40 km/h) sampled at 500 Hz
    # folds down to a low line unless the IMU's anti-alias filter is on.
    overrides = {"driver.steer.type": "const", "driver.steer.value": "0", "logging.rate_hz": "500"}

    def lines(anti_alias_hz):
        rec = phenomena.run(scenario("sine_steer"), realism={"sensors": "real"}, duration=8.0,
                            overrides={**overrides, "vehicle.sensors.0.anti_alias_hz": anti_alias_hz})
        return {name: f for name, f, _ in phenomena.peaks(rec, ["sens.imu.az"], start=4.0)}

    assert 5.0 < lines("0")["sens.imu.az"] < 40.0
    assert "sens.imu.az" not in lines("100")


def test_isolation_attributes_deviation_to_its_category(scenario):
    table = phenomena.isolate(scenario("acceleration"), ["sens.imu.ax", "vcu.vx_est"], duration=1.0)
    assert list(table) == ["all"] + phenomena.CATEGORIES
    assert table["sensors"]["sens.imu.ax"] > 0.0
    assert table["variation"]["sens.imu.ax"] > 0.0
    assert table["all"]["vcu.vx_est"] > 0.0
    without = phenomena.isolate(scenario("acceleration"), ["sens.imu.ax"], duration=0.5,
                                categories=["environment", "sensors"], mode="without")
    assert list(without) == ["all", "environment", "sensors"]
    assert without["sensors"]["sens.imu.ax"] < without["all"]["sens.imu.ax"]  # less noise without the sensor errors
    with pytest.raises(KeyError):
        phenomena.isolate(scenario("acceleration"), ["no.such.signal"], duration=0.1)
