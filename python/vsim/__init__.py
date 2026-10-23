"""Python interface to the vsim simulation core.

    import vsim
    rec = vsim.run_scenario("templates/projects/example_fs_ev/scenarios/sine_steer.yaml")
    rec["veh.chassis.vx"]   # numpy array at the logging rate, rec["t"] holds the times
"""

import json as _json

from ._vsim import Simulation, Track, __version__, _electrical_operating_point, can_frame_bits, git_commit

__all__ = ["Simulation", "Track", "can_frame_bits", "electrical_operating_point", "run_scenario", "__version__",
           "git_commit"]


def run_scenario(scenario, *, duration=None, overrides=None, log_dir=None):
    """Runs a scenario headless and returns its recording.

    Runs for `duration` seconds, or until the scenario's time.t_end.
    """
    with Simulation(str(scenario), overrides=overrides or {}, log_dir=log_dir) as sim:
        sim.run(duration)
        return sim.recording()


def electrical_operating_point(scenario, overrides=None):
    """DC operating point of a scenario's electrical network (design doc §12.3).

    Returns {"nodes": {net: volts}, "signals": {"elec.<id>.<quantity>": value},
    "converged": bool, "summary": {...}} with the car at rest (no motor power,
    pedals released), contactors and ECUs where the supplies put them.
    """
    overrides = {str(k): str(v) for k, v in (overrides or {}).items()}
    return _json.loads(_electrical_operating_point(str(scenario), overrides))
