"""Python interface to the vsim simulation core.

    import vsim
    rec = vsim.run_scenario("templates/projects/example_fs_ev/scenarios/sine_steer.yaml")
    rec["veh.chassis.vx"]   # numpy array at the logging rate, rec["t"] holds the times
"""

from ._vsim import Simulation, Track, __version__, can_frame_bits, git_commit

__all__ = ["Simulation", "Track", "can_frame_bits", "run_scenario", "__version__", "git_commit"]


def run_scenario(scenario, *, duration=None, overrides=None, log_dir=None):
    """Runs a scenario headless and returns its recording.

    Runs for `duration` seconds, or until the scenario's time.t_end.
    """
    with Simulation(str(scenario), overrides=overrides or {}, log_dir=log_dir) as sim:
        sim.run(duration)
        return sim.recording()
