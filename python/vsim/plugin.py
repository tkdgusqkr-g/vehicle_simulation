"""Base class for Python plugins (design doc §11.6).

A plugin is a class in the vehicle project (plugins/*.py) connected by the
project's components file:

    tv_py:
      type: python
      module: tv_py:TorqueVectoring      # plugins/tv_py.py, class TorqueVectoring
      after: vcu
      params: {gain: 40}
      ports:
        in: {yaw_rate: sens.imu.gz}
        out: {mz: tv.mz}

    from vsim.plugin import Block

    class TorqueVectoring(Block):
        period = 0.01
        inputs = {"yaw_rate": "rad/s"}          # name -> unit (or a list of names)
        outputs = {"mz": "N*m"}

        def init(self, params):
            self.gain = params.get("gain", 1.0)

        def step(self, t, u):
            return {"mz": -self.gain * u["yaw_rate"]}

vsim converts units between ports and the signals they connect to. step()
may return a dict or a sequence in output order. Use `t` (simulated time),
never the wall clock, and seed any randomness from params, so runs repeat.
Python plugins run at up to a few hundred Hz; faster loops belong in C/C++.
"""


class Block:
    period = 0.01   # [s] when the project gives none
    inputs = {}
    outputs = {}
    version = ""

    def init(self, params):
        """Called once with the component's params (a dict)."""

    def step(self, t, u):
        """One period: u maps input names to values; returns the outputs."""
        raise NotImplementedError
