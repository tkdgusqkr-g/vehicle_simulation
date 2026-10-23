"""Torque vectoring as a Python plugin: the same law as sdk/examples/tv_cpp
(C++) and tv_fmu (FMU), so the three give the same run (plugins/components.yaml)."""

from vsim.plugin import Block


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class TorqueVectoring(Block):
    period = 0.01
    version = "1.0"
    inputs = {"steer": "rad", "vx": "m/s", "yaw_rate": "rad/s",
              "tq_fl_in": "N*m", "tq_fr_in": "N*m", "tq_rl_in": "N*m", "tq_rr_in": "N*m"}
    outputs = {"tq_fl": "N*m", "tq_fr": "N*m", "tq_rl": "N*m", "tq_rr": "N*m", "shift": "N*m"}

    def init(self, params):
        self.gain = float(params.get("gain", 40.0))
        self.wheelbase = float(params.get("wheelbase", 1.53))
        self.steer_ratio = float(params.get("steer_ratio", 4.5))
        self.understeer = float(params.get("understeer", 0.0))
        self.a_max = float(params.get("a_max", 15.0))
        self.max_shift = float(params.get("max_shift", 8.0))
        self.v_on = float(params.get("v_on", 3.0))

    def step(self, t, u):
        vx = u["vx"]
        delta = u["steer"] / self.steer_ratio
        v = vx if vx > 1.0 else 1.0
        r_ref = vx * delta / (self.wheelbase + self.understeer * vx * vx)
        r_lim = self.a_max / v
        r_ref = clamp(r_ref, -r_lim, r_lim)
        fade = clamp((vx - self.v_on) / 2.0, 0.0, 1.0)
        shift = clamp(self.gain * (r_ref - u["yaw_rate"]), -self.max_shift, self.max_shift) * fade
        return {"tq_fl": u["tq_fl_in"] - shift, "tq_fr": u["tq_fr_in"] + shift,
                "tq_rl": u["tq_rl_in"] - shift, "tq_rr": u["tq_rr_in"] + shift, "shift": shift}
