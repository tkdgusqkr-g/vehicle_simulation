"""A VCU as an external program (vsim.external): torque from the pedal on all four
motors, and a status frame on the CAN bus every 10 ms. Stands in for real firmware
running in an MCU emulator; vsim starts it (components.<name>.command) or you start
it yourself with VSIM_EXTERNAL=host:port.

    components:
      vcu:
        type: external
        slot: vcu
        period: 0.001
        command: [python, ecus/simple_vcu.py]
        ports: {in: {pedal: drv.cmd.throttle},
                out: {tq_fl: pt.motor.fl.torque_cmd, tq_fr: pt.motor.fr.torque_cmd,
                      tq_rl: pt.motor.rl.torque_cmd, tq_rr: pt.motor.rr.torque_cmd}}
"""
from vsim.external import Ecu, Frame

TORQUE_MAX = 21.0  # [N m] per motor

ecu = Ecu("simple_vcu", inputs={"pedal": "-"},
          outputs={"tq_fl": "N*m", "tq_fr": "N*m", "tq_rl": "N*m", "tq_rr": "N*m"}, version="1.0")
steps = 0


def step(t, inputs, frames):
    global steps
    steps += 1
    torque = TORQUE_MAX * min(max(inputs["pedal"], 0.0), 1.0)
    out = []
    if ecu.setup["buses"] and steps % 10 == 0:
        counter = (steps // 10) & 0xFF
        out.append(Frame(bus=0, id=0x3E0, data=bytes([counter, int(torque * 10) & 0xFF])))
    return {"tq_fl": torque, "tq_fr": torque, "tq_rl": torque, "tq_rr": torque}, out


if __name__ == "__main__":
    ecu.run(step)
