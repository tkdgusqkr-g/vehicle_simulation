"""Client side of vsim's external components (design doc §11, §12.5).

An external component is a separate program that vsim drives step by step
over TCP (sdk/c/include/vsim_external.h has the protocol). vsim listens; the
program connects to $VSIM_EXTERNAL, declares its ports and answers every
step. Declared in the vehicle project like any component:

    components:
      fw_vcu:
        type: external
        slot: vcu
        period: 0.001
        command: [python, ecus/fw_vcu.py]   # optional: vsim starts it (else start it yourself)
        listen: 127.0.0.1:47800             # optional: fixed address (default: a free port)
        can: [can_main]                     # buses whose frames it sees and sends
        ports: {in: {pedal: drv.cmd.throttle}, out: {tq: pt.motor.rl.torque_cmd}}

The program:

    from vsim.external import Ecu, Frame

    ecu = Ecu("fw_vcu", inputs={"pedal": "-"}, outputs={"tq": "N*m"})

    def step(t, inputs, frames):
        out_frames = [Frame(bus=0, id=0x100, data=bytes([1, 2]))]
        return {"tq": 21.0 * inputs["pedal"]}, out_frames

    ecu.run(step)        # until vsim ends the run

An MCU emulator bridge (Renode, QEMU) does the same: each step it advances
the emulated firmware by `period`, moves ADC values and GPIOs in, outputs
out, and passes CAN frames between the emulated controller and vsim's buses.
"""

import json
import os
import socket
import struct
from dataclasses import dataclass, field

PROTOCOL = 1
HELLO, SETUP, STEP, RESULT, END = 1, 2, 3, 4, 5
EXTENDED, FD, BRS = 1, 2, 4


@dataclass
class Frame:
    bus: int = 0
    id: int = 0
    data: bytes = b""
    extended: bool = False
    fd: bool = False
    brs: bool = False


@dataclass
class Ecu:
    """One external component: its name, version and ports {name: unit} in order."""
    name: str
    inputs: dict = field(default_factory=dict)
    outputs: dict = field(default_factory=dict)
    version: str = "1"

    def __post_init__(self):
        self.sock = None
        self.setup = None

    # --- connection -------------------------------------------------------------------------
    def connect(self, address=None, timeout=30.0):
        """Connects to vsim ("host:port", default $VSIM_EXTERNAL), sends HELLO, returns SETUP."""
        address = address or os.environ.get("VSIM_EXTERNAL")
        if not address:
            raise RuntimeError("no address: pass one or set VSIM_EXTERNAL (vsim prints it while it waits)")
        host, _, port = address.rpartition(":")
        self.sock = socket.create_connection((host or "127.0.0.1", int(port)), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(None)
        ports = [{"name": n, "unit": u, "dir": "in"} for n, u in self.inputs.items()]
        ports += [{"name": n, "unit": u, "dir": "out"} for n, u in self.outputs.items()]
        self._send(HELLO, json.dumps({"protocol": PROTOCOL, "name": self.name, "version": self.version,
                                      "ports": ports}).encode())
        kind, payload = self._read()
        if kind != SETUP:
            raise RuntimeError(f"expected SETUP, got message type {kind}")
        self.setup = json.loads(payload)
        return self.setup

    def run(self, step, address=None):
        """Serves steps until the run ends: step(t, inputs: dict, frames: [Frame]) ->
        (outputs: dict, frames: [Frame]) (or just outputs)."""
        if self.sock is None:
            self.connect(address)
        names_in, names_out = list(self.inputs), list(self.outputs)
        try:
            while True:
                kind, payload = self._read()
                if kind == END:
                    return
                if kind != STEP:
                    raise RuntimeError(f"expected STEP, got message type {kind}")
                t, values, frames = _parse_step(payload)
                result = step(t, dict(zip(names_in, values)), frames)
                outputs, out_frames = result if isinstance(result, tuple) else (result, [])
                self._send(RESULT, _build_result([float(outputs[n]) for n in names_out], out_frames))
        finally:
            self.close()

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    # --- framing ----------------------------------------------------------------------------
    def _send(self, kind, payload):
        self.sock.sendall(struct.pack("<IB", len(payload) + 1, kind) + payload)

    def _recv(self, n):
        chunks = []
        while n:
            chunk = self.sock.recv(n)
            if not chunk:
                raise ConnectionError("vsim closed the connection")
            chunks.append(chunk)
            n -= len(chunk)
        return b"".join(chunks)

    def _read(self):
        length, kind = struct.unpack("<IB", self._recv(5))
        return kind, self._recv(length - 1)


def _parse_step(payload):
    t, n_in = struct.unpack_from("<dI", payload, 0)
    values = list(struct.unpack_from(f"<{n_in}d", payload, 12))
    p = 12 + 8 * n_in
    (n_frames,) = struct.unpack_from("<I", payload, p)
    p += 4
    frames = []
    for _ in range(n_frames):
        bus, flags, ident, length = struct.unpack_from("<BBIB", payload, p)
        p += 7
        frames.append(Frame(bus, ident, bytes(payload[p:p + length]), bool(flags & EXTENDED), bool(flags & FD),
                            bool(flags & BRS)))
        p += length
    return t, values, frames


def _build_result(outputs, frames):
    out = struct.pack(f"<I{len(outputs)}d", len(outputs), *outputs) + struct.pack("<I", len(frames))
    for f in frames:
        flags = (EXTENDED if f.extended else 0) | (FD if f.fd else 0) | (BRS if f.brs else 0)
        out += struct.pack("<BBIB", f.bus, flags, f.id, len(f.data)) + bytes(f.data)
    return out
