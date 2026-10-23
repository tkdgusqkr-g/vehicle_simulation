#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vsim/can/frame.hpp"
#include "vsim/sim/params.hpp"

namespace vsim {

// A real CAN interface (design doc §12.2, R-41): the virtual network's frames
// go out on a real bus and a real ECU's frames come in, so a controller on the
// desk runs against the simulated car (rest-bus simulation, the first step to
// HIL). Only meaningful in real time (--mode realtime).
//
//   can:
//     devices:
//       - {bus: can_main, type: socketcan, channel: can0}                    # Linux (also vcan0)
//       - {bus: can_main, type: pcan, channel: PCAN_USBBUS1, bitrate: 1000000} # PEAK PCAN-Basic
//       - {bus: can_main, type: kvaser, channel: 0, bitrate: 1000000}        # Kvaser CANlib
//       - {bus: can_main, type: udp, local: 127.0.0.1:47900, remote: 127.0.0.1:47901}
//     nodes:
//       - {name: VCU, real: true}   # this node is the device's: its frames come from outside
//
// Vendor libraries load at run time (PCANBasic.dll / libpcanbasic.so, canlib32.dll /
// libcanlib.so.1; `library` overrides the name), so vsim builds without them.
// The udp type carries frames in datagrams (u8 flags: 1 extended, 2 FD, 4 BRS;
// u32 id little-endian; u8 length; data; repeated) for software gateways and tests.
class CanDevice {
 public:
  virtual ~CanDevice() = default;
  virtual void send(const CanFrame& frame) = 0;
  // Frames received since the last call (does not block).
  virtual std::vector<CanFrame> receive() = 0;
  virtual std::string describe() const = 0;
};

// The device described under `prefix` (can.devices.<i>).
std::unique_ptr<CanDevice> open_can_device(const ParamTree& params, const std::string& prefix);

// UDP datagram form of frames (the udp device's format).
std::vector<std::uint8_t> encode_udp_frames(const std::vector<CanFrame>& frames);
std::vector<CanFrame> decode_udp_frames(const std::vector<std::uint8_t>& datagram);

}  // namespace vsim
