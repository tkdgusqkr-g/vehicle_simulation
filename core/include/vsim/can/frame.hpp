#pragma once

#include <cstdint>
#include <vector>

namespace vsim {

struct CanFrame {
  std::uint32_t id = 0;
  bool extended = false;
  bool fd = false;
  bool brs = true;  // CAN FD: data phase at the data bit rate
  std::vector<std::uint8_t> data;
};

// Bits a data frame takes on the bus (design doc §12.2, V-08): classic CAN
// from the actual bit stream (SOF to CRC, CRC-15, stuff bits after five equal
// bits) plus CRC delimiter, ACK, EOF and the 3-bit interframe space. CAN FD:
// arbitration and control at the nominal rate, data and CRC (with fixed stuff
// bits) at the data rate when BRS; dynamic stuffing counted on the bits sent.
struct FrameBits {
  int nominal = 0;  // at the nominal bit rate
  int data = 0;     // at the data bit rate (CAN FD with BRS)
  int stuff = 0;    // dynamic stuff bits included above
};
FrameBits frame_bits(const CanFrame& frame);

// Seconds on the bus.
double frame_duration(const CanFrame& frame, double bitrate, double data_bitrate);

// Worst-case classic frame bits (the textbook bound): 47 + 8n + floor((34 + 8n - 1) / 4)
// for standard IDs, 67 + 8n + floor((54 + 8n - 1) / 4) for extended.
int worst_case_frame_bits(int bytes, bool extended);

}  // namespace vsim
