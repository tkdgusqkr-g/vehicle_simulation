#include "vsim/can/frame.hpp"

#include <algorithm>

#include "vsim/can/dbc.hpp"

namespace vsim {
namespace {

void push(std::vector<int>& bits, std::uint32_t value, int count) {
  for (int i = count - 1; i >= 0; --i) {
    bits.push_back(static_cast<int>((value >> i) & 1U));
  }
}

// CRC-15 of classic CAN (polynomial 0x4599) over the unstuffed bits.
std::uint32_t crc15(const std::vector<int>& bits) {
  std::uint32_t crc = 0;
  for (const int bit : bits) {
    const std::uint32_t next = static_cast<std::uint32_t>(bit) ^ ((crc >> 14) & 1U);
    crc = (crc << 1) & 0x7FFFU;
    if (next) {
      crc ^= 0x4599U;
    }
  }
  return crc;
}

// Stuff bits: after five equal bits one of the opposite value, which counts
// toward the next run.
int stuff_count(const std::vector<int>& bits, int run_start = 0, int last_start = -1) {
  int stuffed = 0, run = run_start, last = last_start;
  for (const int bit : bits) {
    if (bit == last) {
      ++run;
    } else {
      run = 1;
      last = bit;
    }
    if (run == 5) {
      ++stuffed;
      last = 1 - bit;  // the stuff bit
      run = 1;
    }
  }
  return stuffed;
}

}  // namespace

FrameBits frame_bits(const CanFrame& frame) {
  const int bytes = static_cast<int>(frame.data.size());
  FrameBits out;
  std::vector<int> bits;
  bits.push_back(0);  // SOF
  if (!frame.fd) {
    if (frame.extended) {
      push(bits, frame.id >> 18, 11);
      bits.push_back(1);  // SRR
      bits.push_back(1);  // IDE
      push(bits, frame.id & 0x3FFFFU, 18);
      bits.push_back(0);  // RTR (data frame)
      bits.push_back(0);  // r1
      bits.push_back(0);  // r0
    } else {
      push(bits, frame.id, 11);
      bits.push_back(0);  // RTR
      bits.push_back(0);  // IDE
      bits.push_back(0);  // r0
    }
    push(bits, static_cast<std::uint32_t>(std::min(bytes, 8)), 4);
    for (const std::uint8_t b : frame.data) {
      push(bits, b, 8);
    }
    push(bits, crc15(bits), 15);
    out.stuff = stuff_count(bits);
    out.nominal = static_cast<int>(bits.size()) + out.stuff + 3 /*CRC del, ACK, ACK del*/ + 7 /*EOF*/ + 3 /*IFS*/;
    return out;
  }
  // CAN FD: arbitration phase.
  if (frame.extended) {
    push(bits, frame.id >> 18, 11);
    bits.push_back(1);  // SRR
    bits.push_back(1);  // IDE
    push(bits, frame.id & 0x3FFFFU, 18);
    bits.push_back(0);  // RRS
  } else {
    push(bits, frame.id, 11);
    bits.push_back(0);  // RRS
    bits.push_back(0);  // IDE
  }
  bits.push_back(1);  // FDF
  bits.push_back(0);  // res
  bits.push_back(frame.brs ? 1 : 0);  // BRS
  const int arbitration_stuff = stuff_count(bits);
  int last = bits.back(), run = 1;
  for (std::size_t i = bits.size() - 1; i-- > 0 && bits[i] == last;) {
    ++run;
  }
  std::vector<int> data;
  data.push_back(0);  // ESI
  push(data, static_cast<std::uint32_t>(length_to_dlc(bytes)), 4);
  for (const std::uint8_t b : frame.data) {
    push(data, b, 8);
  }
  const int data_stuff = stuff_count(data, std::min(run, 4), last);
  // Stuff count (3 bits + parity) with its fixed stuff bit, CRC with fixed
  // stuff bits every four bits, then the CRC delimiter.
  const int crc = bytes <= 16 ? 17 : 21;
  const int crc_field = 4 + 1 + crc + (crc + 3) / 4 + 1;
  const int data_phase = static_cast<int>(data.size()) + data_stuff + crc_field;
  out.stuff = arbitration_stuff + data_stuff;
  const int arbitration = static_cast<int>(bits.size()) + arbitration_stuff;
  const int tail = 2 /*ACK, ACK del*/ + 7 /*EOF*/ + 3 /*IFS*/;
  if (frame.brs) {
    out.nominal = arbitration + tail;
    out.data = data_phase;
  } else {
    out.nominal = arbitration + data_phase + tail;
  }
  return out;
}

double frame_duration(const CanFrame& frame, double bitrate, double data_bitrate) {
  const FrameBits b = frame_bits(frame);
  return static_cast<double>(b.nominal) / bitrate + (b.data > 0 ? static_cast<double>(b.data) / data_bitrate : 0.0);
}

int worst_case_frame_bits(int bytes, bool extended) {
  const int stuffable = (extended ? 54 : 34) + 8 * bytes;
  return (extended ? 67 : 47) + 8 * bytes + (stuffable - 1) / 4;
}

}  // namespace vsim
