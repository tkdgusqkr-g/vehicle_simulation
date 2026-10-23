#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace vsim {

// A CAN database (Vector DBC): nodes, messages, signals (design doc §12.2).
// Covers what vehicle networks use: standard and extended IDs, CAN FD
// frames (VFrameFormat), Intel and Motorola byte order, signed, float and
// multiplexed signals, cycle times (GenMsgCycleTime) and comments.
struct DbcSignal {
  std::string name;
  int start_bit = 0;   // as written in the DBC (Motorola: the MSB, sawtooth numbering)
  int length = 1;
  bool little_endian = true;  // @1 Intel, @0 Motorola
  bool is_signed = false;
  enum class Kind { Integer, Float32, Float64 } kind = Kind::Integer;
  double factor = 1.0, offset = 0.0, minimum = 0.0, maximum = 0.0;
  std::string unit;
  std::vector<std::string> receivers;
  bool multiplexer = false;             // M
  std::optional<std::int64_t> mux_value; // mN: present only when the multiplexer is N
  std::string comment;

  // Physical value -> raw (rounded, saturated to the bit field) and back.
  std::uint64_t encode(double physical) const;
  double decode(std::uint64_t raw) const;
  // The raw value is written into / read from the payload.
  void write(std::vector<std::uint8_t>& data, double physical) const;
  double read(const std::vector<std::uint8_t>& data) const;
  std::uint64_t read_raw(const std::vector<std::uint8_t>& data) const;
};

struct DbcMessage {
  std::uint32_t id = 0;      // without the extended flag
  bool extended = false;
  bool fd = false;           // CAN FD frame (VFrameFormat 14/15)
  std::string name;
  int length = 8;            // payload bytes
  std::string transmitter;
  std::vector<DbcSignal> signals;
  double cycle_time = 0.0;   // [s] 0: not cyclic
  std::string comment;

  const DbcSignal* signal(const std::string& name) const;
  const DbcSignal* multiplexer() const;
};

struct Dbc {
  std::string version;
  std::vector<std::string> nodes;
  std::vector<DbcMessage> messages;

  static Dbc parse(const std::string& text, const std::string& source = "dbc");
  static Dbc from_file(const std::filesystem::path& file);
  // DBC text that parses back to the same database.
  std::string to_text() const;

  const DbcMessage* message(const std::string& name) const;
  const DbcMessage* message(std::uint32_t id, bool extended) const;
};

// Payload bytes for a DLC code and back (CAN FD: 12, 16, 20, 24, 32, 48, 64).
int dlc_to_length(int dlc);
int length_to_dlc(int length);

}  // namespace vsim
