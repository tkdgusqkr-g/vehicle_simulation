#define MCAP_IMPLEMENTATION
#include "vsim/log/mcap_logger.hpp"

#include <mcap/writer.hpp>
#include <nlohmann/json.hpp>

#include <charconv>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

// Shortest round-trip text for a double; non-finite values become null.
void append_number(std::string& out, double value) {
  if (!std::isfinite(value)) {
    out += "null";
    return;
  }
  char buffer[32];
  const auto [end, error] = std::to_chars(buffer, buffer + sizeof(buffer), value);
  if (error != std::errc()) {
    throw std::runtime_error("cannot format log value");
  }
  out.append(buffer, end);
}

}  // namespace

struct McapLogger::Impl {
  std::ofstream stream;
  mcap::McapWriter writer;
  mcap::ChannelId channel = 0;
  std::vector<std::string> keys;  // JSON-quoted signal names, ready to append
  std::uint32_t sequence = 0;
  std::string buffer;
  bool open = false;
};

McapLogger::McapLogger(const std::filesystem::path& file, std::vector<std::string> signal_names)
    : impl_(std::make_unique<Impl>()) {
  impl_->stream.open(file, std::ios::binary);
  if (!impl_->stream) {
    throw std::runtime_error("cannot write '" + path_to_utf8(file) + "'");
  }
  mcap::McapWriterOptions options("");
  options.compression = mcap::Compression::None;
  impl_->writer.open(impl_->stream, options);
  impl_->open = true;

  nlohmann::json schema;
  schema["type"] = "object";
  schema["properties"]["t"] = {{"type", "number"}, {"description", "sim time [s]"}};
  for (const std::string& name : signal_names) {
    schema["properties"][name] = {{"type", "number"}};
    impl_->keys.push_back(nlohmann::json(name).dump());
  }
  mcap::Schema mcap_schema("vsim.Signals", "jsonschema", schema.dump());
  impl_->writer.addSchema(mcap_schema);
  mcap::Channel channel("signals", "json", mcap_schema.id);
  impl_->writer.addChannel(channel);
  impl_->channel = channel.id;
}

McapLogger::~McapLogger() {
  try {
    close();
  } catch (...) {
    // Destructors must not throw.
  }
}

void McapLogger::write(std::uint64_t time_ns, const std::vector<double>& values) {
  if (!impl_->open) {
    throw std::logic_error("McapLogger::write after close");
  }
  std::string& buffer = impl_->buffer;
  buffer.clear();
  buffer += "{\"t\":";
  append_number(buffer, static_cast<double>(time_ns) / 1e9);
  for (std::size_t i = 0; i < values.size() && i < impl_->keys.size(); ++i) {
    buffer += ',';
    buffer += impl_->keys[i];
    buffer += ':';
    append_number(buffer, values[i]);
  }
  buffer += '}';

  mcap::Message message;
  message.channelId = impl_->channel;
  message.sequence = ++impl_->sequence;
  message.logTime = time_ns;
  message.publishTime = time_ns;
  message.data = reinterpret_cast<const std::byte*>(buffer.data());
  message.dataSize = buffer.size();
  const mcap::Status status = impl_->writer.write(message);
  if (!status.ok()) {
    throw std::runtime_error("MCAP write failed: " + status.message);
  }
}

void McapLogger::close() {
  if (!impl_->open) {
    return;
  }
  impl_->open = false;
  impl_->writer.close();
  impl_->stream.close();
}

}  // namespace vsim
