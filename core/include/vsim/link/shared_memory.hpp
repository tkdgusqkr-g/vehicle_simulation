#pragma once

#include <cstddef>
#include <string>

namespace vsim {

// A named shared-memory object mapped into this process: a file mapping in
// the session's Local\ namespace on Windows, shm_open elsewhere.
class SharedMemory {
 public:
  SharedMemory() = default;
  ~SharedMemory();
  SharedMemory(SharedMemory&& other) noexcept;
  SharedMemory& operator=(SharedMemory&& other) noexcept;
  SharedMemory(const SharedMemory&) = delete;
  SharedMemory& operator=(const SharedMemory&) = delete;

  // Creates the object, or maps the existing one of that name (existed()).
  // The creator removes the name again when it is destroyed (POSIX; on
  // Windows the object lives while any process has it open). Throws on failure.
  static SharedMemory create(const std::string& name, std::size_t size);
  // Maps an existing object read-write; empty (false) if there is none.
  static SharedMemory open(const std::string& name);

  explicit operator bool() const { return data_ != nullptr; }
  void* data() const { return data_; }
  std::size_t size() const { return size_; }
  bool existed() const { return existed_; }
  // Leave the name in place on destruction (someone else owns it).
  void keep_name() { unlink_.clear(); }

 private:
  void release();

  void* data_ = nullptr;
  std::size_t size_ = 0;
  bool existed_ = false;
  void* handle_ = nullptr;  // Windows mapping handle
  std::string unlink_;      // POSIX name to remove on destruction
};

}  // namespace vsim
