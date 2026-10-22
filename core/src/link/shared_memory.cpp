#include "vsim/link/shared_memory.hpp"

#include <stdexcept>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace vsim {
namespace {

#ifdef _WIN32
std::wstring object_name(const std::string& name) {
  const std::string full = "Local\\" + name;
  return std::wstring(full.begin(), full.end());  // names are ASCII
}

std::string last_error() {
  return "Windows error " + std::to_string(GetLastError());
}
#else
std::string object_name(const std::string& name) { return "/" + name; }

std::string last_error() { return std::strerror(errno); }
#endif

}  // namespace

SharedMemory::~SharedMemory() { release(); }

SharedMemory::SharedMemory(SharedMemory&& other) noexcept
    : data_(std::exchange(other.data_, nullptr)),
      size_(std::exchange(other.size_, 0)),
      existed_(other.existed_),
      handle_(std::exchange(other.handle_, nullptr)),
      unlink_(std::exchange(other.unlink_, {})) {}

SharedMemory& SharedMemory::operator=(SharedMemory&& other) noexcept {
  if (this != &other) {
    release();
    data_ = std::exchange(other.data_, nullptr);
    size_ = std::exchange(other.size_, 0);
    existed_ = other.existed_;
    handle_ = std::exchange(other.handle_, nullptr);
    unlink_ = std::exchange(other.unlink_, {});
  }
  return *this;
}

void SharedMemory::release() {
#ifdef _WIN32
  if (data_) {
    UnmapViewOfFile(data_);
  }
  if (handle_) {
    CloseHandle(static_cast<HANDLE>(handle_));
  }
#else
  if (data_) {
    munmap(data_, size_);
  }
  if (!unlink_.empty()) {
    shm_unlink(unlink_.c_str());
  }
#endif
  data_ = nullptr;
  handle_ = nullptr;
  size_ = 0;
  unlink_.clear();
}

SharedMemory SharedMemory::create(const std::string& name, std::size_t size) {
  SharedMemory memory;
  const auto fail = [&](const char* what) {
    return std::runtime_error("shared memory '" + name + "': " + what + ": " + last_error());
  };
#ifdef _WIN32
  const auto high = static_cast<DWORD>(static_cast<unsigned long long>(size) >> 32);
  const auto low = static_cast<DWORD>(size & 0xffffffffu);
  HANDLE handle = CreateFileMappingW(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, high, low,
                                     object_name(name).c_str());
  if (!handle) {
    throw fail("CreateFileMapping");
  }
  memory.existed_ = GetLastError() == ERROR_ALREADY_EXISTS;
  memory.handle_ = handle;
  memory.data_ = MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, size);
  if (!memory.data_) {
    throw fail("MapViewOfFile (an older vsim may hold a smaller object of this name)");
  }
  memory.size_ = size;
#else
  const std::string object = object_name(name);
  int fd = shm_open(object.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
  if (fd < 0 && errno == EEXIST) {
    memory.existed_ = true;
    fd = shm_open(object.c_str(), O_RDWR, 0600);
  }
  if (fd < 0) {
    throw fail("shm_open");
  }
  if (ftruncate(fd, static_cast<off_t>(size)) != 0) {
    close(fd);
    throw fail("ftruncate");
  }
  void* data = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (data == MAP_FAILED) {
    throw fail("mmap");
  }
  memory.data_ = data;
  memory.size_ = size;
  memory.unlink_ = object;
#endif
  return memory;
}

SharedMemory SharedMemory::open(const std::string& name) {
  SharedMemory memory;
#ifdef _WIN32
  HANDLE handle = OpenFileMappingW(FILE_MAP_ALL_ACCESS, FALSE, object_name(name).c_str());
  if (!handle) {
    return memory;
  }
  memory.handle_ = handle;
  memory.data_ = MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, 0);
  if (!memory.data_) {
    memory.release();
    return memory;
  }
  MEMORY_BASIC_INFORMATION info{};
  VirtualQuery(memory.data_, &info, sizeof info);
  memory.size_ = info.RegionSize;  // whole pages; the region headers carry exact sizes
#else
  const int fd = shm_open(object_name(name).c_str(), O_RDWR, 0600);
  if (fd < 0) {
    return memory;
  }
  struct stat info {};
  if (fstat(fd, &info) != 0 || info.st_size <= 0) {
    close(fd);
    return memory;
  }
  const auto size = static_cast<std::size_t>(info.st_size);
  void* data = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (data == MAP_FAILED) {
    return memory;
  }
  memory.data_ = data;
  memory.size_ = size;
#endif
  memory.existed_ = true;
  return memory;
}

}  // namespace vsim
