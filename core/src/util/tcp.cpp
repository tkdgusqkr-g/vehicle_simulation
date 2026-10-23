#include "tcp.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

namespace vsim {
namespace {

#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket kInvalid = INVALID_SOCKET;
void close_socket(Socket s) { closesocket(s); }
int last_error() { return WSAGetLastError(); }
struct WinsockInit {
  WinsockInit() {
    WSADATA data;
    WSAStartup(MAKEWORD(2, 2), &data);
  }
  ~WinsockInit() { WSACleanup(); }
};
void ensure_winsock() { static WinsockInit init; }
int wait_readable(Socket s, double timeout_s) {
  WSAPOLLFD p{};
  p.fd = s;
  p.events = POLLRDNORM;
  return WSAPoll(&p, 1, static_cast<int>(timeout_s * 1000.0));
}
#else
using Socket = int;
constexpr Socket kInvalid = -1;
void close_socket(Socket s) { ::close(s); }
int last_error() { return errno; }
void ensure_winsock() {}
int wait_readable(Socket s, double timeout_s) {
  pollfd p{};
  p.fd = s;
  p.events = POLLIN;
  return ::poll(&p, 1, static_cast<int>(timeout_s * 1000.0));
}
#endif

Socket as_socket(std::intptr_t s) { return static_cast<Socket>(s); }

}  // namespace

TcpStream::TcpStream(std::intptr_t socket, std::string peer) : socket_(socket), peer_(std::move(peer)) {
  int one = 1;
  setsockopt(as_socket(socket_), IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof one);
}

TcpStream::~TcpStream() { close_socket(as_socket(socket_)); }

void TcpStream::send_all(const void* data, std::size_t size) {
  const char* p = static_cast<const char*>(data);
  while (size > 0) {
#ifdef _WIN32
    const int n = ::send(as_socket(socket_), p, static_cast<int>(size), 0);
#else
    const ssize_t n = ::send(as_socket(socket_), p, size, MSG_NOSIGNAL);
#endif
    if (n <= 0) {
      throw std::runtime_error("connection to " + peer_ + " lost while sending");
    }
    p += n;
    size -= static_cast<std::size_t>(n);
  }
}

void TcpStream::receive_all(void* data, std::size_t size, double timeout_s) {
  char* p = static_cast<char*>(data);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (size > 0) {
    const double left = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
    if (left <= 0.0 || wait_readable(as_socket(socket_), left) <= 0) {
      throw std::runtime_error(peer_ + " did not answer within " + std::to_string(timeout_s) + " s");
    }
#ifdef _WIN32
    const int n = ::recv(as_socket(socket_), p, static_cast<int>(size), 0);
#else
    const ssize_t n = ::recv(as_socket(socket_), p, size, 0);
#endif
    if (n <= 0) {
      throw std::runtime_error("connection to " + peer_ + " closed");
    }
    p += n;
    size -= static_cast<std::size_t>(n);
  }
}

TcpListener::TcpListener(const std::string& host, int port) {
  ensure_winsock();
  const Socket s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == kInvalid) {
    throw std::runtime_error("cannot open a socket (error " + std::to_string(last_error()) + ")");
  }
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof one);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    close_socket(s);
    throw std::invalid_argument("not an IPv4 address: " + host);
  }
  if (::bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 || ::listen(s, 1) != 0) {
    const int error = last_error();
    close_socket(s);
    throw std::runtime_error("cannot listen on " + host + ":" + std::to_string(port) + " (error " +
                             std::to_string(error) + ")");
  }
  socklen_t len = sizeof addr;
  getsockname(s, reinterpret_cast<sockaddr*>(&addr), &len);
  port_ = ntohs(addr.sin_port);
  socket_ = static_cast<std::intptr_t>(s);
}

TcpListener::~TcpListener() { close_socket(as_socket(socket_)); }

std::unique_ptr<TcpStream> TcpListener::accept(double timeout_s) {
  if (wait_readable(as_socket(socket_), timeout_s) <= 0) {
    throw std::runtime_error("nothing connected to port " + std::to_string(port_) + " within " +
                             std::to_string(timeout_s) + " s");
  }
  sockaddr_in addr{};
  socklen_t len = sizeof addr;
  const Socket c = ::accept(as_socket(socket_), reinterpret_cast<sockaddr*>(&addr), &len);
  if (c == kInvalid) {
    throw std::runtime_error("accept failed (error " + std::to_string(last_error()) + ")");
  }
  char text[INET_ADDRSTRLEN] = {};
  inet_ntop(AF_INET, &addr.sin_addr, text, sizeof text);
  return std::unique_ptr<TcpStream>(
      new TcpStream(static_cast<std::intptr_t>(c), std::string(text) + ":" + std::to_string(ntohs(addr.sin_port))));
}

// --- UDP -----------------------------------------------------------------------------------------

void split_address(const std::string& address, std::string& host, int& port) {
  const auto colon = address.rfind(':');
  if (colon == std::string::npos) {
    throw std::invalid_argument("'" + address + "': expected host:port");
  }
  host = address.substr(0, colon);
  port = std::stoi(address.substr(colon + 1));
}

UdpSocket::UdpSocket(const std::string& host, int port) {
  ensure_winsock();
  const Socket s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s == kInvalid) {
    throw std::runtime_error("cannot open a UDP socket (error " + std::to_string(last_error()) + ")");
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1 ||
      ::bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0) {
    const int error = last_error();
    close_socket(s);
    throw std::runtime_error("cannot bind UDP " + host + ":" + std::to_string(port) + " (error " +
                             std::to_string(error) + ")");
  }
#ifdef _WIN32
  u_long nonblocking = 1;
  ioctlsocket(s, FIONBIO, &nonblocking);
#else
  fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
#endif
  socklen_t len = sizeof addr;
  getsockname(s, reinterpret_cast<sockaddr*>(&addr), &len);
  port_ = ntohs(addr.sin_port);
  socket_ = static_cast<std::intptr_t>(s);
}

UdpSocket::~UdpSocket() { close_socket(as_socket(socket_)); }

void UdpSocket::send_to(const std::string& host, int port, const void* data, std::size_t size) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    throw std::invalid_argument("not an IPv4 address: " + host);
  }
  ::sendto(as_socket(socket_), static_cast<const char*>(data), static_cast<int>(size), 0,
           reinterpret_cast<sockaddr*>(&addr), sizeof addr);
}

std::vector<std::uint8_t> UdpSocket::receive(std::size_t capacity) {
  std::vector<std::uint8_t> buffer(capacity);
  const auto n = ::recv(as_socket(socket_), reinterpret_cast<char*>(buffer.data()), static_cast<int>(capacity), 0);
  if (n <= 0) {
    return {};
  }
  buffer.resize(static_cast<std::size_t>(n));
  return buffer;
}

// --- Child process -----------------------------------------------------------------------------

#ifdef _WIN32
namespace {
std::wstring widen(const std::string& s) {
  if (s.empty()) {
    return {};
  }
  const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
  std::wstring w(static_cast<std::size_t>(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
  return w;
}
std::wstring quote(const std::wstring& arg) {
  if (!arg.empty() && arg.find_first_of(L" \t\"") == std::wstring::npos) {
    return arg;
  }
  std::wstring out = L"\"";
  for (const wchar_t c : arg) {
    out += c == L'"' ? L"\\\"" : std::wstring(1, c);
  }
  return out + L"\"";
}
}  // namespace

ChildProcess::ChildProcess(const std::vector<std::string>& argv,
                           const std::vector<std::pair<std::string, std::string>>& env, const std::string& working_dir) {
  std::wstring command;
  for (const std::string& a : argv) {
    command += (command.empty() ? L"" : L" ") + quote(widen(a));
  }
  for (const auto& [key, value] : env) {
    SetEnvironmentVariableW(widen(key).c_str(), widen(value).c_str());  // inherited by the child
  }
  STARTUPINFOW si{};
  si.cb = sizeof si;
  PROCESS_INFORMATION pi{};
  std::wstring dir = widen(working_dir);
  if (!CreateProcessW(nullptr, command.data(), nullptr, nullptr, FALSE, 0, nullptr,
                      dir.empty() ? nullptr : dir.c_str(), &si, &pi)) {
    throw std::runtime_error("cannot start " + argv.front() + " (error " + std::to_string(GetLastError()) + ")");
  }
  CloseHandle(pi.hThread);
  handle_ = reinterpret_cast<std::intptr_t>(pi.hProcess);
}

bool ChildProcess::running(int* exit_code) {
  if (!done_) {
    DWORD code = 0;
    GetExitCodeProcess(reinterpret_cast<HANDLE>(handle_), &code);
    if (code != STILL_ACTIVE) {
      done_ = true;
      code_ = static_cast<int>(code);
    }
  }
  if (done_ && exit_code != nullptr) {
    *exit_code = code_;
  }
  return !done_;
}

void ChildProcess::finish(double timeout_s) {
  if (!done_ && WaitForSingleObject(reinterpret_cast<HANDLE>(handle_), static_cast<DWORD>(timeout_s * 1000.0)) ==
                    WAIT_TIMEOUT) {
    TerminateProcess(reinterpret_cast<HANDLE>(handle_), 1);
    WaitForSingleObject(reinterpret_cast<HANDLE>(handle_), 2000);
  }
  running();
}

ChildProcess::~ChildProcess() {
  finish(0.0);
  CloseHandle(reinterpret_cast<HANDLE>(handle_));
}
#else
ChildProcess::ChildProcess(const std::vector<std::string>& argv,
                           const std::vector<std::pair<std::string, std::string>>& env, const std::string& working_dir) {
  std::vector<std::string> environment;
  for (char** e = environ; *e != nullptr; ++e) {
    const std::string entry(*e);
    const std::string key = entry.substr(0, entry.find('='));
    if (std::none_of(env.begin(), env.end(), [&](const auto& kv) { return kv.first == key; })) {
      environment.push_back(entry);  // overridden ones are left out
    }
  }
  for (const auto& [key, value] : env) {
    environment.push_back(key + "=" + value);
  }
  std::vector<char*> args, envp;
  for (const std::string& a : argv) {
    args.push_back(const_cast<char*>(a.c_str()));
  }
  args.push_back(nullptr);
  for (const std::string& e : environment) {
    envp.push_back(const_cast<char*>(e.c_str()));
  }
  envp.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
#if defined(__GLIBC__) && (__GLIBC__ > 2 || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 29))
  if (!working_dir.empty()) {
    posix_spawn_file_actions_addchdir_np(&actions, working_dir.c_str());
  }
#else
  (void)working_dir;
#endif
  pid_t pid = 0;
  const int error = posix_spawnp(&pid, args[0], &actions, nullptr, args.data(), envp.data());
  posix_spawn_file_actions_destroy(&actions);
  if (error != 0) {
    throw std::runtime_error("cannot start " + argv.front() + " (" + std::strerror(error) + ")");
  }
  handle_ = pid;
}

bool ChildProcess::running(int* exit_code) {
  if (!done_) {
    int status = 0;
    if (waitpid(static_cast<pid_t>(handle_), &status, WNOHANG) == static_cast<pid_t>(handle_)) {
      done_ = true;
      code_ = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    }
  }
  if (done_ && exit_code != nullptr) {
    *exit_code = code_;
  }
  return !done_;
}

void ChildProcess::finish(double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (running() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (running()) {
    kill(static_cast<pid_t>(handle_), SIGKILL);
    int status = 0;
    waitpid(static_cast<pid_t>(handle_), &status, 0);
    done_ = true;
    code_ = -1;
  }
}

ChildProcess::~ChildProcess() { finish(0.0); }
#endif

}  // namespace vsim
