#include "vsim/api/api_server.hpp"

#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocketServer.h>

#include <mutex>
#include <utility>

namespace vsim {
namespace {

// Drop the oldest queued messages if the sim loop stops draining them.
constexpr std::size_t kMaxQueuedMessages = 1000;

}  // namespace

struct ApiServer::Impl {
  Impl(int port, const std::string& host) : server(port, host) {}

  ix::WebSocketServer server;
  mutable std::mutex mutex;
  std::vector<std::string> inbox;
  std::string hello;
  bool started = false;
};

ApiServer::ApiServer(int port, std::string host) {
  ix::initNetSystem();
  impl_ = std::make_unique<Impl>(port, host);
  impl_->server.setOnClientMessageCallback(
      [impl = impl_.get()](std::shared_ptr<ix::ConnectionState> /*state*/, ix::WebSocket& socket,
                           const ix::WebSocketMessagePtr& message) {
        if (message->type == ix::WebSocketMessageType::Open) {
          std::string hello;
          {
            std::lock_guard lock(impl->mutex);
            hello = impl->hello;
          }
          if (!hello.empty()) {
            socket.send(hello);
          }
        } else if (message->type == ix::WebSocketMessageType::Message) {
          std::lock_guard lock(impl->mutex);
          if (impl->inbox.size() >= kMaxQueuedMessages) {
            impl->inbox.erase(impl->inbox.begin());
          }
          impl->inbox.push_back(message->str);
        }
      });
}

ApiServer::~ApiServer() {
  stop();
  ix::uninitNetSystem();
}

std::optional<std::string> ApiServer::start() {
  const auto [ok, error] = impl_->server.listen();
  if (!ok) {
    return error;
  }
  impl_->server.disablePerMessageDeflate();
  impl_->server.start();
  impl_->started = true;
  return std::nullopt;
}

void ApiServer::stop() {
  if (impl_ && impl_->started) {
    impl_->server.stop();
    impl_->started = false;
  }
}

void ApiServer::set_hello(std::string text) {
  std::lock_guard lock(impl_->mutex);
  impl_->hello = std::move(text);
}

void ApiServer::broadcast(const std::string& text) {
  for (const auto& client : impl_->server.getClients()) {
    client->send(text);
  }
}

std::vector<std::string> ApiServer::take_messages() {
  std::lock_guard lock(impl_->mutex);
  return std::exchange(impl_->inbox, {});
}

std::size_t ApiServer::client_count() const { return impl_->server.getClients().size(); }

}  // namespace vsim
