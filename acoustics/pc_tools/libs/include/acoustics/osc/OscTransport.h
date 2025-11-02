#pragma once

#include "acoustics/osc/OscPacket.h"

#include <asio.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace acoustics::osc {

class IoContextRunner {
public:
    IoContextRunner();
    ~IoContextRunner();

    IoContextRunner(const IoContextRunner&) = delete;
    IoContextRunner& operator=(const IoContextRunner&) = delete;

    asio::io_context& context() noexcept { return ioContext_; }

    void start();
    void stop();

private:
    asio::io_context ioContext_;
    std::optional<asio::executor_work_guard<asio::io_context::executor_type>> workGuard_;
    std::thread worker_;
    std::atomic_bool running_{false};
};

class OscSender {
public:
    using Endpoint = asio::ip::udp::endpoint;

    OscSender(asio::io_context& ctx,
              const Endpoint& destination,
              bool allowBroadcast = false);

    void setEndpoint(const Endpoint& destination);
    Endpoint endpoint() const;

    void setBroadcastEnabled(bool enable);
    bool broadcastEnabled() const;

    void send(const Message& message);
    void send(const Bundle& bundle);

private:
    void sendPacket(const std::vector<std::uint8_t>& payload);

    asio::ip::udp::socket socket_;
    Endpoint destination_;
    bool broadcast_{false};
    mutable std::mutex mutex_;
};

class OscListener {
public:
    using Endpoint = asio::ip::udp::endpoint;
    using PacketHandler = std::function<void(const Packet&, const Endpoint&)>;

    OscListener(asio::io_context& ctx,
                const Endpoint& listenEndpoint,
                PacketHandler handler);

    void start();
    void stop();

private:
    void issueReceive();
    void handleReceive(std::error_code ec, std::size_t bytesReceived);

    asio::ip::udp::socket socket_;
    PacketHandler handler_;
    std::array<std::uint8_t, 4096> buffer_{};
    Endpoint remoteEndpoint_{};
    std::atomic_bool running_{false};
};

}  // namespace acoustics::osc
