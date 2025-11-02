#include "acoustics/osc/OscTransport.h"

#include "acoustics/osc/OscPacket.h"

#include <spdlog/spdlog.h>

namespace acoustics::osc {

IoContextRunner::IoContextRunner() = default;

IoContextRunner::~IoContextRunner() {
    stop();
}

void IoContextRunner::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }

    workGuard_.emplace(asio::make_work_guard(ioContext_));
    worker_ = std::thread([this] {
        try {
            ioContext_.run();
        } catch (const std::exception& ex) {
            spdlog::error("IoContextRunner crashed: {}", ex.what());
        }
    });
}

void IoContextRunner::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;
    }

    if (workGuard_) {
        workGuard_->reset();
        workGuard_.reset();
    }
    ioContext_.stop();
    if (worker_.joinable()) {
        worker_.join();
    }
    ioContext_.restart();
}

OscSender::OscSender(asio::io_context& ctx,
                     const Endpoint& destination,
                     bool allowBroadcast)
    : socket_(ctx), destination_(destination), broadcast_(allowBroadcast) {
    socket_.open(destination.protocol());
    socket_.set_option(asio::socket_base::reuse_address(true));
    if (broadcast_) {
        socket_.set_option(asio::socket_base::broadcast(true));
    }
}

void OscSender::setEndpoint(const Endpoint& destination) {
    std::lock_guard lock(mutex_);
    destination_ = destination;
}

OscSender::Endpoint OscSender::endpoint() const {
    std::lock_guard lock(mutex_);
    return destination_;
}

void OscSender::setBroadcastEnabled(bool enable) {
    std::lock_guard lock(mutex_);
    broadcast_ = enable;
    std::error_code ec;
    socket_.set_option(asio::socket_base::broadcast(enable), ec);
    if (ec) {
        spdlog::warn("Failed to set broadcast {}: {}", enable ? "on" : "off", ec.message());
    }
}

bool OscSender::broadcastEnabled() const {
    std::lock_guard lock(mutex_);
    return broadcast_;
}

void OscSender::send(const Message& message) {
    sendPacket(encodeMessage(message));
}

void OscSender::send(const Bundle& bundle) {
    sendPacket(encodeBundle(bundle));
}

void OscSender::sendPacket(const std::vector<std::uint8_t>& payload) {
    std::lock_guard lock(mutex_);
    std::error_code ec;
    socket_.send_to(asio::buffer(payload), destination_, 0, ec);
    if (ec) {
        spdlog::warn("OSC send failed: {}", ec.message());
    }
}

OscListener::OscListener(asio::io_context& ctx,
                         const Endpoint& listenEndpoint,
                         PacketHandler handler)
    : socket_(ctx), handler_(std::move(handler)) {
    socket_.open(listenEndpoint.protocol());
    socket_.set_option(asio::socket_base::reuse_address(true));
    socket_.bind(listenEndpoint);
}

void OscListener::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }
    issueReceive();
}

void OscListener::stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        return;
    }
    std::error_code ec;
    socket_.cancel(ec);
    socket_.close(ec);
}

void OscListener::issueReceive() {
    socket_.async_receive_from(
        asio::buffer(buffer_),
        remoteEndpoint_,
        [this](std::error_code ec, std::size_t bytesReceived) {
            handleReceive(std::move(ec), bytesReceived);
        });
}

void OscListener::handleReceive(std::error_code ec, std::size_t bytesReceived) {
    if (ec) {
        if (running_.load()) {
            spdlog::warn("OSC receive error: {}", ec.message());
            issueReceive();
        }
        return;
    }

    try {
        std::vector<std::uint8_t> payload(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(bytesReceived));
        Packet packet = decodePacket(payload);
        if (handler_) {
            handler_(packet, remoteEndpoint_);
        }
    } catch (const std::exception& ex) {
        spdlog::warn("Failed to decode OSC packet: {}", ex.what());
    }

    if (running_.load()) {
        issueReceive();
    }
}

}  // namespace acoustics::osc
