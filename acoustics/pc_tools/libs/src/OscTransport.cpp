#include "acoustics/osc/OscTransport.h"

#include "acoustics/osc/OscPacket.h"

#include <spdlog/spdlog.h>
#include <cstddef>
#include <limits>

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

void OscSender::enableEncryption(const OscEncryptor::Key256& key,
                                 const OscEncryptor::Iv128& iv) {
    std::lock_guard lock(mutex_);
    encryptor_.setKey(key, iv);
    sendCounter_ = 0;
}

void OscSender::disableEncryption() {
    std::lock_guard lock(mutex_);
    encryptor_.clear();
    sendCounter_ = 0;
}

bool OscSender::encryptionEnabled() const {
    std::lock_guard lock(mutex_);
    return encryptor_.enabled();
}

void OscSender::send(const Message& message) {
    sendPacket(encodeMessage(message));
}

void OscSender::send(const Bundle& bundle) {
    sendPacket(encodeBundle(bundle));
}

void OscSender::sendPacket(const std::vector<std::uint8_t>& payload) {
    std::lock_guard lock(mutex_);
    std::vector<std::uint8_t> buffer;
    if (encryptor_.enabled()) {
        if (sendCounter_ == std::numeric_limits<std::uint64_t>::max()) {
            spdlog::error("OSC encryption counter exhausted");
            return;
        }
        const std::uint64_t counter = ++sendCounter_;
        auto iv = encryptor_.deriveIv(counter);
        std::vector<std::uint8_t> ciphertext = encryptor_.encrypt(payload, iv);

        buffer.resize(sizeof(counter) + ciphertext.size());
        for (std::size_t i = 0; i < sizeof(counter); ++i) {
            buffer[i] = static_cast<std::uint8_t>((counter >> (56U - static_cast<unsigned>(i) * 8U)) & 0xFFU);
        }
        std::copy(ciphertext.begin(), ciphertext.end(), buffer.begin() + static_cast<std::ptrdiff_t>(sizeof(counter)));
    } else {
        buffer = payload;
    }
    std::error_code ec;
    socket_.send_to(asio::buffer(buffer), destination_, 0, ec);
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
        spdlog::debug("OSC raw packet from {}:{} ({} bytes)",
                      remoteEndpoint_.address().to_string(),
                      remoteEndpoint_.port(),
                      bytesReceived);
        std::vector<std::uint8_t> payload(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(bytesReceived));
        Packet packet = decodePacket(payload);
        if (spdlog::should_log(spdlog::level::debug)) {
            spdlog::debug("OSC packet decoded from {}:{} ({} bytes)",
                          remoteEndpoint_.address().to_string(),
                          remoteEndpoint_.port(),
                          bytesReceived);
        }
        if (handler_) {
            handler_(packet, remoteEndpoint_);
        }
    } catch (const std::exception& ex) {
        spdlog::warn("Failed to decode OSC packet from {}:{}: {}",
                     remoteEndpoint_.address().to_string(),
                     remoteEndpoint_.port(),
                     ex.what());
    }

    if (running_.load()) {
        issueReceive();
    }
}

}  // namespace acoustics::osc
