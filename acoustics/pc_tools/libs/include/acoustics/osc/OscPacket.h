#pragma once

#include <cstdint>
#include <chrono>
#include <string>
#include <variant>
#include <vector>

namespace acoustics::osc {

using Blob = std::vector<std::uint8_t>;

using Argument = std::variant<
    std::int32_t,
    float,
    std::string,
    bool,
    Blob
>;

struct Message {
    std::string address;
    std::vector<Argument> arguments;
};

struct Bundle {
    std::uint64_t timetag;  // NTP 64-bit
    std::vector<Message> elements;
};

using Packet = std::variant<Message, Bundle>;

std::uint64_t toTimetag(std::chrono::system_clock::time_point tp);
std::chrono::system_clock::time_point fromTimetag(std::uint64_t tag);

std::vector<std::uint8_t> encodeMessage(const Message& message);
std::vector<std::uint8_t> encodeBundle(const Bundle& bundle);

Message decodeMessage(const std::vector<std::uint8_t>& payload);
Bundle decodeBundle(const std::vector<std::uint8_t>& payload);
Packet decodePacket(const std::vector<std::uint8_t>& payload);

}  // namespace acoustics::osc
