#include "acoustics/osc/OscPacket.h"

#include <array>
#include <cstring>
#include <stdexcept>

namespace acoustics::osc {

namespace {

constexpr std::uint64_t NTP_UNIX_OFFSET = 2'208'988'800ULL;

std::size_t align4(std::size_t size) {
    return (size + 3U) & ~std::size_t(3U);
}

void appendPadded(std::vector<std::uint8_t>& buffer, const std::string& value) {
    buffer.insert(buffer.end(), value.begin(), value.end());
    buffer.push_back(0);
    while (buffer.size() % 4 != 0) {
        buffer.push_back(0);
    }
}

void appendBlob(std::vector<std::uint8_t>& buffer, const Blob& blob) {
    std::uint32_t len = static_cast<std::uint32_t>(blob.size());
    std::array<std::uint8_t, 4> lenBytes{
        static_cast<std::uint8_t>((len >> 24) & 0xFF),
        static_cast<std::uint8_t>((len >> 16) & 0xFF),
        static_cast<std::uint8_t>((len >> 8) & 0xFF),
        static_cast<std::uint8_t>(len & 0xFF),
    };
    buffer.insert(buffer.end(), lenBytes.begin(), lenBytes.end());
    buffer.insert(buffer.end(), blob.begin(), blob.end());
    while (buffer.size() % 4 != 0) {
        buffer.push_back(0);
    }
}

std::string readString(const std::vector<std::uint8_t>& data, std::size_t& offset) {
    std::size_t end = offset;
    while (end < data.size() && data[end] != 0) {
        ++end;
    }
    if (end >= data.size()) {
        throw std::runtime_error("OSC string not null terminated");
    }
    std::string result(reinterpret_cast<const char*>(data.data() + offset), end - offset);
    offset = align4(end + 1);
    if (offset > data.size()) {
        throw std::runtime_error("OSC string exceeded buffer");
    }
    return result;
}

Blob readBlob(const std::vector<std::uint8_t>& data, std::size_t& offset) {
    if (offset + 4 > data.size()) {
        throw std::runtime_error("OSC blob truncated");
    }
    std::uint32_t len = (static_cast<std::uint32_t>(data[offset]) << 24) |
                        (static_cast<std::uint32_t>(data[offset + 1]) << 16) |
                        (static_cast<std::uint32_t>(data[offset + 2]) << 8) |
                        (static_cast<std::uint32_t>(data[offset + 3]));
    offset += 4;
    if (offset + len > data.size()) {
        throw std::runtime_error("OSC blob length exceeds buffer");
    }
    Blob blob(data.begin() + static_cast<std::ptrdiff_t>(offset),
              data.begin() + static_cast<std::ptrdiff_t>(offset + len));
    offset = align4(offset + len);
    if (offset > data.size()) {
        throw std::runtime_error("OSC blob padding exceeds buffer");
    }
    return blob;
}

}  // namespace

std::uint64_t toTimetag(std::chrono::system_clock::time_point tp) {
    using seconds = std::chrono::seconds;
    using fractional = std::chrono::duration<double>;

    const auto duration = tp.time_since_epoch();
    const auto secPart = std::chrono::duration_cast<seconds>(duration);
    const auto fracPart = fractional(duration - secPart);

    auto ntpSeconds = static_cast<std::uint64_t>(secPart.count()) + NTP_UNIX_OFFSET;
    auto ntpFraction = static_cast<std::uint32_t>(fracPart.count() * (static_cast<double>(1ULL << 32)));

    return (ntpSeconds << 32) | ntpFraction;
}

std::chrono::system_clock::time_point fromTimetag(std::uint64_t tag) {
    std::uint64_t secondsPart = tag >> 32;
    std::uint64_t fractionPart = tag & 0xFFFFFFFFULL;
    std::int64_t unixSeconds = static_cast<std::int64_t>(secondsPart) - static_cast<std::int64_t>(NTP_UNIX_OFFSET);
    double fractional = static_cast<double>(fractionPart) / static_cast<double>(1ULL << 32);
    auto tp = std::chrono::system_clock::time_point{std::chrono::seconds{unixSeconds}} +
              std::chrono::duration_cast<std::chrono::system_clock::duration>(
                  std::chrono::duration<double>(fractional));
    return tp;
}

std::vector<std::uint8_t> encodeMessage(const Message& message) {
    if (message.address.empty() || message.address.front() != '/') {
        throw std::invalid_argument("OSC address must start with '/'");
    }

    std::vector<std::uint8_t> buffer;
    appendPadded(buffer, message.address);

    std::string typeTags(",");
    std::vector<std::uint8_t> payload;

    for (const auto& arg : message.arguments) {
        if (std::holds_alternative<std::int32_t>(arg)) {
            typeTags.push_back('i');
            std::int32_t value = std::get<std::int32_t>(arg);
            std::array<std::uint8_t, 4> bytes{
                static_cast<std::uint8_t>((value >> 24) & 0xFF),
                static_cast<std::uint8_t>((value >> 16) & 0xFF),
                static_cast<std::uint8_t>((value >> 8) & 0xFF),
                static_cast<std::uint8_t>(value & 0xFF),
            };
            payload.insert(payload.end(), bytes.begin(), bytes.end());
        } else if (std::holds_alternative<float>(arg)) {
            typeTags.push_back('f');
            float value = std::get<float>(arg);
            std::uint32_t raw;
            std::memcpy(&raw, &value, sizeof(float));
            std::array<std::uint8_t, 4> bytes{
                static_cast<std::uint8_t>((raw >> 24) & 0xFF),
                static_cast<std::uint8_t>((raw >> 16) & 0xFF),
                static_cast<std::uint8_t>((raw >> 8) & 0xFF),
                static_cast<std::uint8_t>(raw & 0xFF),
            };
            payload.insert(payload.end(), bytes.begin(), bytes.end());
        } else if (std::holds_alternative<std::string>(arg)) {
            typeTags.push_back('s');
            appendPadded(payload, std::get<std::string>(arg));
        } else if (std::holds_alternative<bool>(arg)) {
            typeTags.push_back(std::get<bool>(arg) ? 'T' : 'F');
        } else if (std::holds_alternative<Blob>(arg)) {
            typeTags.push_back('b');
            appendBlob(payload, std::get<Blob>(arg));
        } else {
            throw std::runtime_error("Unsupported OSC argument type");
        }
    }

    appendPadded(buffer, typeTags);
    buffer.insert(buffer.end(), payload.begin(), payload.end());
    return buffer;
}

std::vector<std::uint8_t> encodeBundle(const Bundle& bundle) {
    std::vector<std::uint8_t> buffer;
    appendPadded(buffer, "#bundle");

    std::array<std::uint8_t, 8> timetagBytes{
        static_cast<std::uint8_t>((bundle.timetag >> 56) & 0xFF),
        static_cast<std::uint8_t>((bundle.timetag >> 48) & 0xFF),
        static_cast<std::uint8_t>((bundle.timetag >> 40) & 0xFF),
        static_cast<std::uint8_t>((bundle.timetag >> 32) & 0xFF),
        static_cast<std::uint8_t>((bundle.timetag >> 24) & 0xFF),
        static_cast<std::uint8_t>((bundle.timetag >> 16) & 0xFF),
        static_cast<std::uint8_t>((bundle.timetag >> 8) & 0xFF),
        static_cast<std::uint8_t>(bundle.timetag & 0xFF),
    };
    buffer.insert(buffer.end(), timetagBytes.begin(), timetagBytes.end());

    for (const auto& msg : bundle.elements) {
        auto encoded = encodeMessage(msg);
        std::uint32_t len = static_cast<std::uint32_t>(encoded.size());
        std::array<std::uint8_t, 4> lenBytes{
            static_cast<std::uint8_t>((len >> 24) & 0xFF),
            static_cast<std::uint8_t>((len >> 16) & 0xFF),
            static_cast<std::uint8_t>((len >> 8) & 0xFF),
            static_cast<std::uint8_t>(len & 0xFF),
        };
        buffer.insert(buffer.end(), lenBytes.begin(), lenBytes.end());
        buffer.insert(buffer.end(), encoded.begin(), encoded.end());
    }
    return buffer;
}

Message decodeMessage(const std::vector<std::uint8_t>& payload) {
    std::size_t offset = 0;
    Message message;
    message.address = readString(payload, offset);
    std::string tags = readString(payload, offset);
    if (tags.empty() || tags.front() != ',') {
        throw std::runtime_error("Invalid OSC type tag string");
    }

    for (std::size_t i = 1; i < tags.size(); ++i) {
        char tag = tags[i];
        switch (tag) {
            case 'i': {
                if (offset + 4 > payload.size()) {
                    throw std::runtime_error("OSC int truncated");
                }
                std::int32_t value = (static_cast<std::int32_t>(payload[offset]) << 24) |
                                     (static_cast<std::int32_t>(payload[offset + 1]) << 16) |
                                     (static_cast<std::int32_t>(payload[offset + 2]) << 8) |
                                     (static_cast<std::int32_t>(payload[offset + 3]));
                offset += 4;
                message.arguments.emplace_back(value);
                break;
            }
            case 'f': {
                if (offset + 4 > payload.size()) {
                    throw std::runtime_error("OSC float truncated");
                }
                std::uint32_t raw = (static_cast<std::uint32_t>(payload[offset]) << 24) |
                                    (static_cast<std::uint32_t>(payload[offset + 1]) << 16) |
                                    (static_cast<std::uint32_t>(payload[offset + 2]) << 8) |
                                    (static_cast<std::uint32_t>(payload[offset + 3]));
                offset += 4;
                float value;
                std::memcpy(&value, &raw, sizeof(float));
                message.arguments.emplace_back(value);
                break;
            }
            case 's': {
                message.arguments.emplace_back(readString(payload, offset));
                break;
            }
            case 'b': {
                message.arguments.emplace_back(readBlob(payload, offset));
                break;
            }
            case 'T':
                message.arguments.emplace_back(true);
                break;
            case 'F':
                message.arguments.emplace_back(false);
                break;
            default:
                throw std::runtime_error("Unsupported OSC type tag");
        }
    }
    return message;
}

Bundle decodeBundle(const std::vector<std::uint8_t>& payload) {
    std::size_t offset = 0;
    auto bundleTag = readString(payload, offset);
    if (bundleTag != "#bundle") {
        throw std::runtime_error("Payload is not an OSC bundle");
    }
    if (offset + 8 > payload.size()) {
        throw std::runtime_error("OSC bundle missing timetag");
    }

    std::uint64_t timetag = 0;
    for (int i = 0; i < 8; ++i) {
        timetag = (timetag << 8) | payload[offset + i];
    }
    offset += 8;

    Bundle bundle{timetag, {}};

    while (offset < payload.size()) {
        if (offset + 4 > payload.size()) {
            throw std::runtime_error("OSC bundle element truncated");
        }
        std::uint32_t size = (static_cast<std::uint32_t>(payload[offset]) << 24) |
                             (static_cast<std::uint32_t>(payload[offset + 1]) << 16) |
                             (static_cast<std::uint32_t>(payload[offset + 2]) << 8) |
                             (static_cast<std::uint32_t>(payload[offset + 3]));
        offset += 4;
        if (offset + size > payload.size()) {
            throw std::runtime_error("OSC bundle element size exceeds payload");
        }
        std::vector<std::uint8_t> element(payload.begin() + static_cast<std::ptrdiff_t>(offset),
                                          payload.begin() + static_cast<std::ptrdiff_t>(offset + size));
        offset += size;
        bundle.elements.emplace_back(decodeMessage(element));
    }

    return bundle;
}

Packet decodePacket(const std::vector<std::uint8_t>& payload) {
    if (payload.empty()) {
        throw std::runtime_error("OSC payload is empty");
    }
    if (payload.size() >= 7 && std::memcmp(payload.data(), "#bundle", 7) == 0) {
        return decodeBundle(payload);
    }
    return decodeMessage(payload);
}

}  // namespace acoustics::osc
