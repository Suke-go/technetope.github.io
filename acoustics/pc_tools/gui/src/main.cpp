#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "misc/cpp/imgui_stdlib.h"

#include "acoustics/common/DeviceRegistry.h"
#include "acoustics/osc/OscTransport.h"
#include "acoustics/scheduler/SoundTimeline.h"

#include "json.hpp"

#include <asio.hpp>
#include <asio/ip/address.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cctype>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <limits>
#include <variant>
#include <vector>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

constexpr std::chrono::milliseconds kRegistryRefreshInterval{500};
constexpr std::chrono::milliseconds kDiagnosticsRefreshInterval{1500};
constexpr std::chrono::minutes kSendStatsWindow{60};
constexpr int kSendStatsBuckets = 12;
constexpr std::chrono::seconds kMetricsFlushInterval{1};
constexpr double kLatencyWarningMs = 100.0;
constexpr double kLatencyCriticalMs = 250.0;
constexpr double kHeartbeatWarningSeconds = 3.0;
constexpr double kHeartbeatCriticalSeconds = 10.0;
constexpr std::size_t kMaxLogEntries = 300;
const fs::path kDefaultEventLogCsv{"logs/gui_event_log.csv"};
const fs::path kAuditLogPath{"logs/gui_audit.jsonl"};
const fs::path kMetricsLogPath{"logs/gui_dashboard_metrics.jsonl"};
const fs::path kDiagnosticsPath{"state/diagnostics.json"};
const fs::path kDiagnosticsNotesPath{"state/diagnostics_notes.json"};
constexpr std::chrono::seconds kMonitorStaleThreshold{5};
constexpr std::size_t kMonitorHistoryLimit = 64;

std::string formatIso8601(const std::chrono::system_clock::time_point& tp, bool includeDate = true);
std::string formatTimestamp(const std::chrono::system_clock::time_point& tp);
std::string base64Encode(const std::uint8_t* data, std::size_t len);
std::array<std::uint8_t, 20> sha1Digest(const std::uint8_t* data, std::size_t len);
std::string computeWebSocketAcceptKey(const std::string& clientKey);

class Sha1 {
public:
    void update(const std::uint8_t* data, std::size_t len) {
        bitLength_ += static_cast<std::uint64_t>(len) * 8ULL;
        while (len > 0) {
            const std::size_t toCopy = std::min<std::size_t>(64 - bufferSize_, len);
            std::memcpy(buffer_.data() + bufferSize_, data, toCopy);
            bufferSize_ += toCopy;
            data += toCopy;
            len -= toCopy;
            if (bufferSize_ == 64) {
                processBlock(buffer_.data());
                bufferSize_ = 0;
            }
        }
    }

    std::array<std::uint8_t, 20> finalize() {
        buffer_[bufferSize_++] = 0x80;
        if (bufferSize_ > 56) {
            while (bufferSize_ < 64) {
                buffer_[bufferSize_++] = 0x00;
            }
            processBlock(buffer_.data());
            bufferSize_ = 0;
        }
        while (bufferSize_ < 56) {
            buffer_[bufferSize_++] = 0x00;
        }
        for (int i = 7; i >= 0; --i) {
            buffer_[bufferSize_++] = static_cast<std::uint8_t>((bitLength_ >> (static_cast<std::uint64_t>(i) * 8ULL)) & 0xFFULL);
        }
        processBlock(buffer_.data());

        std::array<std::uint8_t, 20> digest{};
        for (int i = 0; i < 5; ++i) {
            digest[i * 4 + 0] = static_cast<std::uint8_t>((state_[i] >> 24) & 0xFF);
            digest[i * 4 + 1] = static_cast<std::uint8_t>((state_[i] >> 16) & 0xFF);
            digest[i * 4 + 2] = static_cast<std::uint8_t>((state_[i] >> 8) & 0xFF);
            digest[i * 4 + 3] = static_cast<std::uint8_t>((state_[i]) & 0xFF);
        }
        return digest;
    }

private:
    static std::uint32_t leftRotate(std::uint32_t value, std::uint32_t bits) {
        return (value << bits) | (value >> (32 - bits));
    }

    void processBlock(const std::uint8_t* block) {
        std::uint32_t w[80];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<std::uint32_t>(block[i * 4 + 0]) << 24) |
                   (static_cast<std::uint32_t>(block[i * 4 + 1]) << 16) |
                   (static_cast<std::uint32_t>(block[i * 4 + 2]) << 8) |
                   (static_cast<std::uint32_t>(block[i * 4 + 3]));
        }
        for (int i = 16; i < 80; ++i) {
            w[i] = leftRotate(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }

        std::uint32_t a = state_[0];
        std::uint32_t b = state_[1];
        std::uint32_t c = state_[2];
        std::uint32_t d = state_[3];
        std::uint32_t e = state_[4];

        for (int i = 0; i < 80; ++i) {
            std::uint32_t f = 0;
            std::uint32_t k = 0;
            if (i < 20) {
                f = (b & c) | ((~b) & d);
                k = 0x5A827999;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDC;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6;
            }
            std::uint32_t temp = leftRotate(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = leftRotate(b, 30);
            b = a;
            a = temp;
        }

        state_[0] += a;
        state_[1] += b;
        state_[2] += c;
        state_[3] += d;
        state_[4] += e;
    }

    std::array<std::uint8_t, 64> buffer_{};
    std::size_t bufferSize_{0};
    std::uint64_t bitLength_{0};
    std::array<std::uint32_t, 5> state_{{
        0x67452301u,
        0xEFCDAB89u,
        0x98BADCFEu,
        0x10325476u,
        0xC3D2E1F0u
    }};
};

std::array<std::uint8_t, 20> sha1Digest(const std::uint8_t* data, std::size_t len) {
    Sha1 sha;
    sha.update(data, len);
    return sha.finalize();
}

std::string base64Encode(const std::uint8_t* data, std::size_t len) {
    static constexpr char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    std::string encoded;
    encoded.reserve(((len + 2) / 3) * 4);
    std::size_t index = 0;
    while (index < len) {
        const std::size_t chunk = std::min<std::size_t>(3, len - index);
        std::uint32_t octetA = static_cast<std::uint32_t>(data[index++]);
        std::uint32_t octetB = chunk >= 2 ? static_cast<std::uint32_t>(data[index++]) : 0;
        std::uint32_t octetC = chunk == 3 ? static_cast<std::uint32_t>(data[index++]) : 0;

        std::uint32_t triple = (octetA << 16) | (octetB << 8) | octetC;
        encoded.push_back(kAlphabet[(triple >> 18) & 0x3F]);
        encoded.push_back(kAlphabet[(triple >> 12) & 0x3F]);
        encoded.push_back(chunk >= 2 ? kAlphabet[(triple >> 6) & 0x3F] : '=');
        encoded.push_back(chunk == 3 ? kAlphabet[triple & 0x3F] : '=');
    }
    return encoded;
}

std::string computeWebSocketAcceptKey(const std::string& clientKey) {
    static constexpr char kMagic[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    std::string merged = clientKey + kMagic;
    auto digest = sha1Digest(reinterpret_cast<const std::uint8_t*>(merged.data()), merged.size());
    return base64Encode(digest.data(), digest.size());
}

std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

struct MonitorEventDisplay;

std::string trimString(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

void pushMonitorHistory(std::deque<MonitorEventDisplay>& history,
                        const std::string& type,
                        const std::string& summary) {
    history.push_back(MonitorEventDisplay{
        std::chrono::system_clock::now(),
        type,
        summary
    });
    while (history.size() > kMonitorHistoryLimit) {
        history.pop_front();
    }
}

struct EventLogEntry {
    std::chrono::system_clock::time_point timestamp;
    spdlog::level::level_enum level{spdlog::level::info};
    std::string message;
};

struct SendLogSample {
    std::chrono::system_clock::time_point timestamp;
    bool success{false};
    std::string label;
    std::string detail;
};

class SendStatsTracker {
public:
    void record(bool success, std::string label, std::string detail) {
        samples_.push_back(SendLogSample{std::chrono::system_clock::now(), success, std::move(label), std::move(detail)});
        prune();
    }

    std::pair<int, int> lastHourCounts() const {
        prune();
        int ok = 0;
        int ng = 0;
        for (const auto& sample : samples_) {
            if (sample.success) {
                ++ok;
            } else {
                ++ng;
            }
        }
        return {ok, ng};
    }

    std::array<float, kSendStatsBuckets> bucketizedSuccessRates() const {
        prune();
        std::array<float, kSendStatsBuckets> ratios{};
        std::array<int, kSendStatsBuckets> totals{};
        auto now = std::chrono::system_clock::now();
        const auto windowStart = now - kSendStatsWindow;
        const double bucketDuration = std::chrono::duration<double>(kSendStatsWindow).count() / static_cast<double>(kSendStatsBuckets);
        if (bucketDuration <= 0.0) {
            return ratios;
        }
        for (const auto& sample : samples_) {
            if (sample.timestamp < windowStart) {
                continue;
            }
            const double secondsFromStart = std::chrono::duration<double>(sample.timestamp - windowStart).count();
            int bucket = static_cast<int>(secondsFromStart / bucketDuration);
            if (bucket < 0) {
                bucket = 0;
            }
            if (bucket >= kSendStatsBuckets) {
                bucket = kSendStatsBuckets - 1;
            }
            totals[bucket] += 1;
            if (sample.success) {
                ratios[bucket] += 1.0f;
            }
        }
        for (int i = 0; i < kSendStatsBuckets; ++i) {
            if (totals[i] > 0) {
                ratios[i] /= static_cast<float>(totals[i]);
            }
        }
        return ratios;
    }

private:
    void prune() const {
        const auto windowStart = std::chrono::system_clock::now() - kSendStatsWindow;
        while (!samples_.empty() && samples_.front().timestamp < windowStart) {
            samples_.pop_front();
        }
    }

    mutable std::deque<SendLogSample> samples_;
};

struct MonitorEvent {
    std::string type;
    json payload;
};


class MonitorEventQueue {
public:
    void push(MonitorEvent event) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(std::move(event));
    }

    bool pop(MonitorEvent& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        out = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

private:
    std::mutex mutex_;
    std::deque<MonitorEvent> queue_;
};

struct MonitorConnectionSnapshot {
    bool connected{false};
    bool connecting{false};
    std::string status;
    int attempt{0};
    std::chrono::system_clock::time_point lastEventAt{};
    std::chrono::system_clock::time_point lastStateChange{};
};

struct DiagnosticsEntry {
    std::string id;
    std::string deviceId;
    std::string severity;
    std::string reason;
    std::string relatedEventId;
    std::string recommendedAction;
    std::chrono::system_clock::time_point timestamp{};
};

class DiagnosticsNotesStore {
public:
    explicit DiagnosticsNotesStore(fs::path path) : path_(std::move(path)) {
        ensureParentExists();
        load();
    }

    std::string noteFor(const std::string& diagId) const {
        if (auto it = notes_.find(diagId); it != notes_.end()) {
            return it->second;
        }
        return {};
    }

    void setNote(const std::string& diagId, std::string note) {
        if (note.empty()) {
            notes_.erase(diagId);
        } else {
            notes_[diagId] = std::move(note);
        }
        save();
    }

    const fs::path& path() const noexcept { return path_; }

private:
    void ensureParentExists() {
        if (path_.has_parent_path()) {
            fs::create_directories(path_.parent_path());
        }
    }

    void load() {
        notes_.clear();
        if (!fs::exists(path_)) {
            return;
        }
        std::ifstream in(path_);
        if (!in) {
            spdlog::warn("Failed to open diagnostics notes: {}", path_.string());
            return;
        }
        try {
            json data;
            in >> data;
            if (data.is_object()) {
                for (auto it = data.begin(); it != data.end(); ++it) {
                    if (it.value().is_string()) {
                        notes_[it.key()] = it.value().get<std::string>();
                    }
                }
            }
        } catch (const std::exception& ex) {
            spdlog::error("Diagnostics notes parse error: {}", ex.what());
        }
    }

    void save() const {
        std::ofstream out(path_);
        if (!out) {
            spdlog::error("Failed to save diagnostics notes: {}", path_.string());
            return;
        }
        json data(json::value_t::object);
        for (const auto& [id, note] : notes_) {
            data[id] = note;
        }
        out << data.dump(2);
    }

    fs::path path_;
    std::unordered_map<std::string, std::string> notes_;
};

struct TimelinePreview {
    fs::path sourcePath;
    std::optional<acoustics::scheduler::SoundTimeline> timeline;
    std::chrono::system_clock::time_point baseTime{std::chrono::system_clock::now()};
    double leadSeconds{1.0};
    std::string lastError;

    bool ready() const noexcept { return timeline.has_value(); }
    void clearError() { lastError.clear(); }
};

struct DispatchOutcome {
    bool success{false};
    std::size_t targetCount{0};
    std::size_t bundleCount{0};
    std::size_t bundlesSucceeded{0};
    std::string detail;
};

struct SingleShotForm {
    int selectedDeviceIndex{-1};
    std::string preset{"test_ping"};
    float leadSeconds{0.5f};
    float gainDb{-3.0f};
    bool limitDuration{false};
    float maxDurationSeconds{5.0f};
    bool armed{false};
    bool dryRun{false};
};

struct DeviceWsStats {
    double lastLatencyMs{0.0};
    std::optional<int> queueDepth;
    std::optional<bool> isPlaying;
    std::chrono::system_clock::time_point lastHeartbeatAt{};
};

struct ParsedWsUrl {
    std::string host;
    std::string port;
    std::string path;
};

std::optional<ParsedWsUrl> parseWebSocketUrl(const std::string& url) {
    const std::string prefix = "ws://";
    if (url.rfind(prefix, 0) != 0) {
        return std::nullopt;
    }
    std::string remainder = url.substr(prefix.size());
    auto slashPos = remainder.find('/');
    std::string authority = slashPos == std::string::npos ? remainder : remainder.substr(0, slashPos);
    std::string path = slashPos == std::string::npos ? "/" : remainder.substr(slashPos);
    auto colonPos = authority.find(':');
    std::string host = colonPos == std::string::npos ? authority : authority.substr(0, colonPos);
    std::string port = colonPos == std::string::npos ? "80" : authority.substr(colonPos + 1);
    host = trimString(host);
    port = trimString(port);
    if (host.empty()) {
        return std::nullopt;
    }
    if (port.empty()) {
        port = "80";
    }
    if (path.empty()) {
        path = "/";
    }
    return ParsedWsUrl{host, port, path};
}

struct MonitorEventDisplay {
    std::chrono::system_clock::time_point timestamp{};
    std::string type;
    std::string summary;
};

class MonitorWebSocketClient {
public:
    using EventHandler = std::function<void(MonitorEvent&&)>;
    using StateHandler = std::function<void(const MonitorConnectionSnapshot&)>;
    using MetricsHandler = std::function<void(double, bool)>;

    MonitorWebSocketClient(EventHandler eventHandler,
                           StateHandler stateHandler,
                           MetricsHandler metricsHandler)
        : eventHandler_(std::move(eventHandler)),
          stateHandler_(std::move(stateHandler)),
          metricsHandler_(std::move(metricsHandler)) {}

    ~MonitorWebSocketClient() {
        stop();
    }

    void start(const std::string& url) {
        stop();
        std::lock_guard<std::mutex> lock(controlMutex_);
        targetUrl_ = url;
        shouldStop_.store(false);
        worker_ = std::thread(&MonitorWebSocketClient::run, this, targetUrl_);
    }

    void stop() {
        std::thread localThread;
        {
            std::lock_guard<std::mutex> lock(controlMutex_);
            shouldStop_.store(true);
            {
                std::lock_guard<std::mutex> socketLock(socketMutex_);
                if (activeSocket_) {
                    asio::error_code ec;
                    activeSocket_->close(ec);
                    activeSocket_.reset();
                }
            }
            if (worker_.joinable()) {
                localThread = std::move(worker_);
            }
        }
        if (localThread.joinable()) {
            localThread.join();
        }
        shouldStop_.store(false);
        running_.store(false);
    }

    bool isRunning() const {
        return running_.load();
    }

private:
    void run(std::string url);
    void publishState(const MonitorConnectionSnapshot& snapshot);
    bool performHandshake(asio::ip::tcp::socket& socket,
                          const ParsedWsUrl& parsed,
                          std::mt19937& rng);
    bool readTextMessage(asio::ip::tcp::socket& socket,
                         std::string& message,
                         std::mt19937& rng);
    void sendControlFrame(asio::ip::tcp::socket& socket,
                          std::uint8_t opcode,
                          const std::vector<std::uint8_t>& payload,
                          std::mt19937& rng);
    static std::string readHttpHeaders(asio::ip::tcp::socket& socket);
    static std::string getHeaderValue(const std::string& headers, const std::string& key);

    EventHandler eventHandler_;
    StateHandler stateHandler_;
    MetricsHandler metricsHandler_;
    std::thread worker_;
    std::atomic<bool> shouldStop_{false};
    std::atomic<bool> running_{false};
    std::mutex socketMutex_;
    std::shared_ptr<asio::ip::tcp::socket> activeSocket_;
    std::mutex controlMutex_;
    std::string targetUrl_;
};

void MonitorWebSocketClient::publishState(const MonitorConnectionSnapshot& snapshot) {
    if (stateHandler_) {
        stateHandler_(snapshot);
    }
}

std::string MonitorWebSocketClient::readHttpHeaders(asio::ip::tcp::socket& socket) {
    std::string headers;
    headers.reserve(1024);
    char ch = 0;
    while (true) {
        asio::error_code ec;
        asio::read(socket, asio::buffer(&ch, 1), ec);
        if (ec) {
            throw std::runtime_error(fmt::format("WebSocket header read error: {}", ec.message()));
        }
        headers.push_back(ch);
        if (headers.size() >= 4 &&
            headers.compare(headers.size() - 4, 4, "\r\n\r\n") == 0) {
            break;
        }
        if (headers.size() > 16 * 1024) {
            throw std::runtime_error("WebSocket header too large");
        }
    }
    return headers;
}

std::string MonitorWebSocketClient::getHeaderValue(const std::string& headers, const std::string& key) {
    std::istringstream stream(headers);
    std::string line;
    const std::string target = toLowerCopy(key);
    while (std::getline(stream, line)) {
        auto colon = line.find(':');
        if (colon == std::string::npos) {
            continue;
        }
        auto name = trimString(line.substr(0, colon));
        if (toLowerCopy(name) == target) {
            return trimString(line.substr(colon + 1));
        }
    }
    return {};
}

bool MonitorWebSocketClient::performHandshake(asio::ip::tcp::socket& socket,
                                              const ParsedWsUrl& parsed,
                                              std::mt19937& rng) {
    std::array<std::uint8_t, 16> nonce{};
    std::uniform_int_distribution<int> dist(0, 255);
    for (auto& byte : nonce) {
        byte = static_cast<std::uint8_t>(dist(rng));
    }
    std::string clientKey = base64Encode(nonce.data(), nonce.size());

    std::ostringstream request;
    request << "GET " << parsed.path << " HTTP/1.1\r\n"
            << "Host: " << parsed.host << ":" << parsed.port << "\r\n"
            << "Upgrade: websocket\r\n"
            << "Connection: Upgrade\r\n"
            << "Sec-WebSocket-Key: " << clientKey << "\r\n"
            << "Sec-WebSocket-Version: 13\r\n"
            << "\r\n";
    asio::write(socket, asio::buffer(request.str()));

    auto headers = readHttpHeaders(socket);
    if (!(headers.rfind("HTTP/1.1 101", 0) == 0 || headers.rfind("HTTP/1.0 101", 0) == 0)) {
        return false;
    }
    auto acceptHeader = getHeaderValue(headers, "sec-websocket-accept");
    auto expected = computeWebSocketAcceptKey(clientKey);
    return acceptHeader == expected;
}

void MonitorWebSocketClient::sendControlFrame(asio::ip::tcp::socket& socket,
                                              std::uint8_t opcode,
                                              const std::vector<std::uint8_t>& payload,
                                              std::mt19937& rng) {
    std::uniform_int_distribution<int> dist(0, 255);
    std::array<std::uint8_t, 4> mask{};
    for (auto& byte : mask) {
        byte = static_cast<std::uint8_t>(dist(rng));
    }
    std::vector<std::uint8_t> maskedPayload = payload;
    for (std::size_t i = 0; i < maskedPayload.size(); ++i) {
        maskedPayload[i] ^= mask[i % 4];
    }

    std::vector<std::uint8_t> frame;
    frame.reserve(2 + (maskedPayload.size() >= 126 ? 8 : 0) + 4 + maskedPayload.size());
    frame.push_back(static_cast<std::uint8_t>(0x80 | (opcode & 0x0F)));

    const std::size_t payloadLen = maskedPayload.size();
    if (payloadLen < 126) {
        frame.push_back(static_cast<std::uint8_t>(0x80 | payloadLen));
    } else if (payloadLen <= std::numeric_limits<std::uint16_t>::max()) {
        frame.push_back(0x80 | 126);
        frame.push_back(static_cast<std::uint8_t>((payloadLen >> 8) & 0xFF));
        frame.push_back(static_cast<std::uint8_t>(payloadLen & 0xFF));
    } else {
        frame.push_back(0x80 | 127);
        for (int i = 7; i >= 0; --i) {
            frame.push_back(static_cast<std::uint8_t>((static_cast<std::uint64_t>(payloadLen) >> (i * 8)) & 0xFF));
        }
    }
    frame.insert(frame.end(), mask.begin(), mask.end());
    frame.insert(frame.end(), maskedPayload.begin(), maskedPayload.end());

    asio::error_code ec;
    asio::write(socket, asio::buffer(frame), ec);
}

bool MonitorWebSocketClient::readTextMessage(asio::ip::tcp::socket& socket,
                                             std::string& message,
                                             std::mt19937& rng) {
    std::array<std::uint8_t, 2> header{};
    asio::error_code ec;
    asio::read(socket, asio::buffer(header), ec);
    if (ec) {
        return false;
    }

    const bool fin = (header[0] & 0x80u) != 0;
    const std::uint8_t opcode = header[0] & 0x0Fu;
    const bool masked = (header[1] & 0x80u) != 0;
    std::uint64_t payloadLen = header[1] & 0x7Fu;

    if (payloadLen == 126) {
        std::array<std::uint8_t, 2> extended{};
        asio::read(socket, asio::buffer(extended), ec);
        if (ec) {
            return false;
        }
        payloadLen = (static_cast<std::uint64_t>(extended[0]) << 8) | extended[1];
    } else if (payloadLen == 127) {
        std::array<std::uint8_t, 8> extended{};
        asio::read(socket, asio::buffer(extended), ec);
        if (ec) {
            return false;
        }
        payloadLen = 0;
        for (int i = 0; i < 8; ++i) {
            payloadLen = (payloadLen << 8) | extended[i];
        }
    }

    if (payloadLen > 4 * 1024 * 1024) {
        throw std::runtime_error("WebSocket payload too large");
    }

    std::array<std::uint8_t, 4> mask{};
    if (masked) {
        asio::read(socket, asio::buffer(mask), ec);
        if (ec) {
            return false;
        }
    }

    std::vector<std::uint8_t> payload(payloadLen);
    if (payloadLen > 0) {
        asio::read(socket, asio::buffer(payload.data(), payload.size()), ec);
        if (ec) {
            return false;
        }
    }

    if (masked) {
        for (std::size_t i = 0; i < payload.size(); ++i) {
            payload[i] ^= mask[i % 4];
        }
    }

    if (opcode == 0x8) {  // close
        sendControlFrame(socket, 0x8, {}, rng);
        return false;
    }

    if (opcode == 0x9) {  // ping
        sendControlFrame(socket, 0xA, payload, rng);
        return true;
    }

    if (opcode == 0xA) {  // pong
        return true;
    }

    if (opcode != 0x1 || !fin) {
        return true;
    }

    message.assign(payload.begin(), payload.end());
    return true;
}

void MonitorWebSocketClient::run(std::string url) {
    running_.store(true);
    auto parsed = parseWebSocketUrl(url);
    if (!parsed) {
        MonitorConnectionSnapshot snapshot;
        snapshot.connected = false;
        snapshot.connecting = false;
        snapshot.status = "Invalid WebSocket URL";
        snapshot.lastStateChange = std::chrono::system_clock::now();
        publishState(snapshot);
        running_.store(false);
        return;
    }

    std::mt19937 rng(std::random_device{}());
    int attempt = 0;
    while (!shouldStop_.load()) {
        ++attempt;
        MonitorConnectionSnapshot connecting;
        connecting.connected = false;
        connecting.connecting = true;
        connecting.status = fmt::format("Connecting to ws://{}:{}{}", parsed->host, parsed->port, parsed->path);
        connecting.attempt = attempt;
        connecting.lastStateChange = std::chrono::system_clock::now();
        publishState(connecting);

        auto attemptStart = std::chrono::steady_clock::now();
        bool success = false;

        try {
            asio::io_context ioContext;
            asio::ip::tcp::resolver resolver(ioContext);
            auto endpoints = resolver.resolve(parsed->host, parsed->port);
            auto socket = std::make_shared<asio::ip::tcp::socket>(ioContext);
            {
                std::lock_guard<std::mutex> lock(socketMutex_);
                activeSocket_ = socket;
            }
            asio::connect(*socket, endpoints);
            if (!performHandshake(*socket, *parsed, rng)) {
                throw std::runtime_error("WebSocket handshake failed");
            }
            success = true;

            MonitorConnectionSnapshot connected;
            connected.connected = true;
            connected.connecting = false;
            connected.status = "Connected";
            connected.attempt = attempt;
            connected.lastStateChange = std::chrono::system_clock::now();
            publishState(connected);

            std::string message;
            while (!shouldStop_.load()) {
                if (!readTextMessage(*socket, message, rng)) {
                    break;
                }
                if (!eventHandler_) {
                    continue;
                }
                try {
                    json parsedJson = json::parse(message);
                    MonitorEvent event;
                    if (parsedJson.contains("type") && parsedJson["type"].is_string()) {
                        event.type = parsedJson["type"].get<std::string>();
                        parsedJson.erase("type");
                    } else {
                        event.type = "raw";
                    }
                    event.payload = std::move(parsedJson);
                    eventHandler_(std::move(event));
                } catch (const std::exception& ex) {
                    spdlog::warn("Monitor WS JSON parse error: {}", ex.what());
                }
            }
        } catch (const std::exception& ex) {
            MonitorConnectionSnapshot error;
            error.connected = false;
            error.connecting = false;
            error.status = fmt::format("Error: {}", ex.what());
            error.attempt = attempt;
            error.lastStateChange = std::chrono::system_clock::now();
            publishState(error);
        }

        double durationMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - attemptStart).count();
        if (metricsHandler_) {
            metricsHandler_(durationMs, success);
        }

        {
            std::lock_guard<std::mutex> lock(socketMutex_);
            if (activeSocket_) {
                asio::error_code ec;
                activeSocket_->close(ec);
                activeSocket_.reset();
            }
        }

        if (shouldStop_.load()) {
            break;
        }

        auto backoffSeconds = std::min<std::chrono::seconds>(
            std::chrono::seconds(1 << std::min(attempt, 3)),
            std::chrono::seconds(8));
        std::this_thread::sleep_for(backoffSeconds);
    }

    running_.store(false);
    MonitorConnectionSnapshot finalState;
    finalState.connected = false;
    finalState.connecting = false;
    finalState.status = shouldStop_.load() ? "Stopped" : "Idle";
    finalState.lastStateChange = std::chrono::system_clock::now();
    publishState(finalState);
}

class MetricsLogger {
public:
    explicit MetricsLogger(fs::path path)
        : path_(std::move(path)),
          lastFlush_(std::chrono::steady_clock::now()) {
        if (path_.has_parent_path()) {
            fs::create_directories(path_.parent_path());
        }
    }

    void recordFrame(double ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        frameSamples_.push_back(ms);
    }

    void recordTimelineSend(double durationMs, bool success) {
        std::lock_guard<std::mutex> lock(mutex_);
        lastTimelineMs_ = durationMs;
        lastTimelineSuccess_ = success;
        lastTimelineTimestamp_ = std::chrono::system_clock::now();
    }

    void recordMonitorReconnect(double durationMs, bool success) {
        std::lock_guard<std::mutex> lock(mutex_);
        lastMonitorReconnectMs_ = durationMs;
        lastMonitorReconnectSuccess_ = success;
        lastMonitorReconnectTimestamp_ = std::chrono::system_clock::now();
    }

    void flushIfNeeded() {
        auto now = std::chrono::steady_clock::now();
        if (now - lastFlush_ < kMetricsFlushInterval) {
            return;
        }

        std::vector<double> frameSamplesCopy;
        std::optional<double> timelineDuration;
        bool timelineSuccess = false;
        std::chrono::system_clock::time_point timelineTimestamp{};
        std::optional<double> monitorReconnectDuration;
        bool monitorReconnectSuccess = false;
        std::chrono::system_clock::time_point monitorReconnectTimestamp{};

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (frameSamples_.empty() && !lastTimelineMs_ && !lastMonitorReconnectMs_) {
                return;
            }
            lastFlush_ = now;
            frameSamplesCopy.swap(frameSamples_);
            if (lastTimelineMs_) {
                timelineDuration = *lastTimelineMs_;
                timelineSuccess = lastTimelineSuccess_;
                timelineTimestamp = lastTimelineTimestamp_;
                lastTimelineMs_.reset();
            }
            if (lastMonitorReconnectMs_) {
                monitorReconnectDuration = *lastMonitorReconnectMs_;
                monitorReconnectSuccess = lastMonitorReconnectSuccess_;
                monitorReconnectTimestamp = lastMonitorReconnectTimestamp_;
                lastMonitorReconnectMs_.reset();
            }
        }

        json entry{
            {"timestamp", formatIso8601(std::chrono::system_clock::now())}
        };

        if (!frameSamplesCopy.empty()) {
            double sum = 0.0;
            double max = 0.0;
            std::vector<double> sorted = frameSamplesCopy;
            std::sort(sorted.begin(), sorted.end());
            for (double sample : frameSamplesCopy) {
                sum += sample;
                if (sample > max) {
                    max = sample;
                }
            }
            double avg = sum / static_cast<double>(frameSamplesCopy.size());
            std::size_t idx = sorted.size() - 1;
            if (sorted.size() > 1) {
                double rank = 0.95 * static_cast<double>(sorted.size() - 1);
                idx = static_cast<std::size_t>(std::round(rank));
            }
            double p95 = sorted[idx];
            entry["frame_time"] = {
                {"count", frameSamplesCopy.size()},
                {"avg_ms", avg},
                {"max_ms", max},
                {"p95_ms", p95}
            };
        }

        if (timelineDuration) {
            entry["timeline_send"] = {
                {"duration_ms", *timelineDuration},
                {"success", timelineSuccess},
                {"recorded_at", formatIso8601(timelineTimestamp)}
            };
        }

        if (monitorReconnectDuration) {
            entry["monitor_reconnect"] = {
                {"duration_ms", *monitorReconnectDuration},
                {"success", monitorReconnectSuccess},
                {"recorded_at", formatIso8601(monitorReconnectTimestamp)}
            };
        }

        std::ofstream out(path_, std::ios::app);
        if (!out) {
            spdlog::warn("Failed to write metrics log: {}", path_.string());
            return;
        }
        out << entry.dump() << '\n';
    }

private:
    fs::path path_;
    std::vector<double> frameSamples_;
    std::optional<double> lastTimelineMs_;
    bool lastTimelineSuccess_{false};
    std::chrono::system_clock::time_point lastTimelineTimestamp_{};
    std::optional<double> lastMonitorReconnectMs_;
    bool lastMonitorReconnectSuccess_{false};
    std::chrono::system_clock::time_point lastMonitorReconnectTimestamp_{};
    std::chrono::steady_clock::time_point lastFlush_;
    std::mutex mutex_;
};

void trimLog(std::deque<EventLogEntry>& log);

enum class DeviceHealth {
    Ok,
    Warning,
    Critical
};

struct DeviceSummary {
    acoustics::common::DeviceSnapshot snapshot;
    std::string alias;
    double meanLatency{0.0};
    double stdLatency{0.0};
    double secondsSinceSeen{0.0};
    DeviceHealth health{DeviceHealth::Critical};
};

class AliasStore {
public:
    explicit AliasStore(fs::path path) : path_(std::move(path)) {
        ensureParentExists();
        load();
    }

    std::string aliasFor(const std::string& deviceId) const {
        if (auto it = aliases_.find(deviceId); it != aliases_.end()) {
            return it->second;
        }
        return {};
    }

    void setAlias(const std::string& deviceId, const std::string& alias) {
        if (alias.empty()) {
            aliases_.erase(deviceId);
        } else {
            aliases_[deviceId] = alias;
        }
        save();
    }

    const fs::path& path() const noexcept { return path_; }

private:
    fs::path path_;
    std::unordered_map<std::string, std::string> aliases_;

    void ensureParentExists() {
        if (path_.has_parent_path()) {
            fs::create_directories(path_.parent_path());
        }
    }

    void load() {
        aliases_.clear();
        if (!fs::exists(path_)) {
            return;
        }
        std::ifstream in(path_);
        if (!in) {
            spdlog::warn("Failed to open alias store: {}", path_.string());
            return;
        }
        json data;
        try {
            in >> data;
            if (data.is_object()) {
                for (auto it = data.begin(); it != data.end(); ++it) {
                    if (it.value().is_string()) {
                        aliases_[it.key()] = it.value().get<std::string>();
                    }
                }
            }
        } catch (const std::exception& ex) {
            spdlog::error("Alias store parse error: {}", ex.what());
        }
    }

    void save() const {
        std::ofstream out(path_);
        if (!out) {
            spdlog::error("Failed to write alias store: {}", path_.string());
            return;
        }
        json data(json::value_t::object);
        for (const auto& [id, alias] : aliases_) {
            data[id] = alias;
        }
        out << data.dump(2);
    }
};

struct OscConfig {
    std::string host{"192.168.2.255"};
    int port{9000};
    bool broadcast{true};
};

class OscController {
public:
    OscController()
        : ioContext_(),
          endpoint_(asio::ip::make_address("192.168.2.255"), 9000),
          sender_(ioContext_, endpoint_, true) {}

    void updateConfig(const OscConfig& cfg, std::deque<EventLogEntry>& log) {
        try {
            asio::ip::address address = asio::ip::make_address(cfg.host);
            endpoint_ = acoustics::osc::OscSender::Endpoint(address, static_cast<unsigned short>(cfg.port));
            sender_.setEndpoint(endpoint_);
            sender_.setBroadcastEnabled(cfg.broadcast);
            log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::info,
                                           fmt::format("OSC endpoint set to {}:{} (broadcast={})", cfg.host, cfg.port, cfg.broadcast)});
        } catch (const std::exception& ex) {
            log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::err,
                                           fmt::format("Failed to apply OSC endpoint: {}", ex.what())});
        }
        trimLog(log);
    }

    bool sendMessage(const acoustics::osc::Message& msg, std::deque<EventLogEntry>& log) {
        try {
            sender_.send(msg);
            return true;
        } catch (const std::exception& ex) {
            log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::err,
                                           fmt::format("OSC send failed: {}", ex.what())});
            trimLog(log);
            return false;
        }
    }

    bool sendBundle(const acoustics::osc::Bundle& bundle, std::deque<EventLogEntry>& log) {
        try {
            sender_.send(bundle);
            return true;
        } catch (const std::exception& ex) {
            log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::err,
                                           fmt::format("OSC bundle send failed: {}", ex.what())});
            trimLog(log);
            return false;
        }
    }

private:
    asio::io_context ioContext_;
    acoustics::osc::OscSender::Endpoint endpoint_;
    acoustics::osc::OscSender sender_;
};

std::string formatTimestamp(const std::chrono::system_clock::time_point& tp) {
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%H:%M:%S", &tm);
    return std::string(buffer);
}

DeviceHealth classifyHealth(double secondsSinceSeen, double meanLatency) {
    if (secondsSinceSeen > kHeartbeatCriticalSeconds) {
        return DeviceHealth::Critical;
    }
    if (secondsSinceSeen > kHeartbeatWarningSeconds) {
        return DeviceHealth::Warning;
    }
    if (meanLatency > kLatencyCriticalMs) {
        return DeviceHealth::Critical;
    }
    if (meanLatency > kLatencyWarningMs) {
        return DeviceHealth::Warning;
    }
    return DeviceHealth::Ok;
}

ImU32 colorForHealth(DeviceHealth health) {
    switch (health) {
        case DeviceHealth::Ok:
            return IM_COL32(76, 217, 100, 255);
        case DeviceHealth::Warning:
            return IM_COL32(255, 204, 0, 255);
        case DeviceHealth::Critical:
        default:
            return IM_COL32(255, 59, 48, 255);
    }
}

std::vector<DeviceSummary> buildDeviceSummaries(
    acoustics::common::DeviceRegistry& registry,
    AliasStore& aliases,
    std::chrono::steady_clock::time_point& lastRefresh,
    std::chrono::steady_clock::time_point now) {

    if (now - lastRefresh < kRegistryRefreshInterval) {
        return {};
    }
    lastRefresh = now;

    registry.load();
    auto snapshots = registry.snapshot();
    std::vector<DeviceSummary> result;
    result.reserve(snapshots.size());
    for (auto& snap : snapshots) {
        DeviceSummary summary;
        summary.snapshot = snap;
        summary.alias = aliases.aliasFor(snap.state.id);
        const auto& heartbeat = snap.state.heartbeat;
        summary.meanLatency = heartbeat.count > 0 ? heartbeat.meanLatencyMs : 0.0;
        summary.stdLatency = heartbeat.standardDeviation();
        summary.secondsSinceSeen = std::chrono::duration<double>(snap.snapshotTime - snap.state.lastSeen).count();
        summary.health = classifyHealth(summary.secondsSinceSeen, summary.meanLatency);
        result.push_back(std::move(summary));
    }
    std::sort(result.begin(), result.end(), [](const DeviceSummary& a, const DeviceSummary& b) {
        return a.snapshot.state.id < b.snapshot.state.id;
    });
    return result;
}

std::optional<std::chrono::system_clock::time_point> parseIso8601(const std::string& value) {
    if (value.empty()) {
        return std::nullopt;
    }
    std::string copy = value;
    if (!copy.empty() && copy.back() == 'Z') {
        copy.pop_back();
    }
    auto dotPos = copy.find('.');
    std::string fractional;
    if (dotPos != std::string::npos) {
        fractional = copy.substr(dotPos + 1);
        copy = copy.substr(0, dotPos);
    }
    std::tm tm{};
    std::istringstream iss(copy);
    iss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
    if (iss.fail()) {
        return std::nullopt;
    }
    auto timeT =
#if defined(_WIN32)
        _mkgmtime(&tm);
#else
        timegm(&tm);
#endif
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(timeT);
    if (!fractional.empty()) {
        double fraction = std::stod("0." + fractional);
        tp += std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::duration<double>(fraction));
    }
    return tp;
}

std::string formatIso8601(const std::chrono::system_clock::time_point& tp, bool includeDate) {
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &tt);
#else
    gmtime_r(&tt, &tm);
#endif
    char buffer[64];
    if (includeDate) {
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
    } else {
        std::strftime(buffer, sizeof(buffer), "%H:%M:%S", &tm);
    }
    return std::string(buffer);
}

std::vector<DiagnosticsEntry> loadDiagnosticsEntries(const fs::path& path) {
    std::vector<DiagnosticsEntry> entries;
    if (!fs::exists(path)) {
        return entries;
    }
    std::ifstream in(path);
    if (!in) {
        spdlog::warn("Failed to open diagnostics file: {}", path.string());
        return entries;
    }
    try {
        json data;
        in >> data;
        auto parseEntry = [](const json& obj, std::size_t index) -> std::optional<DiagnosticsEntry> {
            if (!obj.is_object()) {
                return std::nullopt;
            }
            DiagnosticsEntry entry;
            if (obj.contains("id") && obj["id"].is_string()) {
                entry.id = obj["id"].get<std::string>();
            } else {
                entry.id = fmt::format("diag-{}", index);
            }
            if (obj.contains("device_id") && obj["device_id"].is_string()) {
                entry.deviceId = obj["device_id"].get<std::string>();
            }
            if (obj.contains("severity") && obj["severity"].is_string()) {
                entry.severity = obj["severity"].get<std::string>();
            }
            if (obj.contains("reason") && obj["reason"].is_string()) {
                entry.reason = obj["reason"].get<std::string>();
            } else if (obj.contains("message") && obj["message"].is_string()) {
                entry.reason = obj["message"].get<std::string>();
            }
            if (obj.contains("related_event_id") && obj["related_event_id"].is_string()) {
                entry.relatedEventId = obj["related_event_id"].get<std::string>();
            }
            if (obj.contains("recommended_action") && obj["recommended_action"].is_string()) {
                entry.recommendedAction = obj["recommended_action"].get<std::string>();
            }
            std::optional<std::chrono::system_clock::time_point> timestamp;
            if (obj.contains("timestamp") && obj["timestamp"].is_string()) {
                timestamp = parseIso8601(obj["timestamp"].get<std::string>());
            } else if (obj.contains("time_utc") && obj["time_utc"].is_string()) {
                timestamp = parseIso8601(obj["time_utc"].get<std::string>());
            }
            entry.timestamp = timestamp.value_or(std::chrono::system_clock::now());
            return entry;
        };

        if (data.is_array()) {
            for (std::size_t i = 0; i < data.size(); ++i) {
                if (auto entry = parseEntry(data[i], i)) {
                    entries.push_back(*entry);
                }
            }
        } else if (data.is_object() && data.contains("entries") && data["entries"].is_array()) {
            const auto& arr = data["entries"];
            for (std::size_t i = 0; i < arr.size(); ++i) {
                if (auto entry = parseEntry(arr[i], i)) {
                    entries.push_back(*entry);
                }
            }
        }
        std::sort(entries.begin(), entries.end(), [](const DiagnosticsEntry& a, const DiagnosticsEntry& b) {
            return a.timestamp > b.timestamp;
        });
    } catch (const std::exception& ex) {
        spdlog::error("Diagnostics parse error: {}", ex.what());
    }
    return entries;
}

std::optional<acoustics::scheduler::SoundTimeline> tryLoadTimeline(const fs::path& path, std::string& errorOut) {
    try {
        auto timeline = acoustics::scheduler::SoundTimeline::fromJsonFile(path);
        errorOut.clear();
        return timeline;
    } catch (const std::exception& ex) {
        errorOut = ex.what();
        spdlog::error("Timeline preview failed: {}", ex.what());
        return std::nullopt;
    }
}

void trimLog(std::deque<EventLogEntry>& log) {
    while (log.size() > kMaxLogEntries) {
        log.pop_front();
    }
}

std::string displayAlias(const DeviceSummary& summary) {
    if (!summary.alias.empty()) {
        return summary.alias;
    }
    return summary.snapshot.state.id;
}

std::string describeTargets(const std::vector<std::string>& targets) {
    if (targets.empty()) {
        return "broadcast";
    }
    if (targets.size() == 1) {
        return targets.front();
    }
    std::ostringstream oss;
    oss << targets.front() << " +" << (targets.size() - 1);
    return oss.str();
}

std::string extractPreset(const acoustics::scheduler::TimelineEvent& event) {
    for (const auto& arg : event.arguments) {
        if (std::holds_alternative<std::string>(arg)) {
            return std::get<std::string>(arg);
        }
    }
    return event.address;
}

std::string currentOperator() {
    if (const char* user = std::getenv("USER")) {
        return user;
    }
#if defined(_WIN32)
    if (const char* username = std::getenv("USERNAME")) {
        return username;
    }
#endif
    return "operator";
}

ImVec4 severityColor(const std::string& severity) {
    std::string lower = severity;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (lower == "critical" || lower == "high") {
        return ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
    }
    if (lower == "medium" || lower == "warn") {
        return ImVec4(1.0f, 0.7f, 0.2f, 1.0f);
    }
    return ImVec4(0.6f, 0.9f, 0.6f, 1.0f);
}

void appendAuditRecord(const std::string& action,
                       const std::string& target,
                       const std::string& preset,
                       bool success,
                       const std::string& detail) {
    fs::create_directories(kAuditLogPath.parent_path());
    std::ofstream out(kAuditLogPath, std::ios::app);
    if (!out) {
        spdlog::warn("Failed to write audit log: {}", kAuditLogPath.string());
        return;
    }
    json record{
        {"timestamp", formatIso8601(std::chrono::system_clock::now())},
        {"operator", currentOperator()},
        {"action", action},
        {"target", target},
        {"preset", preset},
        {"success", success},
        {"detail", detail}
    };
    out << record.dump() << '\n';
}

void sendTestSignal(OscController& osc,
                    const std::string& preset,
                    const DeviceSummary& device,
                    double leadSeconds,
                    std::deque<EventLogEntry>& log,
                    SendStatsTracker& stats) {
    acoustics::osc::Message msg;
    msg.address = "/acoustics/play";
    msg.arguments.emplace_back(preset);
    auto offsetMs = static_cast<std::int32_t>(leadSeconds * 1000.0);
    msg.arguments.emplace_back(offsetMs);
    msg.arguments.emplace_back(static_cast<float>(1.0f));
    msg.arguments.emplace_back(static_cast<std::int32_t>(0));

    const bool success = osc.sendMessage(msg, log);
    if (success) {
        log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::info,
                                       fmt::format("Test signal '{}' sent to {}", preset, device.snapshot.state.id)});
    } else {
        log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::err,
                                       fmt::format("Test signal '{}' failed for {}", preset, device.snapshot.state.id)});
    }
    trimLog(log);
    stats.record(success, fmt::format("test:{}", device.snapshot.state.id), preset);
    appendAuditRecord("test_signal",
                      device.snapshot.state.id,
                      preset,
                      success,
                      success ? "dispatch ok" : "dispatch failed");
}

DispatchOutcome sendTimelineToDevices(const std::vector<DeviceSummary>& devices,
                                      const std::set<std::string>& selected,
                                      const fs::path& timelinePath,
                                      double leadSeconds,
                                      bool baseNow,
                                      const std::string& baseTimeString,
                                      OscController& osc,
                                      std::deque<EventLogEntry>& log,
                                      SendStatsTracker& stats,
                                      MetricsLogger& metrics) {
    DispatchOutcome outcome;
    outcome.detail = "no-op";
    auto dispatchStart = std::chrono::steady_clock::now();
    if (!fs::exists(timelinePath)) {
        log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::err,
                                       fmt::format("Timeline file not found: {}", timelinePath.string())});
        trimLog(log);
        stats.record(false, "timeline", "missing file");
        appendAuditRecord("timeline_send", "none", timelinePath.string(), false, "timeline file missing");
        auto durationMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - dispatchStart).count();
        metrics.recordTimelineSend(durationMs, false);
        return outcome;
    }

    std::vector<const DeviceSummary*> targets;
    if (selected.empty()) {
        for (const auto& dev : devices) {
            targets.push_back(&dev);
        }
    } else {
        for (const auto& dev : devices) {
            if (selected.count(dev.snapshot.state.id)) {
                targets.push_back(&dev);
            }
        }
    }

    if (targets.empty()) {
        log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::warn,
                                       "No devices selected for timeline send."});
        trimLog(log);
        stats.record(false, "timeline", "no targets");
        appendAuditRecord("timeline_send", "none", timelinePath.filename().string(), false, "no targets selected");
        auto durationMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - dispatchStart).count();
        metrics.recordTimelineSend(durationMs, false);
        return outcome;
    }
    outcome.targetCount = targets.size();

    try {
        auto timeline = acoustics::scheduler::SoundTimeline::fromJsonFile(timelinePath);
        std::chrono::system_clock::time_point baseTime = std::chrono::system_clock::now();
        if (!baseNow) {
            if (auto parsed = parseIso8601(baseTimeString)) {
                baseTime = *parsed;
            } else {
                log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::warn,
                                               "Failed to parse base time. Using now."});
            }
        }
        auto bundles = timeline.toBundles(baseTime, leadSeconds);
        outcome.bundleCount = bundles.size();
        for (auto bundle : bundles) {
            if (osc.sendBundle(bundle, log)) {
                ++outcome.bundlesSucceeded;
            }
        }
        outcome.success = outcome.bundleCount > 0 && outcome.bundlesSucceeded == outcome.bundleCount;
        outcome.detail = fmt::format("targets=%zu bundles=%zu success=%zu",
                                     outcome.targetCount,
                                     outcome.bundleCount,
                                     outcome.bundlesSucceeded);
        auto level = outcome.success ? spdlog::level::info : spdlog::level::warn;
        log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), level,
                                       fmt::format("Timeline '{}' dispatched (%s)",
                                                   timelinePath.filename().string(),
                                                   outcome.detail)});
        trimLog(log);
        stats.record(outcome.success,
                     fmt::format("timeline:{}", timelinePath.filename().string()),
                     outcome.detail);
        appendAuditRecord("timeline_send",
                          fmt::format("%zu target(s)", outcome.targetCount),
                          timelinePath.filename().string(),
                          outcome.success,
                          outcome.detail);
    } catch (const std::exception& ex) {
        log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::err,
                                       fmt::format("Timeline send failed: {}", ex.what())});
        trimLog(log);
        outcome.detail = ex.what();
        stats.record(false,
                     fmt::format("timeline:{}", timelinePath.filename().string()),
                     outcome.detail);
        appendAuditRecord("timeline_send",
                          fmt::format("%zu target(s)", outcome.targetCount),
                          timelinePath.filename().string(),
                          false,
                          outcome.detail);
    }
    auto dispatchDuration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - dispatchStart).count();
    metrics.recordTimelineSend(dispatchDuration, outcome.success);
    return outcome;
}

bool sendSingleShot(OscController& osc,
                    const DeviceSummary& device,
                    const SingleShotForm& form,
                    std::deque<EventLogEntry>& log,
                    SendStatsTracker& stats) {
    acoustics::osc::Message msg;
    msg.address = "/acoustics/play";
    msg.arguments.emplace_back(form.preset);
    auto leadMs = static_cast<std::int32_t>(form.leadSeconds * 1000.0f);
    msg.arguments.emplace_back(leadMs);
    const float gainScalar = std::clamp(std::pow(10.0f, form.gainDb / 20.0f), 0.0f, 2.0f);
    msg.arguments.emplace_back(gainScalar);
    auto maxDuration = static_cast<std::int32_t>(form.limitDuration ? form.maxDurationSeconds * 1000.0f : 0.0f);
    msg.arguments.emplace_back(maxDuration);

    bool success = false;
    if (form.dryRun) {
        success = true;
        log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::info,
                                       fmt::format("[DRY-RUN] Single shot '{}' would target {}", form.preset,
                                                   displayAlias(device))});
    } else {
        success = osc.sendMessage(msg, log);
        if (success) {
            log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::info,
                                           fmt::format("Single shot '{}' sent to {}", form.preset,
                                                       displayAlias(device))});
        } else {
            log.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::err,
                                           fmt::format("Single shot '{}' failed for {}", form.preset,
                                                       displayAlias(device))});
        }
    }
    trimLog(log);
    stats.record(success,
                 fmt::format("singleshot:{}", device.snapshot.state.id),
                 form.dryRun ? "dry-run" : "dispatch");
    appendAuditRecord(form.dryRun ? "single_shot_dry_run" : "single_shot_fire",
                      device.snapshot.state.id,
                      form.preset,
                      success,
                      form.dryRun ? "dry-run only" : "dispatch attempted");
    return success;
}

void handleMonitorEvent(const MonitorEvent& event,
                        std::unordered_map<std::string, DeviceWsStats>& telemetry,
                        std::vector<DiagnosticsEntry>& diagnostics,
                        std::deque<EventLogEntry>& log,
                        std::deque<MonitorEventDisplay>& history) {
    const auto now = std::chrono::system_clock::now();
    if (event.type == "heartbeat") {
        const std::string deviceId = event.payload.value("device_id", std::string{});
        if (deviceId.empty()) {
            return;
        }
        auto& stats = telemetry[deviceId];
        stats.lastLatencyMs = event.payload.value("latency_ms", 0.0);
        if (event.payload.contains("queue_depth") && event.payload["queue_depth"].is_number_integer()) {
            stats.queueDepth = event.payload["queue_depth"].get<int>();
        } else {
            stats.queueDepth.reset();
        }
        if (event.payload.contains("is_playing")) {
            if (event.payload["is_playing"].is_boolean()) {
                stats.isPlaying = event.payload["is_playing"].get<bool>();
            } else if (event.payload["is_playing"].is_number_integer()) {
                stats.isPlaying = (event.payload["is_playing"].get<int>() != 0);
            } else {
                stats.isPlaying.reset();
            }
        }
        stats.lastHeartbeatAt = now;
        log.emplace_back(EventLogEntry{now,
                                       spdlog::level::info,
                                       fmt::format("Heartbeat {} latency={:.1f} ms queue={}",
                                                   deviceId,
                                                   stats.lastLatencyMs,
                                                   stats.queueDepth ? std::to_string(*stats.queueDepth) : "-")});
        trimLog(log);
        pushMonitorHistory(history,
                           "heartbeat",
                           fmt::format("{} {:.1f} ms", deviceId, stats.lastLatencyMs));
        return;
    }

    if (event.type == "diagnostics") {
        DiagnosticsEntry entry;
        entry.id = event.payload.value("id", fmt::format("diag-{}", diagnostics.size() + 1));
        entry.deviceId = event.payload.value("device_id", std::string{});
        entry.severity = event.payload.value("severity", std::string{"warn"});
        entry.reason = event.payload.value("reason", std::string{});
        entry.relatedEventId = event.payload.value("related_event_id", std::string{});
        entry.recommendedAction = event.payload.value("recommended_action", std::string{});
        if (event.payload.contains("timestamp") && event.payload["timestamp"].is_string()) {
            if (auto parsed = parseIso8601(event.payload["timestamp"].get<std::string>())) {
                entry.timestamp = *parsed;
            } else {
                entry.timestamp = now;
            }
        } else {
            entry.timestamp = now;
        }
        diagnostics.insert(diagnostics.begin(), entry);
        if (diagnostics.size() > 200) {
            diagnostics.pop_back();
        }

        log.emplace_back(EventLogEntry{now,
                                       spdlog::level::warn,
                                       fmt::format("Diagnostics {} severity={} reason={}",
                                                   entry.deviceId.empty() ? "(unknown)" : entry.deviceId,
                                                   entry.severity,
                                                   entry.reason)});
        trimLog(log);
        pushMonitorHistory(history,
                           "diagnostics",
                           fmt::format("{} {}", entry.deviceId, entry.reason));
        return;
    }

    pushMonitorHistory(history, event.type, event.payload.dump());
}

} // namespace

int main() {
    if (!glfwInit()) {
        spdlog::error("Failed to initialize GLFW");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#if defined(__APPLE__)
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 720, "Acoustics Monitor", nullptr, nullptr);
    if (!window) {
        spdlog::error("Failed to create GLFW window");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    fs::path stateDir = fs::path("state");
    fs::create_directories(stateDir);
    fs::create_directories("logs");

    fs::path devicesPath = stateDir / "devices.json";
    fs::path aliasPath = stateDir / "device_aliases.json";

    acoustics::common::DeviceRegistry registry(devicesPath);
    AliasStore aliasStore(aliasPath);
    DiagnosticsNotesStore diagnosticsNotes(kDiagnosticsNotesPath);
    OscController oscController;
    MetricsLogger metricsLogger(kMetricsLogPath);
    MonitorEventQueue monitorEventQueue;
    std::deque<MonitorEventDisplay> monitorHistory;
    std::unordered_map<std::string, DeviceWsStats> deviceWsTelemetry;
    MonitorConnectionSnapshot monitorState{};
    MonitorConnectionSnapshot previousMonitorState{};
    std::mutex monitorStateMutex;
    bool monitorStateDirty = false;
    std::chrono::system_clock::time_point lastMonitorEventAt{};

    auto monitorStateHandler = [&](const MonitorConnectionSnapshot& snapshot) {
        std::lock_guard<std::mutex> lock(monitorStateMutex);
        monitorState = snapshot;
        monitorStateDirty = true;
    };

    MonitorWebSocketClient monitorClient(
        [&](MonitorEvent&& event) {
            monitorEventQueue.push(std::move(event));
        },
        monitorStateHandler,
        [&](double durationMs, bool success) {
            metricsLogger.recordMonitorReconnect(durationMs, success);
        }
    );

    SendStatsTracker sendStats;
    OscConfig oscConfig{};
    std::optional<std::string> renamingId;
    std::string aliasEditBuffer;

    std::set<std::string> selectedDevices;
    std::deque<EventLogEntry> eventLog;

    std::vector<DiagnosticsEntry> diagnostics;
    std::chrono::steady_clock::time_point lastDiagnosticsRefresh =
        std::chrono::steady_clock::now() - kDiagnosticsRefreshInterval;
    std::optional<std::string> editingDiagnosticId;
    std::string diagnosticNoteDraft;

    std::chrono::steady_clock::time_point lastRefresh = std::chrono::steady_clock::now() - kRegistryRefreshInterval;
    std::vector<DeviceSummary> devices;

    char timelinePathBuffer[512] = {0};
    char baseTimeBuffer[64] = {0};
    std::strncpy(timelinePathBuffer, "acoustics/pc_tools/scheduler/examples/basic_timeline.json", sizeof(timelinePathBuffer) - 1);
    bool baseTimeNow = true;
    double leadTimeSeconds = 1.0;
    char testPresetBuffer[64] = {0};
    std::strncpy(testPresetBuffer, "test_ping", sizeof(testPresetBuffer) - 1);
    double testLeadSeconds = 0.5;
    TimelinePreview timelinePreview;
    timelinePreview.sourcePath = fs::path(timelinePathBuffer);
    timelinePreview.leadSeconds = leadTimeSeconds;
    bool timelinePreviewDirty = true;
    bool timelineArmed = false;
    bool timelineDryRun = false;
    SingleShotForm singleShotForm;
    char monitorUrlBuffer[256];
    std::strncpy(monitorUrlBuffer, "ws://127.0.0.1:48080/ws/events", sizeof(monitorUrlBuffer) - 1);
    monitorUrlBuffer[sizeof(monitorUrlBuffer) - 1] = '\0';
    bool monitorAutoConnect = false;

    char hostBuffer[128];
    std::strncpy(hostBuffer, oscConfig.host.c_str(), sizeof(hostBuffer) - 1);
    int portValue = oscConfig.port;

    oscController.updateConfig(oscConfig, eventLog);

    const bool hasSavedLayout = fs::exists("imgui.ini");
    bool dockspaceBuilt = false;

    while (!glfwWindowShouldClose(window)) {
        auto frameStart = std::chrono::steady_clock::now();
        glfwPollEvents();

        auto now = std::chrono::steady_clock::now();
        auto refreshed = buildDeviceSummaries(registry, aliasStore, lastRefresh, now);
        if (!refreshed.empty()) {
            devices = std::move(refreshed);
        }
        if (now - lastDiagnosticsRefresh >= kDiagnosticsRefreshInterval) {
            diagnostics = loadDiagnosticsEntries(kDiagnosticsPath);
            lastDiagnosticsRefresh = now;
        }
        if (timelinePreviewDirty) {
            timelinePreview.sourcePath = fs::path(timelinePathBuffer);
            auto baseTime = std::chrono::system_clock::now();
            if (!baseTimeNow) {
                if (auto parsed = parseIso8601(baseTimeBuffer)) {
                    baseTime = *parsed;
                }
            }
            timelinePreview.baseTime = baseTime;
            timelinePreview.leadSeconds = leadTimeSeconds;
            if (auto loaded = tryLoadTimeline(timelinePreview.sourcePath, timelinePreview.lastError)) {
                timelinePreview.timeline = std::move(*loaded);
            } else {
                timelinePreview.timeline.reset();
            }
            timelinePreviewDirty = false;
        }

        MonitorEvent monitorEvent;
        while (monitorEventQueue.pop(monitorEvent)) {
            handleMonitorEvent(monitorEvent,
                               deviceWsTelemetry,
                               diagnostics,
                               eventLog,
                               monitorHistory);
            lastMonitorEventAt = std::chrono::system_clock::now();
        }

        MonitorConnectionSnapshot currentMonitorState;
        {
            std::lock_guard<std::mutex> lock(monitorStateMutex);
            currentMonitorState = monitorState;
        }
        if (currentMonitorState.connected != previousMonitorState.connected ||
            currentMonitorState.connecting != previousMonitorState.connecting ||
            currentMonitorState.status != previousMonitorState.status) {
            if (!currentMonitorState.status.empty()) {
                eventLog.emplace_back(EventLogEntry{
                    std::chrono::system_clock::now(),
                    spdlog::level::info,
                    fmt::format("Monitor WS: {}", currentMonitorState.status)
                });
                trimLog(eventLog);
            }
            previousMonitorState = currentMonitorState;
        }

        const bool hasMonitorEvent = lastMonitorEventAt.time_since_epoch().count() > 0;
        const bool monitorLinkStale = currentMonitorState.connected &&
                                      hasMonitorEvent &&
                                      (std::chrono::system_clock::now() - lastMonitorEventAt > kMonitorStaleThreshold);
        std::string monitorStatusLabel;
        if (!currentMonitorState.status.empty()) {
            monitorStatusLabel = currentMonitorState.status;
        } else if (currentMonitorState.connected) {
            monitorStatusLabel = "Connected";
        } else if (currentMonitorState.connecting) {
            monitorStatusLabel = "Connecting";
        } else {
            monitorStatusLabel = "Idle";
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiID dockspaceId = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
        if (!dockspaceBuilt && !hasSavedLayout) {
            dockspaceBuilt = true;
            ImGui::DockBuilderRemoveNode(dockspaceId);
            ImGui::DockBuilderAddNode(dockspaceId, ImGuiDockNodeFlags_None);
            ImGui::DockBuilderSetNodeSize(dockspaceId, ImGui::GetMainViewport()->Size);

            ImGuiID dockMain = dockspaceId;
            ImGuiID dockRight = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Right, 0.30f, nullptr, &dockMain);
            ImGuiID dockBottom = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Down, 0.25f, nullptr, &dockMain);

            ImGui::DockBuilderDockWindow("Dispatch", dockRight);
            ImGui::DockBuilderDockWindow("Timeline Preview", dockRight);
            ImGui::DockBuilderDockWindow("Monitor Link", dockRight);
            ImGui::DockBuilderDockWindow("Single Shot Console", dockRight);
            ImGui::DockBuilderDockWindow("OSC Endpoint", dockRight);
            ImGui::DockBuilderDockWindow("Event Log", dockBottom);
            ImGui::DockBuilderDockWindow("Diagnostics Center", dockBottom);
            ImGui::DockBuilderDockWindow("Status", dockBottom);
            ImGui::DockBuilderDockWindow("Top Bar", dockMain);
            ImGui::DockBuilderDockWindow("Devices", dockMain);
            ImGui::DockBuilderFinish(dockspaceId);
        }

        const auto nowUtc = std::chrono::system_clock::now();
        const auto nowJst = nowUtc + std::chrono::hours(9);
        const bool monitorReady = fs::exists(devicesPath) && !devices.empty();
        const bool schedulerReady = fs::exists("acoustics/pc_tools/scheduler/src/main.cpp");
        const bool secretsReady = fs::exists("acoustics/firmware/include/Secrets.h");

        if (ImGui::Begin("Top Bar", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("UTC %s", formatIso8601(nowUtc, false).c_str());
            ImGui::SameLine();
            ImGui::Text("JST %s", formatIso8601(nowJst, false).c_str());

            auto drawStatus = [](const char* label, bool ok) {
                ImGui::TextUnformatted(label);
                ImGui::SameLine();
                ImGui::TextColored(ok ? ImVec4(0.5f, 1.0f, 0.5f, 1.0f) : ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                                   ok ? "OK" : "NG");
            };
            drawStatus("Scheduler", schedulerReady);
            ImGui::SameLine();
            drawStatus("Monitor", monitorReady);
            ImGui::SameLine();
            drawStatus("Secrets", secretsReady);

            auto [ok, ng] = sendStats.lastHourCounts();
            ImGui::Text("Send stats (60m): success=%d fail=%d", ok, ng);
            auto ratios = sendStats.bucketizedSuccessRates();
            ImGui::PlotLines("##send_spark",
                             ratios.data(),
                             static_cast<int>(ratios.size()),
                             0,
                             "success ratio",
                             0.0f,
                             1.0f,
                             ImVec2(300.0f, 60.0f));
            ImGui::Separator();
            ImGui::Text("Monitor WS: %s", monitorStatusLabel.c_str());
            if (monitorLinkStale) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "STALE");
            }
            if (hasMonitorEvent) {
                double secondsSince = std::chrono::duration<double>(std::chrono::system_clock::now() - lastMonitorEventAt).count();
                ImGui::Text("Last event: %.1f s ago", secondsSince);
            } else {
                ImGui::TextDisabled("No monitor events yet");
            }
        }
        ImGui::End();

        if (ImGui::Begin("Timeline Preview")) {
            ImGui::Text("Source: %s", timelinePreview.sourcePath.string().c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("Refresh")) {
                timelinePreviewDirty = true;
            }
            ImGui::Text("Base UTC: %s", formatIso8601(timelinePreview.baseTime).c_str());
            ImGui::Text("Lead seconds: %.2f", timelinePreview.leadSeconds);
            if (!timelinePreview.ready()) {
                if (!timelinePreview.lastError.empty()) {
                    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Preview error: %s", timelinePreview.lastError.c_str());
                } else {
                    ImGui::TextDisabled("Load a timeline file to preview.");
                }
            } else {
                const auto& events = timelinePreview.timeline->events();
                if (ImGui::BeginTable("timeline_events", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Borders)) {
                    ImGui::TableSetupScrollFreeze(0, 1);
                    ImGui::TableSetupColumn("Scheduled (UTC)");
                    ImGui::TableSetupColumn("Remaining (s)");
                    ImGui::TableSetupColumn("Targets");
                    ImGui::TableSetupColumn("Preset");
                    ImGui::TableSetupColumn("Offset (s)");
                    ImGui::TableHeadersRow();
                    for (std::size_t i = 0; i < events.size(); ++i) {
                        const auto& evt = events[i];
                        auto scheduled = timelinePreview.baseTime + std::chrono::duration_cast<std::chrono::system_clock::duration>(
                            std::chrono::duration<double>(evt.offsetSeconds));
                        const double remaining = std::chrono::duration<double>(scheduled - nowUtc).count();
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%s", formatIso8601(scheduled).c_str());
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%.1f", remaining);
                        ImGui::TableSetColumnIndex(2);
                        ImGui::TextWrapped("%s", describeTargets(evt.targets).c_str());
                        ImGui::TableSetColumnIndex(3);
                        ImGui::TextUnformatted(extractPreset(evt).c_str());
                        ImGui::TableSetColumnIndex(4);
                        ImGui::Text("%.2f", evt.offsetSeconds);
                    }
                    ImGui::EndTable();
                }

                if (!events.empty()) {
                    std::vector<double> offsets;
                    std::vector<double> lanes;
                    offsets.reserve(events.size());
                    lanes.reserve(events.size());
                    for (std::size_t i = 0; i < events.size(); ++i) {
                        offsets.push_back(events[i].offsetSeconds);
                        lanes.push_back(static_cast<double>(events.size() - i));
                    }
                    if (ImPlot::BeginPlot("Offsets", ImVec2(-1, 180), ImPlotFlags_NoLegend | ImPlotFlags_NoTitle)) {
                        ImPlot::SetupAxes("Offset (s)", "Event", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_NoDecorations | ImPlotAxisFlags_AutoFit);
                        ImPlot::PlotScatter("events", offsets.data(), lanes.data(), static_cast<int>(offsets.size()));
                        ImPlot::EndPlot();
                    }
                }
            }
        }
        ImGui::End();

        if (ImGui::Begin("Monitor Link")) {
            ImGui::InputText("WebSocket URL", monitorUrlBuffer, IM_ARRAYSIZE(monitorUrlBuffer));
            bool connectClicked = ImGui::Button("Connect");
            ImGui::SameLine();
            bool disconnectClicked = ImGui::Button("Disconnect");
            ImGui::SameLine();
            bool autoChanged = ImGui::Checkbox("Auto-connect", &monitorAutoConnect);

            if (connectClicked) {
                monitorAutoConnect = true;
                monitorClient.start(monitorUrlBuffer);
            }
            if (disconnectClicked) {
                monitorAutoConnect = false;
                monitorClient.stop();
            }
            if (autoChanged) {
                if (monitorAutoConnect && !monitorClient.isRunning()) {
                    monitorClient.start(monitorUrlBuffer);
                } else if (!monitorAutoConnect) {
                    monitorClient.stop();
                }
            }

            ImGui::Separator();
            ImGui::Text("Status: %s", monitorStatusLabel.c_str());
            if (currentMonitorState.attempt > 0) {
                ImGui::Text("Attempts: %d", currentMonitorState.attempt);
            }
            if (monitorLinkStale) {
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                                   "STALE (>%lld s without events)",
                                   static_cast<long long>(kMonitorStaleThreshold.count()));
            }
            if (currentMonitorState.connected && hasMonitorEvent) {
                double secondsSince = std::chrono::duration<double>(std::chrono::system_clock::now() - lastMonitorEventAt).count();
                ImGui::Text("Last event %.1f s ago", secondsSince);
            } else if (!hasMonitorEvent) {
                ImGui::TextDisabled("No events yet");
            }

            if (ImGui::Button("Inject Sample Heartbeat")) {
                MonitorEvent sample;
                sample.type = "heartbeat";
                const std::string deviceId = devices.empty() ? std::string("device-sim") : devices.front().snapshot.state.id;
                sample.payload = {
                    {"device_id", deviceId},
                    {"latency_ms", 42.0},
                    {"queue_depth", 0},
                    {"is_playing", false}
                };
                monitorEventQueue.push(std::move(sample));
            }
            ImGui::SameLine();
            if (ImGui::Button("Inject Diagnostics")) {
                MonitorEvent diag;
                diag.type = "diagnostics";
                diag.payload = {
                    {"device_id", devices.empty() ? "device-sim" : devices.front().snapshot.state.id},
                    {"severity", "warn"},
                    {"reason", "Mock high latency"},
                    {"recommended_action", "Check Wi-Fi link"}
                };
                monitorEventQueue.push(std::move(diag));
            }

            ImGui::Separator();
            if (ImGui::BeginTable("monitor_history_table", 3, ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter)) {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("Time");
                ImGui::TableSetupColumn("Type");
                ImGui::TableSetupColumn("Summary");
                ImGui::TableHeadersRow();
                for (auto it = monitorHistory.rbegin(); it != monitorHistory.rend(); ++it) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%s", formatTimestamp(it->timestamp).c_str());
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%s", it->type.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextWrapped("%s", it->summary.c_str());
                }
                ImGui::EndTable();
            } else {
                ImGui::TextDisabled("No history yet");
            }
        }
        ImGui::End();

        if (ImGui::Begin("Single Shot Console")) {
            std::vector<const DeviceSummary*> deviceRefs;
            deviceRefs.reserve(devices.size());
            for (const auto& dev : devices) {
                deviceRefs.push_back(&dev);
            }
            if (singleShotForm.selectedDeviceIndex >= static_cast<int>(deviceRefs.size())) {
                singleShotForm.selectedDeviceIndex = static_cast<int>(deviceRefs.size()) - 1;
            }
            std::string targetLabel = "(select)";
            if (singleShotForm.selectedDeviceIndex >= 0 &&
                singleShotForm.selectedDeviceIndex < static_cast<int>(deviceRefs.size())) {
                const auto& dev = *deviceRefs[singleShotForm.selectedDeviceIndex];
                targetLabel = fmt::format("{} ({})", displayAlias(dev), dev.snapshot.state.id);
            }
            if (ImGui::BeginCombo("Target", targetLabel.c_str())) {
                for (int idx = 0; idx < static_cast<int>(deviceRefs.size()); ++idx) {
                    const auto& dev = *deviceRefs[idx];
                    auto label = fmt::format("{} ({})", displayAlias(dev), dev.snapshot.state.id);
                    bool selected = idx == singleShotForm.selectedDeviceIndex;
                    if (ImGui::Selectable(label.c_str(), selected)) {
                        singleShotForm.selectedDeviceIndex = idx;
                    }
                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            if (ImGui::SmallButton("Adopt from selection")) {
                if (!selectedDevices.empty()) {
                    const auto& targetId = *selectedDevices.begin();
                    for (int idx = 0; idx < static_cast<int>(deviceRefs.size()); ++idx) {
                        if (deviceRefs[idx]->snapshot.state.id == targetId) {
                            singleShotForm.selectedDeviceIndex = idx;
                            break;
                        }
                    }
                }
            }

            ImGui::InputText("Preset", &singleShotForm.preset);
            ImGui::SliderFloat("Lead (s)", &singleShotForm.leadSeconds, 0.0f, 5.0f);
            ImGui::SliderFloat("Gain (dB)", &singleShotForm.gainDb, -24.0f, 6.0f);
            ImGui::Checkbox("Limit duration", &singleShotForm.limitDuration);
            if (singleShotForm.limitDuration) {
                ImGui::SliderFloat("Max duration (s)", &singleShotForm.maxDurationSeconds, 0.1f, 30.0f);
            }
            ImGui::Checkbox("Dry run", &singleShotForm.dryRun);
            ImGui::Checkbox("Arm single shot", &singleShotForm.armed);
            const bool shotReady = singleShotForm.armed &&
                                   singleShotForm.selectedDeviceIndex >= 0 &&
                                   singleShotForm.selectedDeviceIndex < static_cast<int>(deviceRefs.size());
            ImGui::BeginDisabled(!shotReady);
            if (ImGui::Button("Fire")) {
                const auto& targetDev = *deviceRefs[singleShotForm.selectedDeviceIndex];
                sendSingleShot(oscController, targetDev, singleShotForm, eventLog, sendStats);
                singleShotForm.armed = false;
            }
            ImGui::EndDisabled();
        }
        ImGui::End();

        if (ImGui::Begin("Diagnostics Center")) {
            ImGui::Text("Entries: %zu", diagnostics.size());
            ImGui::SameLine();
            if (ImGui::SmallButton("Force Refresh")) {
                diagnostics = loadDiagnosticsEntries(kDiagnosticsPath);
                lastDiagnosticsRefresh = std::chrono::steady_clock::now();
            }
            ImGui::Text("Notes: %s", diagnosticsNotes.path().string().c_str());
            ImGui::Separator();
            ImGui::BeginChild("DiagnosticsList", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
            for (const auto& diag : diagnostics) {
                ImGui::PushID(diag.id.c_str());
                ImGui::TextColored(severityColor(diag.severity), "%s", diag.severity.empty() ? "unknown" : diag.severity.c_str());
                ImGui::SameLine();
                ImGui::Text("%s", formatIso8601(diag.timestamp).c_str());
                if (!diag.deviceId.empty()) {
                    ImGui::Text("Device: %s", diag.deviceId.c_str());
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Focus##diagfocus")) {
                        selectedDevices.clear();
                        selectedDevices.insert(diag.deviceId);
                    }
                }
                if (!diag.reason.empty()) {
                    ImGui::TextWrapped("%s", diag.reason.c_str());
                }
                if (!diag.recommendedAction.empty()) {
                    ImGui::Text("Action: %s", diag.recommendedAction.c_str());
                }
                if (!diag.relatedEventId.empty()) {
                    ImGui::TextDisabled("Related: %s", diag.relatedEventId.c_str());
                }

                std::string note = diagnosticsNotes.noteFor(diag.id);
                if (editingDiagnosticId && *editingDiagnosticId == diag.id) {
                    ImGui::InputTextMultiline("Note", &diagnosticNoteDraft, ImVec2(-1, 80.0f));
                    if (ImGui::Button("Save Note")) {
                        diagnosticsNotes.setNote(diag.id, diagnosticNoteDraft);
                        eventLog.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::info,
                                                            fmt::format("Diagnostics note updated (%s)", diag.id)});
                        trimLog(eventLog);
                        appendAuditRecord("diagnostic_note",
                                          diag.deviceId,
                                          diag.id,
                                          true,
                                          "Note updated");
                        editingDiagnosticId.reset();
                        diagnosticNoteDraft.clear();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel##note")) {
                        editingDiagnosticId.reset();
                    }
                } else {
                    if (note.empty()) {
                        ImGui::TextDisabled("Note: (none)");
                    } else {
                        ImGui::TextWrapped("Note: %s", note.c_str());
                    }
                    if (ImGui::SmallButton("Edit Note")) {
                        editingDiagnosticId = diag.id;
                        diagnosticNoteDraft = note;
                    }
                }
                ImGui::Separator();
                ImGui::PopID();
            }
            ImGui::EndChild();
        }
        ImGui::End();

        if (ImGui::Begin("OSC Endpoint")) {
            ImGui::InputText("Host", hostBuffer, IM_ARRAYSIZE(hostBuffer));
            ImGui::InputInt("Port", &portValue);
            ImGui::Checkbox("Broadcast", &oscConfig.broadcast);
            if (ImGui::Button("Apply")) {
                oscConfig.host = hostBuffer;
                oscConfig.port = std::clamp(portValue, 1, 65535);
                oscController.updateConfig(oscConfig, eventLog);
            }
        }
        ImGui::End();

        if (ImGui::Begin("Devices")) {
            ImGui::Text("Online: %zu", devices.size());
            const int tilesPerColumn = 20;
            int tileIndex = 0;
            ImGui::BeginChild("DeviceGrid", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
            const auto nowSystem = std::chrono::system_clock::now();
            for (auto& dev : devices) {
                if (tileIndex % tilesPerColumn == 0) {
                    if (tileIndex != 0) {
                        ImGui::SameLine();
                    }
                    ImGui::BeginGroup();
                }

                ImGui::PushID(dev.snapshot.state.id.c_str());
                ImGui::BeginChild("DeviceTile", ImVec2(220, 135), true);
            std::string title = displayAlias(dev);
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "%s", title.c_str());
            ImGui::TextDisabled("%s", dev.snapshot.state.id.c_str());

            ImGui::SameLine(160.0f);
            ImGui::ColorButton("##status", ImColor(colorForHealth(dev.health)), ImGuiColorEditFlags_NoTooltip, ImVec2(18, 18));

            if (ImGui::Button("Rename")) {
                renamingId = dev.snapshot.state.id;
                aliasEditBuffer = dev.alias;
            }
            if (renamingId && *renamingId == dev.snapshot.state.id) {
                ImGui::InputText("Alias", &aliasEditBuffer);
                if (ImGui::Button("Save")) {
                    aliasStore.setAlias(dev.snapshot.state.id, aliasEditBuffer);
                    dev.alias = aliasEditBuffer;
                    eventLog.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::info,
                                                        fmt::format("Alias updated: {} => '{}'", dev.snapshot.state.id, aliasEditBuffer)});
                    trimLog(eventLog);
                    renamingId.reset();
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel")) {
                    renamingId.reset();
                }
            }

            bool selected = selectedDevices.count(dev.snapshot.state.id) > 0;
            if (ImGui::Checkbox("Select", &selected)) {
                if (selected) {
                    selectedDevices.insert(dev.snapshot.state.id);
                } else {
                    selectedDevices.erase(dev.snapshot.state.id);
                }
            }

            ImGui::Text("Latency: %.1f ms (std %.1f)", dev.meanLatency, dev.stdLatency);
            ImGui::Text("Heartbeat: %.1f s ago", dev.secondsSinceSeen);
            if (auto it = deviceWsTelemetry.find(dev.snapshot.state.id); it != deviceWsTelemetry.end()) {
                const bool wsStale = nowSystem - it->second.lastHeartbeatAt > kMonitorStaleThreshold;
                ImVec4 wsColor = wsStale ? ImVec4(1.0f, 0.5f, 0.5f, 1.0f) : ImVec4(0.6f, 0.9f, 0.6f, 1.0f);
                ImGui::TextColored(wsColor,
                                   "WS %.1f ms @ %s",
                                   it->second.lastLatencyMs,
                                   formatTimestamp(it->second.lastHeartbeatAt).c_str());
                if (it->second.queueDepth) {
                    ImGui::SameLine();
                    ImGui::Text("Queue=%d", *it->second.queueDepth);
                }
                if (it->second.isPlaying) {
                    ImGui::SameLine();
                    ImGui::Text("%s", *it->second.isPlaying ? "Playing" : "Idle");
                }
            }

            if (ImGui::Button("Test Signal")) {
                sendTestSignal(oscController, testPresetBuffer, dev, testLeadSeconds, eventLog, sendStats);
            }
            ImGui::SameLine();
            if (ImGui::Button("Focus")) {
                selectedDevices.clear();
                selectedDevices.insert(dev.snapshot.state.id);
            }

            ImGui::EndChild();
            ImGui::PopID();

            ++tileIndex;
            if (tileIndex % tilesPerColumn == 0) {
                ImGui::EndGroup();
            }
        }
        if (tileIndex % tilesPerColumn != 0) {
            ImGui::EndGroup();
        }
        ImGui::EndChild();
        }
        ImGui::End();

        if (ImGui::Begin("Dispatch")) {
            ImGui::TextUnformatted("Selected Devices");
            if (selectedDevices.empty()) {
                ImGui::TextDisabled("(none)");
            } else {
                for (const auto& id : selectedDevices) {
                    ImGui::BulletText("%s", id.c_str());
                }
                if (ImGui::Button("Clear Selection")) {
                    selectedDevices.clear();
                }
            }

            ImGui::Separator();
            bool timelineInputChanged = false;
            if (ImGui::InputText("Timeline", timelinePathBuffer, IM_ARRAYSIZE(timelinePathBuffer))) {
                timelineInputChanged = true;
            }
            if (ImGui::Checkbox("Use current time", &baseTimeNow)) {
                timelineInputChanged = true;
            }
            if (!baseTimeNow) {
                if (ImGui::InputText("Base time (ISO)", baseTimeBuffer, IM_ARRAYSIZE(baseTimeBuffer))) {
                    timelineInputChanged = true;
                }
            }
            if (ImGui::SliderFloat("Lead time (s)", reinterpret_cast<float*>(&leadTimeSeconds), 0.0f, 5.0f)) {
                timelineInputChanged = true;
            }
            if (timelineInputChanged) {
                timelinePreviewDirty = true;
            }

            if (!timelinePreview.lastError.empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Preview error: %s", timelinePreview.lastError.c_str());
            } else if (!timelinePreview.ready()) {
                ImGui::TextDisabled("Preview not available yet.");
            } else {
                ImGui::Text("Preview events: %zu | Lead=%.2f s", timelinePreview.timeline->events().size(), timelinePreview.leadSeconds);
            }

            ImGui::Checkbox("Dry run only", &timelineDryRun);
            ImGui::Checkbox("Arm timeline send", &timelineArmed);
            ImGui::BeginDisabled(!timelineArmed);
            if (ImGui::Button("Send Timeline")) {
                std::size_t targetCount = selectedDevices.empty() ? devices.size() : selectedDevices.size();
                const auto timelineName = fs::path(timelinePathBuffer).filename().string();
                if (timelineDryRun) {
                    const std::size_t eventCount = timelinePreview.ready() ? timelinePreview.timeline->events().size() : 0;
                    auto detail = fmt::format("Dry-run timeline '%s' (targets=%zu events=%zu)", timelineName, targetCount, eventCount);
                    eventLog.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::info, detail});
                    trimLog(eventLog);
                    sendStats.record(true, fmt::format("timeline:dryrun:%s", timelineName), detail);
                    appendAuditRecord("timeline_dry_run",
                                      fmt::format("%zu target(s)", targetCount),
                                      timelineName,
                                      true,
                                      detail);
                } else {
                    auto outcome = sendTimelineToDevices(devices,
                                                         selectedDevices,
                                                         fs::path(timelinePathBuffer),
                                                         leadTimeSeconds,
                                                         baseTimeNow,
                                                         baseTimeBuffer,
                                                         oscController,
                                                         eventLog,
                                                         sendStats,
                                                         metricsLogger);
                    (void)outcome;
                }
                timelineArmed = false;
                timelineDryRun = false;
                timelinePreviewDirty = true;
            }
            ImGui::EndDisabled();

            ImGui::Separator();
            ImGui::InputText("Test preset", testPresetBuffer, IM_ARRAYSIZE(testPresetBuffer));
            ImGui::SliderFloat("Test lead (s)", reinterpret_cast<float*>(&testLeadSeconds), 0.0f, 2.0f);
        }
        ImGui::End();

        if (ImGui::Begin("Event Log")) {
            if (ImGui::Button("Export CSV")) {
                try {
                    std::ofstream out(kDefaultEventLogCsv);
                    if (!out) {
                        throw std::runtime_error("cannot open log file");
                    }
                    out << "timestamp,level,message\n";
                    for (const auto& entry : eventLog) {
                    out << formatTimestamp(entry.timestamp) << ',';
                    auto level_sv = spdlog::level::to_string_view(entry.level);
                    out.write(level_sv.data(), static_cast<std::streamsize>(level_sv.size()));
                    out << ',' << '"' << entry.message << '"' << '\n';
                    }
                    eventLog.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::info,
                                                        fmt::format("Event log exported to {}", kDefaultEventLogCsv.string())});
                } catch (const std::exception& ex) {
                    eventLog.emplace_back(EventLogEntry{std::chrono::system_clock::now(), spdlog::level::err,
                                                        fmt::format("Export failed: {}", ex.what())});
                }
                trimLog(eventLog);
            }
            ImGui::Separator();
            if (ImGui::BeginTable("logtable", 1, ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("Message");
                ImGui::TableHeadersRow();
                ImGuiListClipper clipper;
                clipper.Begin(static_cast<int>(eventLog.size()));
                while (clipper.Step()) {
                    for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                        auto it = eventLog[eventLog.size() - 1 - i];
                        ImGui::TableNextRow();
                        ImVec4 color = (it.level >= spdlog::level::warn) ? ImVec4(1.0f, 0.6f, 0.2f, 1.0f) : ImVec4(0.8f, 0.8f, 0.8f, 1.0f);
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextColored(color, "[%s] %s", formatTimestamp(it.timestamp).c_str(), it.message.c_str());
                    }
                }
                ImGui::EndTable();
            }
        }
        ImGui::End();

        if (ImGui::Begin("Status")) {
            ImGui::Text("Alias store: %s", aliasStore.path().string().c_str());
            ImGui::Text("OSC: %s:%d (broadcast=%s)", oscConfig.host.c_str(), oscConfig.port, oscConfig.broadcast ? "true" : "false");
            ImGui::Text("Selected: %zu", selectedDevices.size());
            ImGui::Text("Audit log: %s", kAuditLogPath.string().c_str());
            ImGui::Text("Diag notes: %s", diagnosticsNotes.path().string().c_str());
            ImGui::Text("Monitor WS: %s", monitorStatusLabel.c_str());
        }
        ImGui::End();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        auto frameEnd = std::chrono::steady_clock::now();
        double frameMs = std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();
        metricsLogger.recordFrame(frameMs);
        metricsLogger.flushIfNeeded();
    }

    monitorClient.stop();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
