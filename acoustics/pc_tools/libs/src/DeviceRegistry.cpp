#include "acoustics/common/DeviceRegistry.h"

#include "json.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace acoustics::common {

namespace {

using json = nlohmann::json;

std::string timePointToIso(std::chrono::system_clock::time_point tp) {
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm tm = *std::gmtime(&tt);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

std::time_t toUtcTimeT(std::tm tm) {
#if defined(_WIN32)
    return _mkgmtime(&tm);
#else
    return timegm(&tm);
#endif
}

std::chrono::system_clock::time_point isoToTimePoint(const std::string& iso) {
    std::tm tm{};
    std::istringstream iss(iso);
    iss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    if (iss.fail()) {
        throw std::runtime_error("Failed to parse ISO8601 timestamp: " + iso);
    }
    auto timeT = toUtcTimeT(tm);
    return std::chrono::system_clock::from_time_t(timeT);
}

}  // namespace

void HeartbeatStats::addSample(double latencyMs) {
    ++count;
    double delta = latencyMs - meanLatencyMs;
    meanLatencyMs += delta / static_cast<double>(count);
    double delta2 = latencyMs - meanLatencyMs;
    m2 += delta * delta2;
}

double HeartbeatStats::variance() const {
    if (count < 2) {
        return 0.0;
    }
    return m2 / static_cast<double>(count - 1);
}

double HeartbeatStats::standardDeviation() const {
    return std::sqrt(variance());
}

DeviceRegistry::DeviceRegistry(std::filesystem::path storagePath)
    : storagePath_(std::move(storagePath)) {}

void DeviceRegistry::load() {
    std::lock_guard lock(mutex_);
    devicesById_.clear();
    macToId_.clear();

    if (!std::filesystem::exists(storagePath_)) {
        return;
    }

    std::ifstream input(storagePath_);
    if (!input) {
        throw std::runtime_error("Failed to open registry file: " + storagePath_.string());
    }

    json root;
    input >> root;
    if (!root.is_array()) {
        throw std::runtime_error("Device registry JSON must be an array");
    }

    for (const auto& entry : root) {
        DeviceState state;
        state.id = entry.at("id").get<std::string>();
        state.mac = entry.at("mac").get<std::string>();
        state.firmwareVersion = entry.value("fw_version", "");
        if (entry.contains("alias") && !entry.at("alias").is_null()) {
            state.alias = entry.at("alias").get<std::string>();
        }
        if (entry.contains("last_seen")) {
            state.lastSeen = isoToTimePoint(entry.at("last_seen").get<std::string>());
        }
        if (entry.contains("heartbeat")) {
            const auto& hb = entry.at("heartbeat");
            state.heartbeat.count = hb.value("count", 0ULL);
            state.heartbeat.meanLatencyMs = hb.value("mean_ms", 0.0);
            state.heartbeat.m2 = hb.value("m2", 0.0);
        }
        macToId_[normalizeMac(state.mac)] = state.id;
        devicesById_.emplace(state.id, std::move(state));
    }
}

void DeviceRegistry::save() const {
    std::lock_guard lock(mutex_);

    json root = json::array();
    for (const auto& [id, state] : devicesById_) {
        json node;
        node["id"] = state.id;
        node["mac"] = state.mac;
        node["fw_version"] = state.firmwareVersion;
        if (state.alias.has_value()) {
            node["alias"] = *state.alias;
        } else {
            node["alias"] = nullptr;
        }
        node["last_seen"] = timePointToIso(state.lastSeen);

        json hb;
        hb["count"] = state.heartbeat.count;
        hb["mean_ms"] = state.heartbeat.meanLatencyMs;
        hb["m2"] = state.heartbeat.m2;
        node["heartbeat"] = hb;

        root.push_back(std::move(node));
    }

    const auto parent = storagePath_.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    std::ofstream output(storagePath_);
    if (!output) {
        throw std::runtime_error("Failed to write registry file: " + storagePath_.string());
    }
    output << std::setw(2) << root;
}

DeviceState DeviceRegistry::registerAnnounce(const std::string& mac,
                                             const std::string& fwVersion,
                                             std::optional<std::string> alias,
                                             std::chrono::system_clock::time_point now) {
    std::unique_lock lock(mutex_);
    DeviceState& state = ensureDeviceLocked(mac, fwVersion, std::move(alias), now);
    state.lastSeen = now;
    DeviceState snapshot = state;
    lock.unlock();
    save();
    return snapshot;
}

void DeviceRegistry::recordHeartbeat(const std::string& deviceId,
                                     double latencyMs,
                                     std::chrono::system_clock::time_point now) {
    std::unique_lock lock(mutex_);
    auto it = devicesById_.find(deviceId);
    if (it == devicesById_.end()) {
        return;
    }
    it->second.lastSeen = now;
    it->second.heartbeat.addSample(latencyMs);
    lock.unlock();
    save();
}

std::optional<DeviceState> DeviceRegistry::findById(const std::string& deviceId) const {
    std::lock_guard lock(mutex_);
    auto it = devicesById_.find(deviceId);
    if (it == devicesById_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<DeviceState> DeviceRegistry::findByMac(const std::string& mac) const {
    std::lock_guard lock(mutex_);
    auto norm = normalizeMac(mac);
    auto itId = macToId_.find(norm);
    if (itId == macToId_.end()) {
        return std::nullopt;
    }
    auto itDevice = devicesById_.find(itId->second);
    if (itDevice == devicesById_.end()) {
        return std::nullopt;
    }
    return itDevice->second;
}

std::vector<DeviceSnapshot> DeviceRegistry::snapshot() const {
    std::lock_guard lock(mutex_);
    std::vector<DeviceSnapshot> out;
    auto now = std::chrono::system_clock::now();
    out.reserve(devicesById_.size());
    for (const auto& [id, state] : devicesById_) {
        out.push_back(DeviceSnapshot{state, now});
    }
    std::sort(out.begin(), out.end(), [](const DeviceSnapshot& a, const DeviceSnapshot& b) {
        return a.state.id < b.state.id;
    });
    return out;
}

std::string DeviceRegistry::normalizeMac(const std::string& mac) {
    std::string out;
    out.reserve(mac.size());
    for (char c : mac) {
        if (c == ':' || c == '-') {
            continue;
        }
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return out;
}

std::string DeviceRegistry::generateDeviceId(const std::string& mac) {
    auto normalized = normalizeMac(mac);
    if (normalized.empty()) {
        throw std::runtime_error("MAC address cannot be empty");
    }
    return "dev-" + normalized;
}

DeviceState& DeviceRegistry::ensureDeviceLocked(const std::string& mac,
                                                const std::string& fwVersion,
                                                std::optional<std::string> alias,
                                                std::chrono::system_clock::time_point now) {
    auto normalized = normalizeMac(mac);
    auto it = macToId_.find(normalized);
    if (it != macToId_.end()) {
        DeviceState& state = devicesById_.at(it->second);
        state.firmwareVersion = fwVersion;
        state.alias = std::move(alias);
        state.mac = mac;
        state.lastSeen = now;
        return state;
    }

    DeviceState state;
    state.id = generateDeviceId(mac);
    state.mac = mac;
    state.firmwareVersion = fwVersion;
    state.alias = std::move(alias);
    state.lastSeen = now;
    auto [insertedIt, inserted] = devicesById_.emplace(state.id, std::move(state));
    macToId_[normalized] = insertedIt->first;
    return insertedIt->second;
}

}  // namespace acoustics::common
