#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace acoustics::common {

struct HeartbeatStats {
    std::uint64_t count{0};
    double meanLatencyMs{0.0};
    double m2{0.0};

    void addSample(double latencyMs);
    double variance() const;
    double standardDeviation() const;
};

struct DeviceState {
    std::string id;
    std::string mac;
    std::string firmwareVersion;
    std::optional<std::string> alias;
    std::chrono::system_clock::time_point lastSeen{};
    HeartbeatStats heartbeat;
};

struct DeviceSnapshot {
    DeviceState state;
    std::chrono::system_clock::time_point snapshotTime;
};

class DeviceRegistry {
public:
    explicit DeviceRegistry(std::filesystem::path storagePath);

    void load();
    void save() const;

    DeviceState registerAnnounce(const std::string& mac,
                                 const std::string& fwVersion,
                                 std::optional<std::string> alias,
                                 std::chrono::system_clock::time_point now);

    void recordHeartbeat(const std::string& deviceId,
                         double latencyMs,
                         std::chrono::system_clock::time_point now);

    std::optional<DeviceState> findById(const std::string& deviceId) const;
    std::optional<DeviceState> findByMac(const std::string& mac) const;

    std::vector<DeviceSnapshot> snapshot() const;

private:
    static std::string normalizeMac(const std::string& mac);
    std::string generateDeviceId(const std::string& mac);

    DeviceState& ensureDeviceLocked(const std::string& mac,
                                    const std::string& fwVersion,
                                    std::optional<std::string> alias,
                                    std::chrono::system_clock::time_point now);

    std::filesystem::path storagePath_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, DeviceState> devicesById_;
    std::unordered_map<std::string, std::string> macToId_;
};

}  // namespace acoustics::common
