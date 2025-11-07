#pragma once

#include "acoustics/osc/OscEncryptor.h"
#include "acoustics/scheduler/SoundTimeline.h"
#include "acoustics/scheduler/TargetResolver.h"

#include <chrono>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace acoustics::scheduler {

struct SchedulerConfig {
    std::filesystem::path timelinePath;
    std::string host{"255.255.255.255"};
    std::uint16_t port{9000};
    double leadTimeOverride{-1.0};
    double bundleSpacing{0.01};
    bool broadcast{true};
    bool dryRun{false};
    std::optional<std::chrono::system_clock::time_point> baseTime;
    std::filesystem::path targetMapPath;
    std::vector<std::string> defaultTargets;
    bool encryptOsc{false};
    std::optional<acoustics::osc::OscEncryptor::Key256> oscKey;
    std::optional<acoustics::osc::OscEncryptor::Iv128> oscIv;
};

struct SchedulerReport {
    std::vector<ScheduledBundle> bundles;
};

class SchedulerController {
public:
    SchedulerController() = default;

    SchedulerReport execute(const SchedulerConfig& config) const;

    static std::chrono::system_clock::time_point parseBaseTime(const std::string& iso8601);

private:
    TargetResolver buildResolver(const SchedulerConfig& config) const;
};

}  // namespace acoustics::scheduler
