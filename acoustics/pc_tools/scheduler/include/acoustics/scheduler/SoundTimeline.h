#pragma once

#include "acoustics/osc/OscPacket.h"

#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

namespace acoustics::scheduler {

struct TimelineEvent {
    double offsetSeconds{};
    std::string address;
    std::vector<osc::Argument> arguments;
};

class SoundTimeline {
public:
    static SoundTimeline fromJsonFile(const std::filesystem::path& path);

    const std::vector<TimelineEvent>& events() const noexcept { return events_; }
    double defaultLeadTimeSeconds() const noexcept { return defaultLeadTime_; }
    const std::string& version() const noexcept { return version_; }

    std::vector<osc::Bundle> toBundles(
        std::chrono::system_clock::time_point baseTime,
        double leadTimeSeconds
    ) const;

private:
    std::string version_{"1.0"};
    double defaultLeadTime_{1.0};
    std::vector<TimelineEvent> events_;
};

}  // namespace acoustics::scheduler
