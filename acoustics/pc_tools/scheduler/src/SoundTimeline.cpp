#include "acoustics/scheduler/SoundTimeline.h"

#include "json.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace acoustics::scheduler {

namespace {

using json = nlohmann::json;

osc::Argument jsonToArgument(const json& value) {
    if (value.is_number_integer()) {
        auto raw = value.get<std::int64_t>();
        if (raw < std::numeric_limits<std::int32_t>::min() || raw > std::numeric_limits<std::int32_t>::max()) {
            throw std::runtime_error("OSC int argument exceeds 32-bit range");
        }
        return static_cast<std::int32_t>(raw);
    }
    if (value.is_number_float()) {
        return static_cast<float>(value.get<double>());
    }
    if (value.is_string()) {
        return value.get<std::string>();
    }
    if (value.is_boolean()) {
        return value.get<bool>();
    }
    if (value.is_binary()) {
        const auto& bin = value.get_binary();
        return osc::Blob(bin.begin(), bin.end());
    }
    throw std::runtime_error("Unsupported OSC argument type in timeline JSON");
}

TimelineEvent parseEvent(const json& eventJson) {
    TimelineEvent event;
    if (!eventJson.contains("offset") || !eventJson.contains("address")) {
        throw std::runtime_error("Timeline event missing offset or address");
    }
    event.offsetSeconds = eventJson.at("offset").get<double>();
    event.address = eventJson.at("address").get<std::string>();
    if (event.address.empty() || event.address.front() != '/') {
        throw std::runtime_error("OSC address must start with '/'");
    }

    if (eventJson.contains("args")) {
        for (const auto& arg : eventJson.at("args")) {
            event.arguments.emplace_back(jsonToArgument(arg));
        }
    }
    return event;
}

}  // namespace

SoundTimeline SoundTimeline::fromJsonFile(const std::filesystem::path& path) {
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("Timeline file not found: " + path.string());
    }

    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open timeline file: " + path.string());
    }

    json root;
    input >> root;

    SoundTimeline timeline;
    if (root.contains("version")) {
        timeline.version_ = root.at("version").get<std::string>();
    }
    if (root.contains("default_lead_time")) {
        timeline.defaultLeadTime_ = root.at("default_lead_time").get<double>();
    }
    if (!root.contains("events") || !root.at("events").is_array()) {
        throw std::runtime_error("Timeline JSON must contain an 'events' array");
    }

    for (const auto& eventJson : root.at("events")) {
        timeline.events_.push_back(parseEvent(eventJson));
    }
    std::sort(timeline.events_.begin(), timeline.events_.end(), [](const auto& a, const auto& b) {
        return a.offsetSeconds < b.offsetSeconds;
    });

    return timeline;
}

std::vector<osc::Bundle> SoundTimeline::toBundles(
    std::chrono::system_clock::time_point baseTime,
    double leadTimeSeconds) const {
    const double lead = (leadTimeSeconds >= 0.0) ? leadTimeSeconds : defaultLeadTime_;
    if (lead < 0.0) {
        throw std::runtime_error("Lead time must be non-negative");
    }

    struct ScheduledMessage {
        std::chrono::system_clock::time_point execTime;
        osc::Message message;
    };

    std::vector<ScheduledMessage> scheduled;
    scheduled.reserve(events_.size());

    for (const auto& event : events_) {
        osc::Message msg;
        msg.address = event.address;
        msg.arguments = event.arguments;

        auto execTime = baseTime + std::chrono::duration_cast<std::chrono::system_clock::duration>(
                                          std::chrono::duration<double>(lead + event.offsetSeconds));
        scheduled.push_back(ScheduledMessage{execTime, msg});
    }

    std::sort(scheduled.begin(), scheduled.end(), [](const auto& a, const auto& b) {
        return a.execTime < b.execTime;
    });

    std::vector<osc::Bundle> bundles;
    std::size_t idx = 0;
    while (idx < scheduled.size()) {
        const auto groupTime = scheduled[idx].execTime;
        osc::Bundle bundle;
        bundle.timetag = osc::toTimetag(groupTime);

        while (idx < scheduled.size() && scheduled[idx].execTime == groupTime) {
            bundle.elements.emplace_back(std::move(scheduled[idx].message));
            ++idx;
        }
        bundles.emplace_back(std::move(bundle));
    }

    return bundles;
}

}  // namespace acoustics::scheduler
