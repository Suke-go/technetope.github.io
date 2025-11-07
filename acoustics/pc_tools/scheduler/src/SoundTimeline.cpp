#include "acoustics/scheduler/SoundTimeline.h"

#include "json.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace acoustics::scheduler {

namespace {

using json = nlohmann::json;

constexpr double kMinimumLeadTimeSeconds = 3.0;

std::optional<std::string> extractPresetId(const osc::Message& message) {
    if (message.address == "/acoustics/play" && !message.arguments.empty()) {
        const auto& first = message.arguments.front();
        if (std::holds_alternative<std::string>(first)) {
            return std::get<std::string>(first);
        }
    }
    return std::nullopt;
}

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

    if (eventJson.contains("targets")) {
        if (!eventJson.at("targets").is_array()) {
            throw std::runtime_error("Timeline event 'targets' must be an array");
        }
        for (const auto& value : eventJson.at("targets")) {
            if (!value.is_string()) {
                throw std::runtime_error("Timeline event 'targets' entries must be strings");
            }
            event.targets.emplace_back(value.get<std::string>());
        }
    }
    return event;
}

}  // namespace

osc::Bundle ScheduledBundle::toOscBundle() const {
    osc::Bundle bundle;
    bundle.timetag = osc::toTimetag(executionTime);
    bundle.elements.reserve(messages.size());
    for (const auto& detail : messages) {
        bundle.elements.push_back(detail.message);
    }
    return bundle;
}

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
        if (timeline.defaultLeadTime_ < kMinimumLeadTimeSeconds) {
            throw std::runtime_error("Timeline default_lead_time must be >= 3 seconds");
        }
    }
    if (!root.contains("events") || !root.at("events").is_array()) {
        throw std::runtime_error("Timeline JSON must contain an 'events' array");
    }

    for (const auto& eventJson : root.at("events")) {
        timeline.events_.push_back(parseEvent(eventJson));
    }
    std::stable_sort(timeline.events_.begin(), timeline.events_.end(),
                     [](const auto& a, const auto& b) {
                         return a.offsetSeconds < b.offsetSeconds;
                     });

    return timeline;
}

std::vector<ScheduledBundle> SoundTimeline::schedule(
    std::chrono::system_clock::time_point baseTime,
    double leadTimeSeconds,
    const TargetResolver& resolver) const {
    const double lead = (leadTimeSeconds >= 0.0) ? leadTimeSeconds : defaultLeadTime_;
    if (lead < 0.0) {
        throw std::runtime_error("Lead time must be non-negative");
    }
    if (lead < kMinimumLeadTimeSeconds) {
        throw std::runtime_error("Lead time must be at least 3 seconds to satisfy device scheduling requirements");
    }

    struct TimedMessage {
        std::chrono::system_clock::time_point execTime;
        ScheduledMessage detail;
    };

    std::vector<TimedMessage> scheduled;
    scheduled.reserve(events_.size());

    for (const auto& event : events_) {
        auto execTime = baseTime + std::chrono::duration_cast<std::chrono::system_clock::duration>(
                                          std::chrono::duration<double>(lead + event.offsetSeconds));

        auto targets = resolver.resolve(event.targets);
        if (targets.empty()) {
            ScheduledMessage detail;
            detail.message.address = event.address;
            detail.message.arguments = event.arguments;
            detail.presetId = extractPresetId(detail.message);
            scheduled.push_back(TimedMessage{execTime, std::move(detail)});
        } else {
            for (const auto& targetId : targets) {
                ScheduledMessage detail;
                detail.message.address = event.address;
                detail.message.arguments = event.arguments;
                detail.targetId = targetId;
                detail.presetId = extractPresetId(detail.message);
                scheduled.push_back(TimedMessage{execTime, std::move(detail)});
            }
        }
    }

    std::sort(scheduled.begin(), scheduled.end(), [](const auto& a, const auto& b) {
        return a.execTime < b.execTime;
    });

    std::vector<ScheduledBundle> bundles;
    std::size_t idx = 0;
    while (idx < scheduled.size()) {
        const auto groupTime = scheduled[idx].execTime;
        ScheduledBundle bundle;
        bundle.executionTime = groupTime;
        while (idx < scheduled.size() && scheduled[idx].execTime == groupTime) {
            bundle.messages.emplace_back(std::move(scheduled[idx].detail));
            ++idx;
        }
        bundles.emplace_back(std::move(bundle));
    }

    return bundles;
}

std::vector<osc::Bundle> SoundTimeline::toBundles(
    std::chrono::system_clock::time_point baseTime,
    double leadTimeSeconds) const {
    TargetResolver resolver;
    const auto scheduled = schedule(baseTime, leadTimeSeconds, resolver);
    std::vector<osc::Bundle> bundles;
    bundles.reserve(scheduled.size());
    for (const auto& bundle : scheduled) {
        bundles.push_back(bundle.toOscBundle());
    }
    return bundles;
}

}  // namespace acoustics::scheduler
