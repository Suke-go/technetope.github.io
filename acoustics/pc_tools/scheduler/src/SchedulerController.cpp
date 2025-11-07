#include "acoustics/scheduler/SchedulerController.h"

#include "acoustics/osc/OscTransport.h"

#include "json.hpp"

#include <asio.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace acoustics::scheduler {

namespace {

using json = nlohmann::json;

constexpr double kMinimumLeadTimeSeconds = 3.0;

std::string trimCopy(const std::string& value) {
    auto first = value.begin();
    while (first != value.end() && std::isspace(static_cast<unsigned char>(*first))) {
        ++first;
    }
    if (first == value.end()) {
        return {};
    }
    auto last = value.end();
    do {
        --last;
    } while (last != first && std::isspace(static_cast<unsigned char>(*last)));
    return std::string(first, last + 1);
}

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

std::unordered_map<std::string, std::vector<std::string>> loadJsonTargetMap(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open target JSON mapping: " + path.string());
    }

    json root;
    input >> root;
    if (!root.is_object()) {
        throw std::runtime_error("Target JSON mapping must contain an object at the root");
    }

    std::unordered_map<std::string, std::vector<std::string>> mapping;
    for (const auto& [key, value] : root.items()) {
        std::vector<std::string> deviceIds;
        if (value.is_array()) {
            for (const auto& element : value) {
                if (!element.is_string()) {
                    throw std::runtime_error("Target JSON mapping arrays must contain strings only");
                }
                deviceIds.emplace_back(element.get<std::string>());
            }
        } else if (value.is_string()) {
            deviceIds.emplace_back(value.get<std::string>());
        } else {
            throw std::runtime_error("Target JSON mapping values must be strings or arrays of strings");
        }
        if (!deviceIds.empty()) {
            mapping.emplace(key, std::move(deviceIds));
        }
    }
    return mapping;
}

std::unordered_map<std::string, std::vector<std::string>> loadCsvTargetMap(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open target CSV mapping: " + path.string());
    }

    std::unordered_map<std::string, std::vector<std::string>> mapping;
    std::string line;
    std::size_t lineNumber = 0;

    while (std::getline(input, line)) {
        ++lineNumber;
        auto trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        std::stringstream ss(trimmed);
        std::string logical;
        std::string device;

        if (!std::getline(ss, logical, ',')) {
            continue;
        }
        if (!std::getline(ss, device)) {
            throw std::runtime_error("Target CSV mapping line " + std::to_string(lineNumber) + " missing device id");
        }

        logical = trimCopy(logical);
        device = trimCopy(device);

        if (logical.empty() || device.empty()) {
            continue;
        }

        const auto logicalLower = toLower(logical);
        const auto deviceLower = toLower(device);
        if ((logicalLower == "voice" || logicalLower == "logical") &&
            (deviceLower == "device" || deviceLower == "device_id")) {
            continue;
        }

        mapping[logical].push_back(device);
    }

    return mapping;
}

std::unordered_map<std::string, std::vector<std::string>> loadTargetMap(const std::filesystem::path& path) {
    if (path.empty()) {
        return {};
    }
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("Target mapping file not found: " + path.string());
    }

    auto ext = toLower(path.extension().string());
    if (ext == ".json") {
        return loadJsonTargetMap(path);
    }
    if (ext == ".csv") {
        return loadCsvTargetMap(path);
    }

    try {
        return loadJsonTargetMap(path);
    } catch (...) {
    }
    return loadCsvTargetMap(path);
}

void sendBundles(const std::vector<ScheduledBundle>& bundles, const SchedulerConfig& config) {
    asio::io_context ioContext;
    asio::ip::address address;
    try {
        address = asio::ip::make_address(config.host);
    } catch (const std::exception& ex) {
        throw std::runtime_error("Invalid destination address: " + config.host + " (" + ex.what() + ")");
    }

    acoustics::osc::OscSender sender(
        ioContext,
        acoustics::osc::OscSender::Endpoint(address, config.port),
        config.broadcast);

    if (config.encryptOsc) {
        if (!config.oscKey || !config.oscIv) {
            throw std::runtime_error("OSC encryption enabled without key/iv material");
        }
        sender.enableEncryption(*config.oscKey, *config.oscIv);
    }

    if (config.bundleSpacing < 0.01) {
        throw std::runtime_error("--bundle-spacing must be at least 0.01 seconds");
    }

    for (std::size_t i = 0; i < bundles.size(); ++i) {
        sender.send(bundles[i].toOscBundle());
        if (config.bundleSpacing > 0.0 && i + 1 < bundles.size()) {
            std::this_thread::sleep_for(std::chrono::duration<double>(config.bundleSpacing));
        }
    }
}

}  // namespace

SchedulerReport SchedulerController::execute(const SchedulerConfig& config) const {
    if (config.timelinePath.empty()) {
        throw std::runtime_error("Timeline path is required");
    }

    auto timeline = SoundTimeline::fromJsonFile(config.timelinePath);

    double leadTime = config.leadTimeOverride >= 0.0
                          ? config.leadTimeOverride
                          : timeline.defaultLeadTimeSeconds();
    if (config.leadTimeOverride >= 0.0 && leadTime < kMinimumLeadTimeSeconds) {
        throw std::runtime_error("Override lead time must be at least 3 seconds");
    }

    auto resolver = buildResolver(config);
    auto baseTime = config.baseTime.value_or(std::chrono::system_clock::now());
    auto scheduled = timeline.schedule(baseTime, leadTime, resolver);

    if (!config.dryRun) {
        sendBundles(scheduled, config);
    }

    SchedulerReport report;
    report.bundles = std::move(scheduled);
    return report;
}

std::chrono::system_clock::time_point SchedulerController::parseBaseTime(const std::string& value) {
    if (value.empty()) {
        return std::chrono::system_clock::now();
    }

    std::string mutableValue = value;
    int offsetMinutes = 0;

    auto dotPos = mutableValue.find('.');
    std::string fractionalPart;
    if (dotPos != std::string::npos) {
        fractionalPart = mutableValue.substr(dotPos + 1);
        mutableValue = mutableValue.substr(0, dotPos);
    }

    if (!mutableValue.empty() && mutableValue.back() == 'Z') {
        mutableValue.pop_back();
    }

    const auto timeSepPos = mutableValue.find('T');
    std::size_t tzPos = std::string::npos;
    if (timeSepPos != std::string::npos) {
        const auto plusPos = mutableValue.find_last_of('+');
        if (plusPos != std::string::npos && plusPos > timeSepPos) {
            tzPos = plusPos;
        } else {
            const auto minusPos = mutableValue.find_last_of('-');
            if (minusPos != std::string::npos && minusPos > timeSepPos) {
                tzPos = minusPos;
            }
        }
    }

    if (tzPos != std::string::npos) {
        const char sign = mutableValue[tzPos];
        std::string offsetPart = mutableValue.substr(tzPos + 1);
        mutableValue = mutableValue.substr(0, tzPos);

        int hours = 0;
        int minutes = 0;
        if (offsetPart.size() == 5 && offsetPart[2] == ':') {
            hours = std::stoi(offsetPart.substr(0, 2));
            minutes = std::stoi(offsetPart.substr(3, 2));
        } else if (offsetPart.size() == 4) {
            hours = std::stoi(offsetPart.substr(0, 2));
            minutes = std::stoi(offsetPart.substr(2, 2));
        } else if (offsetPart.size() == 2) {
            hours = std::stoi(offsetPart);
            minutes = 0;
        } else if (!offsetPart.empty()) {
            throw std::runtime_error("Unsupported timezone offset format in base time");
        }

        offsetMinutes = hours * 60 + minutes;
        if (sign == '+') {
            offsetMinutes = +offsetMinutes;
        } else if (sign == '-') {
            offsetMinutes = -offsetMinutes;
        } else {
            throw std::runtime_error("Unexpected timezone indicator in base time");
        }
    }

    std::tm tm{};
    std::istringstream iss(mutableValue);
    iss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
    if (iss.fail()) {
        throw std::runtime_error("Failed to parse base time. Expected format YYYY-MM-DDTHH:MM:SS[.fff][Z]");
    }

    tm.tm_isdst = -1;

#if defined(_WIN32)
    std::time_t epoch = _mkgmtime(&tm);
#else
    std::time_t epoch = timegm(&tm);
#endif
    if (epoch == static_cast<std::time_t>(-1)) {
        throw std::runtime_error("Failed to convert base time to UTC");
    }

    auto tp = std::chrono::system_clock::from_time_t(epoch);
    if (offsetMinutes != 0) {
        tp -= std::chrono::minutes(offsetMinutes);
    }

    if (!fractionalPart.empty()) {
        double fraction = std::stod("0." + fractionalPart);
        tp += std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double>(fraction)
        );
    }
    return tp;
}

TargetResolver SchedulerController::buildResolver(const SchedulerConfig& config) const {
    TargetResolver resolver;
    if (!config.targetMapPath.empty()) {
        auto mapping = loadTargetMap(config.targetMapPath);
        resolver.setMapping(std::move(mapping));
    }
    if (!config.defaultTargets.empty()) {
        resolver.setDefaultTargets(config.defaultTargets);
    }
    return resolver;
}

}  // namespace acoustics::scheduler
