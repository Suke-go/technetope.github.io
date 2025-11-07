#include "acoustics/common/DeviceRegistry.h"
#include "acoustics/osc/OscPacket.h"
#include "acoustics/osc/OscTransport.h"

#include "CLI11.hpp"

#include <asio.hpp>
#include <spdlog/spdlog.h>

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace {

std::atomic_bool g_shouldStop{false};

void handleSignal(int) {
    g_shouldStop.store(true);
}

struct MonitorOptions {
    std::string listenHost{"0.0.0.0"};
    std::uint16_t port{19100};
    std::optional<std::filesystem::path> csv;
    std::uint64_t maxPackets{0};
    bool quiet{false};
    bool debug{false};
    std::filesystem::path registryPath{"state/devices.json"};
};

struct DeviceStats {
    std::uint64_t count{0};
    double meanMs{0.0};
    double m2{0.0};
};

std::chrono::system_clock::time_point secondsToTimePoint(double seconds) {
    auto secs = static_cast<std::int64_t>(seconds);
    double fractional = seconds - static_cast<double>(secs);
    auto tp = std::chrono::system_clock::time_point{std::chrono::seconds{secs}} +
              std::chrono::duration_cast<std::chrono::system_clock::duration>(
                  std::chrono::duration<double>(fractional));
    return tp;
}

double toEpochSeconds(std::chrono::system_clock::time_point tp) {
    auto duration = tp.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

double argumentToSeconds(const acoustics::osc::Argument& arg) {
    if (const auto* f = std::get_if<float>(&arg)) {
        return static_cast<double>(*f);
    }
    if (const auto* i = std::get_if<std::int32_t>(&arg)) {
        return static_cast<double>(*i);
    }
    throw std::runtime_error("Unsupported timestamp argument type");
}

void updateStats(DeviceStats& stats, double sampleMs) {
    ++stats.count;
    double delta = sampleMs - stats.meanMs;
    stats.meanMs += delta / static_cast<double>(stats.count);
    double delta2 = sampleMs - stats.meanMs;
    stats.m2 += delta * delta2;
}

std::ofstream openCsv(const std::filesystem::path& path) {
    const bool exists = std::filesystem::exists(path);
    std::ofstream out(path, std::ios::app);
    if (!out) {
        throw std::runtime_error("Failed to open CSV file: " + path.string());
    }
    if (!exists) {
        out << "arrival_iso,device_id,sequence,latency_ms,sent_iso\n";
    }
    return out;
}

struct HeartbeatData {
    std::string deviceId;
    std::int32_t sequence;
    double sentSeconds;
    std::optional<std::int32_t> queueSize;
    std::optional<bool> isPlaying;
};

std::string describeArgument(const acoustics::osc::Argument& arg) {
    return std::visit(
        [](const auto& value) -> std::string {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, std::int32_t>) {
                return "int32(" + std::to_string(value) + ")";
            } else if constexpr (std::is_same_v<T, float>) {
                return "float(" + std::to_string(value) + ")";
            } else if constexpr (std::is_same_v<T, std::string>) {
                return "string(\"" + value + "\")";
            } else if constexpr (std::is_same_v<T, bool>) {
                return std::string("bool(") + (value ? "true" : "false") + ")";
            } else if constexpr (std::is_same_v<T, acoustics::osc::Blob>) {
                return "blob(size=" + std::to_string(value.size()) + ")";
            }
            return "unknown";
        },
        arg);
}

HeartbeatData parseHeartbeat(const acoustics::osc::Message& message) {
    if (message.address != "/heartbeat" || message.arguments.size() < 3) {
        throw std::runtime_error("Not a heartbeat message");
    }
    HeartbeatData data;
    if (const auto* id = std::get_if<std::string>(&message.arguments[0])) {
        data.deviceId = *id;
    } else {
        throw std::runtime_error("Heartbeat device id must be a string");
    }
    if (const auto* seq = std::get_if<std::int32_t>(&message.arguments[1])) {
        data.sequence = *seq;
    } else {
        throw std::runtime_error("Heartbeat sequence must be int32");
    }

    if (message.arguments.size() >= 4 &&
        std::holds_alternative<std::int32_t>(message.arguments[2]) &&
        std::holds_alternative<std::int32_t>(message.arguments[3])) {
        auto secs = std::get<std::int32_t>(message.arguments[2]);
        auto micros = std::get<std::int32_t>(message.arguments[3]);
        data.sentSeconds = static_cast<double>(secs) +
                          static_cast<double>(micros) / 1'000'000.0;
    } else {
        data.sentSeconds = argumentToSeconds(message.arguments[2]);
    }

    if (message.arguments.size() >= 5) {
        if (const auto* queue = std::get_if<std::int32_t>(&message.arguments[4])) {
            data.queueSize = *queue;
        }
    }

    if (message.arguments.size() >= 6) {
        if (const auto* playingBool = std::get_if<bool>(&message.arguments[5])) {
            data.isPlaying = *playingBool;
        } else if (const auto* playingInt = std::get_if<std::int32_t>(&message.arguments[5])) {
            data.isPlaying = (*playingInt != 0);
        } else if (const auto* playingFloat = std::get_if<float>(&message.arguments[5])) {
            data.isPlaying = (*playingFloat != 0.0f);
        }
    }
    return data;
}

void emitSample(std::ostream& out,
                const HeartbeatData& data,
                double latencyMs,
                std::chrono::system_clock::time_point arrival) {
    auto arrivalTimeT = std::chrono::system_clock::to_time_t(arrival);
    auto arrivalLocal = *std::localtime(&arrivalTimeT);
    auto sentTimeT = std::chrono::system_clock::to_time_t(secondsToTimePoint(data.sentSeconds));
    auto sentLocal = *std::localtime(&sentTimeT);

    out << std::put_time(&arrivalLocal, "%Y-%m-%d %H:%M:%S")
        << "," << data.deviceId
        << "," << data.sequence
        << "," << std::fixed << std::setprecision(3) << latencyMs
        << "," << std::put_time(&sentLocal, "%Y-%m-%d %H:%M:%S")
        << '\n';
}

void processMessage(const acoustics::osc::Message& message,
                    const MonitorOptions& options,
                    std::unordered_map<std::string, DeviceStats>& stats,
                    std::ofstream* csvStream,
                    acoustics::common::DeviceRegistry* registry) {
    spdlog::debug("processMessage: address={} arg_count={}", message.address, message.arguments.size());

    HeartbeatData data;
    try {
        data = parseHeartbeat(message);
    } catch (const std::exception& ex) {
        std::ostringstream argStream;
        for (std::size_t i = 0; i < message.arguments.size(); ++i) {
            if (i > 0) {
                argStream << ", ";
            }
            argStream << describeArgument(message.arguments[i]);
        }
        spdlog::warn("Failed to parse heartbeat: {} (address={} args=[{}])",
                     ex.what(),
                     message.address,
                     argStream.str());
        return;
    }

    auto arrival = std::chrono::system_clock::now();
    double arrivalSeconds = toEpochSeconds(arrival);
    double latencyMs = (arrivalSeconds - data.sentSeconds) * 1000.0;

    auto& deviceStats = stats[data.deviceId];
    updateStats(deviceStats, latencyMs);
    if (data.queueSize || data.isPlaying.has_value()) {
        spdlog::debug("Heartbeat parsed: id={} seq={} sent_seconds={:.6f} latency_ms={:.3f} count={} queue={} playing={}",
                      data.deviceId,
                      data.sequence,
                      data.sentSeconds,
                      latencyMs,
                      deviceStats.count,
                      data.queueSize ? std::to_string(*data.queueSize) : "n/a",
                      data.isPlaying.has_value() ? (data.isPlaying.value() ? "yes" : "no") : "n/a");
    } else {
        spdlog::debug("Heartbeat parsed: id={} seq={} sent_seconds={:.6f} latency_ms={:.3f} count={}",
                      data.deviceId,
                      data.sequence,
                      data.sentSeconds,
                      latencyMs,
                      deviceStats.count);
    }

    if (registry) {
        registry->recordHeartbeat(data.deviceId, latencyMs, arrival);
    }

    if (!options.quiet) {
        std::cout << "[" << data.deviceId << "] seq=" << data.sequence
                  << " latency=" << std::fixed << std::setprecision(3) << latencyMs << " ms";
        if (data.queueSize) {
            std::cout << " queue=" << *data.queueSize;
        }
        if (data.isPlaying.has_value()) {
            std::cout << " playing=" << (data.isPlaying.value() ? "yes" : "no");
        }
        std::cout << std::endl;
    }

    if (csvStream) {
        emitSample(*csvStream, data, latencyMs, arrival);
        csvStream->flush();
    }
}

void processAnnounce(const acoustics::osc::Message& message,
                     const MonitorOptions& options,
                     acoustics::common::DeviceRegistry& registry) {
    if (message.arguments.empty()) {
        if (!options.quiet) {
            std::cerr << "Announce message missing arguments" << std::endl;
        }
        return;
    }

    auto getStringArg = [&](std::size_t index) -> std::optional<std::string> {
        if (index >= message.arguments.size()) {
            return std::nullopt;
        }
        if (const auto* value = std::get_if<std::string>(&message.arguments[index])) {
            return *value;
        }
        return std::nullopt;
    };

    auto looksLikeMac = [](const std::string& text) {
        return text.find(':') != std::string::npos;
    };

    std::optional<std::string> deviceId = getStringArg(0);
    if (!deviceId) {
        if (!options.quiet) {
            std::cerr << "Announce first argument must be string" << std::endl;
        }
        return;
    }

    std::optional<std::string> macArg;
    std::size_t nextIndex = 1;

    if (looksLikeMac(*deviceId)) {
        macArg = deviceId;
        deviceId = std::nullopt;
        auto maybeSecond = getStringArg(1);
        if (maybeSecond && !looksLikeMac(*maybeSecond)) {
            deviceId = maybeSecond;
            nextIndex = 2;
        }
    } else {
        macArg = getStringArg(1);
        nextIndex = 2;
    }

    if (!macArg) {
        if (!options.quiet) {
            std::cerr << "Announce message missing MAC address" << std::endl;
        }
        return;
    }

    std::string fwVersion;
    if (auto maybeFw = getStringArg(nextIndex)) {
        fwVersion = *maybeFw;
        ++nextIndex;
    }

    std::optional<std::string> alias;
    if (auto maybeAlias = getStringArg(nextIndex)) {
        alias = *maybeAlias;
    }
    if (!alias && deviceId) {
        alias = *deviceId;
    }

    auto now = std::chrono::system_clock::now();
    auto state = registry.registerAnnounce(*macArg, fwVersion, alias, now);
    if (!options.quiet) {
        std::cout << "ANNOUNCE id=" << (deviceId ? *deviceId : state.id)
                  << " mac=" << state.mac
                  << " fw=" << state.firmwareVersion;
        if (state.alias) {
            std::cout << " alias=" << *state.alias;
        }
        std::cout << std::endl;
    }
}

void processPacket(const acoustics::osc::Packet& packet,
                   const MonitorOptions& options,
                   std::unordered_map<std::string, DeviceStats>& stats,
                   std::ofstream* csvStream,
                   acoustics::common::DeviceRegistry* registry) {
    auto handle = [&](const acoustics::osc::Message& msg) {
        if (spdlog::should_log(spdlog::level::debug)) {
            std::ostringstream argStream;
            for (std::size_t i = 0; i < msg.arguments.size(); ++i) {
                if (i > 0) {
                    argStream << ", ";
                }
                argStream << describeArgument(msg.arguments[i]);
            }
            spdlog::debug("processPacket: dispatching address={} args=[{}]", msg.address, argStream.str());
        }
        if (registry && msg.address == "/announce") {
            processAnnounce(msg, options, *registry);
            return;
        }
        processMessage(msg, options, stats, csvStream, registry);
    };

    if (const auto* message = std::get_if<acoustics::osc::Message>(&packet)) {
        handle(*message);
    } else if (const auto* bundle = std::get_if<acoustics::osc::Bundle>(&packet)) {
        spdlog::debug("processPacket: bundle with {} elements", bundle->elements.size());
        for (const auto& msg : bundle->elements) {
            handle(msg);
        }
    }
}

void printSummary(const std::unordered_map<std::string, DeviceStats>& stats) {
    if (stats.empty()) {
        std::cout << "No heartbeat samples captured." << std::endl;
        return;
    }

    std::cout << "\nLatency summary (ms):\n";
    std::cout << std::left << std::setw(20) << "Device"
              << std::right << std::setw(10) << "Count"
              << std::setw(15) << "Mean"
              << std::setw(15) << "StdDev" << '\n';

    for (const auto& [device, stat] : stats) {
        double stddev = 0.0;
        if (stat.count > 1) {
            stddev = std::sqrt(stat.m2 / static_cast<double>(stat.count - 1));
        }
        std::cout << std::left << std::setw(20) << device
                  << std::right << std::setw(10) << stat.count
                  << std::setw(15) << std::fixed << std::setprecision(3) << stat.meanMs
                  << std::setw(15) << std::fixed << std::setprecision(3) << stddev
                  << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    CLI::App app{"Agent A Heartbeat Monitor"};
    MonitorOptions options;

    app.add_option("--host", options.listenHost, "Listen address (IPv4)");
    app.add_option("--port", options.port, "Listen port");
    app.add_option("--csv", options.csv, "Append results to CSV file");
    app.add_option("--count", options.maxPackets, "Stop after N packets (0 = unlimited)");
    app.add_flag("--quiet", options.quiet, "Suppress console output");
    app.add_flag("--debug", options.debug, "Enable verbose debug logging");
    app.add_option("--registry", options.registryPath, "Device registry JSON path");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    spdlog::set_pattern("%Y-%m-%d %H:%M:%S.%e [%^%l%$] %v");
    if (options.debug) {
        spdlog::set_level(spdlog::level::debug);
        spdlog::debug("Debug logging enabled");
    } else {
        spdlog::set_level(spdlog::level::info);
    }

    std::signal(SIGINT, handleSignal);

    try {
        std::unique_ptr<std::ofstream> csvStream;
        if (options.csv.has_value()) {
            csvStream = std::make_unique<std::ofstream>(openCsv(*options.csv));
        }

        acoustics::common::DeviceRegistry registry(options.registryPath);
        registry.load();

        std::unordered_map<std::string, DeviceStats> stats;
        std::mutex stateMutex;
        std::atomic_uint64_t processed{0};

        asio::ip::address listenAddress;
        try {
            listenAddress = asio::ip::make_address(options.listenHost);
        } catch (const std::exception& ex) {
            throw std::runtime_error("Invalid listen address: " + options.listenHost + " (" + ex.what() + ")");
        }

        acoustics::osc::IoContextRunner runner;
        acoustics::osc::OscListener listener(
            runner.context(),
            acoustics::osc::OscListener::Endpoint(listenAddress, options.port),
            [&](const acoustics::osc::Packet& packet, const acoustics::osc::OscListener::Endpoint&) {
                std::lock_guard lock(stateMutex);
                processPacket(packet,
                              options,
                              stats,
                              csvStream ? csvStream.get() : nullptr,
                              &registry);
                ++processed;
                if (options.maxPackets > 0 && processed.load() >= options.maxPackets) {
                    g_shouldStop.store(true);
                }
            });

        listener.start();
        runner.start();

        while (!g_shouldStop.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (options.maxPackets > 0 && processed.load() >= options.maxPackets) {
                break;
            }
        }

        listener.stop();
        runner.stop();

        if (!options.quiet) {
            std::lock_guard lock(stateMutex);
            printSummary(stats);
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}
