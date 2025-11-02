#include "acoustics/common/DeviceRegistry.h"
#include "acoustics/osc/OscPacket.h"
#include "acoustics/osc/OscTransport.h"

#include "CLI11.hpp"

#include <asio.hpp>

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
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

std::atomic_bool g_shouldStop{false};

void handleSignal(int) {
    g_shouldStop.store(true);
}

struct MonitorOptions {
    std::string listenHost{"0.0.0.0"};
    std::uint16_t port{9100};
    std::optional<std::filesystem::path> csv;
    std::uint64_t maxPackets{0};
    bool quiet{false};
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
};

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
    HeartbeatData data;
    try {
        data = parseHeartbeat(message);
    } catch (const std::exception&) {
        return;
    }

    auto arrival = std::chrono::system_clock::now();
    double arrivalSeconds = toEpochSeconds(arrival);
    double latencyMs = (arrivalSeconds - data.sentSeconds) * 1000.0;

    updateStats(stats[data.deviceId], latencyMs);

    if (registry) {
        registry->recordHeartbeat(data.deviceId, latencyMs, arrival);
    }

    if (!options.quiet) {
        std::cout << "[" << data.deviceId << "] seq=" << data.sequence
                  << " latency=" << std::fixed << std::setprecision(3) << latencyMs << " ms" << std::endl;
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
        if (registry && msg.address == "/announce") {
            processAnnounce(msg, options, *registry);
            return;
        }
        processMessage(msg, options, stats, csvStream, registry);
    };

    if (const auto* message = std::get_if<acoustics::osc::Message>(&packet)) {
        handle(*message);
    } else if (const auto* bundle = std::get_if<acoustics::osc::Bundle>(&packet)) {
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
    app.add_option("--registry", options.registryPath, "Device registry JSON path");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
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
