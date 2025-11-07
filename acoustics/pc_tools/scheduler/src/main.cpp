#include "acoustics/osc/OscPacket.h"
#include "acoustics/scheduler/SchedulerController.h"

#include "CLI11.hpp"
#include "json.hpp"

#include <array>
#include <chrono>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace {

using acoustics::scheduler::ScheduledBundle;
using json = nlohmann::json;

struct OscMaterial {
    acoustics::osc::OscEncryptor::Key256 key;
    acoustics::osc::OscEncryptor::Iv128 iv;
};

struct SchedulerOptions {
    std::filesystem::path timelinePath;
    std::string host{"255.255.255.255"};
    std::uint16_t port{9000};
    double leadTime{-1.0};
    double spacing{0.01};
    bool broadcast{true};
    bool dryRun{false};
    std::string baseTimeIso;
    std::filesystem::path targetMapPath;
    std::vector<std::string> defaultTargets;
    std::filesystem::path oscConfigPath{"acoustics/secrets/osc_config.json"};
    acoustics::osc::OscEncryptor::Key256 oscKey{};
    acoustics::osc::OscEncryptor::Iv128 oscIv{};
};

template <std::size_t N>
std::array<std::uint8_t, N> parseHexBytes(const std::string& text) {
    std::string sanitized;
    sanitized.reserve(text.size());
    for (char ch : text) {
        if (std::isspace(static_cast<unsigned char>(ch))) {
            continue;
        }
        sanitized.push_back(ch);
    }

    if (sanitized.size() >= 2 &&
        sanitized[0] == '0' &&
        (sanitized[1] == 'x' || sanitized[1] == 'X')) {
        sanitized.erase(0, 2);
    }

    if (sanitized.size() != N * 2) {
        throw std::runtime_error("Expected " + std::to_string(N * 2) + " hex characters, got " + std::to_string(sanitized.size()));
    }

    std::array<std::uint8_t, N> bytes{};
    for (std::size_t i = 0; i < N; ++i) {
        std::string byteStr = sanitized.substr(i * 2, 2);
        if (!std::isxdigit(static_cast<unsigned char>(byteStr[0])) ||
            !std::isxdigit(static_cast<unsigned char>(byteStr[1]))) {
            throw std::runtime_error("Invalid hex characters in key/iv");
        }
        bytes[i] = static_cast<std::uint8_t>(std::stoul(byteStr, nullptr, 16));
    }
    return bytes;
}

void printArgument(const acoustics::osc::Argument& arg) {
    std::visit([](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, acoustics::osc::Blob>) {
            std::cout << "<blob:" << value.size() << ">";
        } else if constexpr (std::is_same_v<T, bool>) {
            std::cout << (value ? "true" : "false");
        } else {
            std::cout << value;
        }
    }, arg);
}

void printBundle(const ScheduledBundle& bundle) {
    auto tt = std::chrono::system_clock::to_time_t(bundle.executionTime);
    auto localTm = *std::localtime(&tt);
    std::cout << "Bundle @ " << std::put_time(&localTm, "%Y-%m-%d %H:%M:%S")
              << " (messages=" << bundle.messages.size() << ")\n";

    for (const auto& msg : bundle.messages) {
        std::cout << "  " << msg.message.address
                  << " target=" << (msg.targetId ? *msg.targetId : "<broadcast>")
                  << " preset=" << (msg.presetId ? *msg.presetId : "-")
                  << " args=[";
        for (std::size_t i = 0; i < msg.message.arguments.size(); ++i) {
            printArgument(msg.message.arguments[i]);
            if (i + 1 < msg.message.arguments.size()) {
                std::cout << ", ";
            }
        }
        std::cout << "]\n";
    }
}

OscMaterial loadOscMaterial(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("OSC config not found: " + path.string());
    }

    json root;
    try {
        input >> root;
    } catch (const std::exception& ex) {
        throw std::runtime_error("Failed to parse OSC config (" + path.string() + "): " + ex.what());
    }

    if (!root.contains("osc")) {
        throw std::runtime_error("OSC config missing 'osc' object: " + path.string());
    }
    const auto& oscNode = root.at("osc");
    if (!oscNode.is_object()) {
        throw std::runtime_error("'osc' must be an object in " + path.string());
    }

    const auto keyIt = oscNode.find("key_hex");
    const auto ivIt = oscNode.find("iv_hex");
    if (keyIt == oscNode.end() || !keyIt->is_string()) {
        throw std::runtime_error("'osc.key_hex' missing or not a string in " + path.string());
    }
    if (ivIt == oscNode.end() || !ivIt->is_string()) {
        throw std::runtime_error("'osc.iv_hex' missing or not a string in " + path.string());
    }

    OscMaterial material;
    material.key = parseHexBytes<32>(keyIt->get<std::string>());
    material.iv = parseHexBytes<16>(ivIt->get<std::string>());
    return material;
}

}  // namespace

int main(int argc, char** argv) {
    CLI::App app{"Agent A Timeline Scheduler"};
    SchedulerOptions opts;

    app.add_option("timeline", opts.timelinePath, "Timeline JSON file")
        ->required()
        ->check(CLI::ExistingFile);
    app.add_option("--host", opts.host, "Destination host (IPv4)");
    app.add_option("--port", opts.port, "Destination port");
    app.add_option("--lead-time", opts.leadTime, "Override lead time in seconds");
    app.add_option("--bundle-spacing", opts.spacing, "Delay between bundle sends (seconds)");
    app.add_option("--target-map", opts.targetMapPath, "Logical-to-device mapping file (JSON or CSV)");
    app.add_option("--default-targets", opts.defaultTargets, "Fallback device IDs when events omit targets")
        ->delimiter(',')
        ->take_last();
    bool disableBroadcast = false;
    app.add_flag("--no-broadcast", disableBroadcast, "Disable broadcast socket option");
    app.add_flag("--dry-run", opts.dryRun, "Print bundles instead of sending");
    app.add_option("--base-time", opts.baseTimeIso, "Base ISO8601 time (default: now)");
    app.add_option("--osc-config", opts.oscConfigPath, "OSC secrets JSON file (default: acoustics/secrets/osc_config.json)");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    opts.broadcast = !disableBroadcast;

    try {
        const auto material = loadOscMaterial(opts.oscConfigPath);
        opts.oscKey = material.key;
        opts.oscIv = material.iv;
    } catch (const std::exception& ex) {
        std::cerr << "Failed to load OSC config: " << ex.what() << "\n";
        std::cerr << "Provide a valid osc_config.json via --osc-config.\n";
        return 1;
    }

    try {
        acoustics::scheduler::SchedulerConfig config;
        config.timelinePath = opts.timelinePath;
        config.host = opts.host;
        config.port = opts.port;
        config.leadTimeOverride = opts.leadTime;
        config.bundleSpacing = opts.spacing;
        config.broadcast = opts.broadcast;
        config.dryRun = opts.dryRun;
        if (!opts.baseTimeIso.empty()) {
            config.baseTime = acoustics::scheduler::SchedulerController::parseBaseTime(opts.baseTimeIso);
        }
        config.targetMapPath = opts.targetMapPath;
        config.defaultTargets = opts.defaultTargets;
        config.encryptOsc = true;
        config.oscKey = opts.oscKey;
        config.oscIv = opts.oscIv;

        acoustics::scheduler::SchedulerController controller;
        auto report = controller.execute(config);

        if (opts.dryRun) {
            std::cout << "DRY RUN: Generated " << report.bundles.size() << " bundle(s)\n";
            for (const auto& bundle : report.bundles) {
                printBundle(bundle);
            }
            return 0;
        }

        std::cout << "Sent " << report.bundles.size() << " bundle(s) to "
                  << opts.host << ":" << opts.port << "\n";
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
