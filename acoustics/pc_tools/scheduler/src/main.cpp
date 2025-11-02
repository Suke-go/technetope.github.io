#include "acoustics/osc/OscTransport.h"
#include "acoustics/scheduler/SoundTimeline.h"

#include "CLI11.hpp"

#include <asio.hpp>

#include <chrono>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <variant>

namespace {

struct SchedulerOptions {
    std::string timelinePath;
    std::string host{"255.255.255.255"};
    std::uint16_t port{9000};
    double leadTime{-1.0};
    double spacing{0.01};
    bool broadcast{true};
    bool dryRun{false};
    std::string baseTimeIso;
};

std::chrono::system_clock::time_point parseIsoTime(std::string value) {
    if (value.empty()) {
        return std::chrono::system_clock::now();
    }

    if (value.back() == 'Z') {
        value.pop_back();
    }

    auto dotPos = value.find('.');
    std::string fractionalPart;
    if (dotPos != std::string::npos) {
        fractionalPart = value.substr(dotPos + 1);
        value = value.substr(0, dotPos);
    }

    std::tm tm{};
    std::istringstream iss(value);
    iss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
    if (iss.fail()) {
        throw std::runtime_error("Failed to parse base time. Expected format YYYY-MM-DDTHH:MM:SS[.fff][Z]");
    }

    auto tp = std::chrono::system_clock::from_time_t(std::mktime(&tm));

    if (!fractionalPart.empty()) {
        double fraction = std::stod("0." + fractionalPart);
        tp += std::chrono::duration_cast<std::chrono::system_clock::duration>(
            std::chrono::duration<double>(fraction)
        );
    }

    return tp;
}

void printBundle(const acoustics::osc::Bundle& bundle) {
    auto execTime = acoustics::osc::fromTimetag(bundle.timetag);
    auto tt = std::chrono::system_clock::to_time_t(execTime);
    auto localTm = *std::localtime(&tt);
    std::cout << "Bundle @ " << std::put_time(&localTm, "%Y-%m-%d %H:%M:%S")
              << " (elements=" << bundle.elements.size() << ")\n";
    for (const auto& msg : bundle.elements) {
        std::cout << "  " << msg.address << "(";
        for (std::size_t i = 0; i < msg.arguments.size(); ++i) {
            const auto& arg = msg.arguments[i];
            std::visit([
            ](const auto& value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<T, acoustics::osc::Blob>) {
                    std::cout << "<blob:" << value.size() << ">";
                } else if constexpr (std::is_same_v<T, bool>) {
                    std::cout << (value ? "true" : "false");
                } else {
                    std::cout << value;
                }
            }, arg);
            if (i + 1 < msg.arguments.size()) {
                std::cout << ", ";
            }
        }
        std::cout << ")\n";
    }
}

void sendBundles(const std::vector<acoustics::osc::Bundle>& bundles,
                 const SchedulerOptions& options) {
    asio::io_context ioContext;
    asio::ip::address address;
    try {
        address = asio::ip::make_address(options.host);
    } catch (const std::exception& ex) {
        throw std::runtime_error("Invalid destination address: " + options.host + " (" + ex.what() + ")");
    }

    acoustics::osc::OscSender sender(
        ioContext,
        acoustics::osc::OscSender::Endpoint(address, options.port),
        options.broadcast);

    for (std::size_t i = 0; i < bundles.size(); ++i) {
        sender.send(bundles[i]);
        if (options.spacing > 0.0 && i + 1 < bundles.size()) {
            std::this_thread::sleep_for(std::chrono::duration<double>(options.spacing));
        }
    }
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
    bool disableBroadcast = false;
    app.add_flag("--no-broadcast", disableBroadcast, "Disable broadcast socket option");
    app.add_flag("--dry-run", opts.dryRun, "Print bundles instead of sending");
    app.add_option("--base-time", opts.baseTimeIso, "Base ISO8601 time (default: now)");

    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    opts.broadcast = !disableBroadcast;

    try {
        auto timeline = acoustics::scheduler::SoundTimeline::fromJsonFile(opts.timelinePath);
        auto baseTime = parseIsoTime(opts.baseTimeIso);
        auto bundles = timeline.toBundles(baseTime,
                                          opts.leadTime >= 0.0 ? opts.leadTime : timeline.defaultLeadTimeSeconds());

        if (opts.dryRun) {
            std::cout << "DRY RUN: Generated " << bundles.size() << " bundle(s)\n";
            for (const auto& bundle : bundles) {
                printBundle(bundle);
            }
            return 0;
        }

        sendBundles(bundles, opts);
        std::cout << "Sent " << bundles.size() << " bundle(s) to " << opts.host << ":" << opts.port << "\n";
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
