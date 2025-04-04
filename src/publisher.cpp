#include <getopt.h>
#include <iostream>
#include <spdlog/spdlog.h>
#include <signal.h>
#include "IMUPublisher.h"

std::atomic<bool> running(true);

void printUsage(const char* programName)
{
    std::cout << "Usage: " << programName << " --socket-path <path> --log-level <level> --frequency-hz <freq>\n"
              << "Options:\n"
              << "  --socket-path  : Unix domain socket path\n"
              << "  --log-level    : Logging level (TRACE, DEBUG, INFO, WARN, ERROR)\n"
              << "  --frequency-hz : Publication frequency in Hz\n";
}

bool parseParameters(int argc, char* argv[], std::string& socketPath, std::string& logLevel, int& frequencyHz)
{
    constexpr struct option long_options[] = {
        {"socket-path", required_argument, 0, 's'},
        {"log-level", required_argument, 0, 'l'},
        {"frequency-hz", required_argument, 0, 'f'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "s:l:f:", long_options, nullptr)) != -1)
    {
        switch (opt)
        {
            case 's':
                socketPath = optarg;
                break;
            case 'l':
                logLevel = optarg;
                break;
            case 'f':
                frequencyHz = std::stoi(optarg);
                break;
            default:
                return false;
        }
    }
    return true;
}

void setupLogger(const std::string& logLevel)
{
    if (logLevel == "TRACE")
    {
        spdlog::set_level(spdlog::level::trace);
    }
    else if (logLevel == "DEBUG")
    {
        spdlog::set_level(spdlog::level::debug);
    }
    else if (logLevel == "INFO")
    {
        spdlog::set_level(spdlog::level::info);
    }
    else if (logLevel == "WARN")
    {
        spdlog::set_level(spdlog::level::warn);
    }
    else if (logLevel == "ERROR")
    {
        spdlog::set_level(spdlog::level::err);
    }
    else
    {
        spdlog::set_level(spdlog::level::info);
    }
}

void signalHandler(int signum)
{
    spdlog::info("Received signal: {}, stopping gracefully", signum);
    running = false;
}

int main(int argc, char* argv[])
{
    IMUPublisher publisher;
    std::string socketPath;
    std::string logLevel = "INFO";
    int frequencyHz = 500;

    if (!parseParameters(argc, argv, socketPath, logLevel, frequencyHz))
    {
        spdlog::error("Failed to parse parameters");
        printUsage(argv[0]);
        return 1;
    }

    if (socketPath.empty())
    {
        spdlog::error("Socket path is required");
        printUsage(argv[0]);
        return 1;
    }

    // Setup logging
    setupLogger(logLevel);

    // register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    spdlog::info("Starting IMU Publisher");
    spdlog::info("Socket path: {}", socketPath);
    spdlog::info("Frequency: {} Hz", frequencyHz);

    publisher.initialise(socketPath, frequencyHz);
    publisher.startThread();

    spdlog::info("IMU Publisher is running. Press Ctrl+C to stop.");
    while (publisher.isRunning() && running)
    {
        // Main loop can be empty, as the thread handles the publishing
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    publisher.stopThread(true);
    spdlog::info("Publisher stopped");
    return 0;
}
