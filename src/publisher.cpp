#include <getopt.h>
#include <iostream>
#include <spdlog/spdlog.h>
#include <signal.h>

#include "IMUPublisher.h"
#include "utils.h"

std::atomic<bool> running(true);

void printUsage(const char* programName)
{
    std::cout << "Usage: " << programName << " --socket-path <path> --log-level <level> --frequency-hz <freq>\n"
              << "Options:\n"
              << "  --socket-path  : Unix domain socket path\n"
              << "  --log-level    : Logging level (TRACE, DEBUG, INFO, WARN, ERROR)\n"
              << "  --frequency-hz : Publication frequency in Hz\n";
}

void signalHandler(int signum)
{
    spdlog::info("Received signal: {}, stopping gracefully", signum);
    running = false;
}

int main(int argc, char* argv[])
{
    IMUPublisher publisher;
    Parameters params;
    setupLogger("INFO");

    if (!parseParameters(argc, argv, params))
    {
        spdlog::error("Failed to parse parameters");
        printUsage(argv[0]);
        return 1;
    }

    if (params.mSocketPath.empty())
    {
        spdlog::error("Socket path is required");
        printUsage(argv[0]);
        return 1;
    }

    // register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    spdlog::info("Starting IMU Publisher");

    publisher.initialise(params);
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
