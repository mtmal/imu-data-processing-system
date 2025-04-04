#include <csignal>
#include <iostream>
#include <spdlog/spdlog.h>

#include "IMUSubscriber.h"
#include "utils.h"

sem_t semaphore;

void printUsage(const char* programName)
{
    std::cout << "Usage: " << programName << " --socket-path <path> --log-level <level> --frequency-hz <freq>\n"
              << "Options:\n"
              << "  --socket-path  : Unix domain socket path\n"
              << "  --log-level    : Logging level (TRACE, DEBUG, INFO, WARN, ERROR)\n"
              << "  --timeout-ms   : Timeout in ms\n";
}

void signalHandler(int signum)
{
    spdlog::info("Received signal: {}, stopping gracefully", signum);
    sem_post(&semaphore);
}

int main(int argc, char* argv[])
{
    IMUSubscriber subscriber;
    Parameters params;
    sem_init(&semaphore, 0, 0);

    // Set default logger level
    setupLogger("INFO");

    // Parse command line arguments
    if (!parseParameters(argc, argv, params))
    {
        spdlog::error("Failed to parse parameters");
        printUsage(argv[0]);
        return 1;
    }

    // register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    signal(SIGALRM, signalHandler);

    spdlog::info("Initialising IMU Subscriber");
    if (subscriber.initialise(params))
    {
        spdlog::info("Starting IMU Subscriber");
        subscriber.startThread();
        spdlog::info("IMU Subscriber is running. Press Ctrl+C to stop.");

        // Main loop can be empty, as the thread handles the publishing
        sem_wait(&semaphore);

        subscriber.stopThread(true);
        spdlog::info("Subscriber stopped");
    }
    else
    {
        spdlog::error("Failed to initialise IMU Subscriber");
    }
    sem_destroy(&semaphore);
    return 0;
}
