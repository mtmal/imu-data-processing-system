#include <iostream>
#include <semaphore.h>
#include <signal.h>
#include <spdlog/spdlog.h>

#include "communication/IMUPublisher.h"
#include "core/Parameters.h"
#include "providers/RandomIMUDataProvider.h"
#include "utils/utils.h"

sem_t sem_waiter;

void printUsage(const char* programName)
{
    std::cout << "Usage: " << programName << " --socket-path <path> [options]\n"
              << "Options:\n"
              << "  --socket-path  : Unix domain socket path\n"
              << "  --log-level    : Logging level (TRACE, DEBUG, INFO, WARN, ERROR)\n"
              << "  --frequency-hz : Publication frequency in Hz\n"
              << "  --real-time    : Enable real-time thread configuration\n"
              << "  --priority     : Thread priority (1-99, only with --real-time)\n"
              << "  --policy       : Scheduling policy (FIFO or RR, only with --real-time)\n";
}

void signalHandler(int signum)
{
    spdlog::info("Received signal: {}, stopping gracefully", signum);
    sem_post(&sem_waiter);
}

int main(int argc, char* argv[])
{
    // Create the random data provider
    RandomIMUDataProvider dataProvider;
    
    // Create the publisher with the data provider and real-time setting
    IMUPublisher publisher(dataProvider);
    
    Parameters params;
    sem_init(&sem_waiter, 0, 0);

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

    spdlog::info("Initialising IMU Publisher");
    if (publisher.initialise(params))
    {
        spdlog::info("Starting IMU Publisher");
        publisher.startThread();
        spdlog::info("IMU Publisher is running. Press Ctrl+C to stop.");

        // Main loop can be empty, as the thread handles the publishing
        sem_wait(&sem_waiter);

        publisher.stopThread();
        spdlog::info("IMU Publisher stopped");
    }
    else
    {
        spdlog::error("Failed to initialise IMU Publisher");
    }
    sem_destroy(&sem_waiter);
    return 0;
}
