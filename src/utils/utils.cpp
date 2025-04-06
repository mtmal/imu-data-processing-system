#include <getopt.h>
#include <spdlog/spdlog.h>
#include <sched.h>

#include "core/Parameters.h"
#include "utils/utils.h"


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

bool parseParameters(int argc, char* argv[], Parameters& params)
{
    /** List of available program options */
    constexpr struct option long_options[] = {
        {"socket-path", required_argument, 0, 's'},
        {"log-level", required_argument, 0, 'l'},
        {"frequency-hz", required_argument, 0, 'f'},
        {"timeout-ms", required_argument, 0, 't'},
        {"ahrs-type", required_argument, 0, 'a'},
        {"real-time", no_argument, 0, 'r'},
        {"priority", required_argument, 0, 'p'},
        {"policy", required_argument, 0, 'P'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "s:l:f:t:a:rp:P:", long_options, nullptr)) != -1)
    {
        switch (opt)
        {
            case 's':
                params.mSocketPath = optarg;
                spdlog::info("Socket path: {}", params.mSocketPath);
                break;
            case 'l':
                setupLogger(optarg);
                break;
            case 'f':
                params.mFrequencyHz = std::stoi(optarg);
                spdlog::info("Frequency: {} Hz", params.mFrequencyHz);
                break;
            case 't':
                params.mTimeoutMs = std::stoi(optarg);
                spdlog::info("Timeout: {} ms", params.mTimeoutMs);
                break;
            case 'a':
                if (std::string(optarg) == "madgwick")
                {
                    params.mAhrsType = AHRSType::MADGWICK;
                    spdlog::info("AHRS: Madgwick algorithm");
                }
                else if (std::string(optarg) == "simple")
                {
                    params.mAhrsType = AHRSType::SIMPLE;
                    spdlog::info("AHRS: Simple algorithm");
                }
                else
                {
                    params.mAhrsType = AHRSType::NONE;
                    spdlog::info("AHRS: Disabled");
                }
                break;
            case 'r':
                params.mRealTime = true;
                spdlog::info("Real-time mode enabled");
                break;
            case 'p':
                {
                    int priority = std::stoi(optarg);
                    if (priority >= 1 && priority <= 99)
                    {
                        params.mPriority = priority;
                        spdlog::info("Thread priority set to: {}", priority);
                    }
                    else
                    {
                        spdlog::error("Invalid priority value (must be 1-99): {}", priority);
                        return false;
                    }
                }
                break;
            case 'P':
                {
                    std::string policy = optarg;
                    if (policy == "FIFO")
                    {
                        params.mPolicy = SCHED_FIFO;
                        spdlog::info("Scheduling policy: FIFO");
                    }
                    else if (policy == "RR")
                    {
                        params.mPolicy = SCHED_RR;
                        spdlog::info("Scheduling policy: Round Robin");
                    }
                    else
                    {
                        spdlog::error("Invalid scheduling policy (must be FIFO or RR): {}", policy);
                        return false;
                    }
                }
                break;
            default:
                return false;
        }
    }
    if (params.mSocketPath.empty())
    {
        spdlog::error("Socket path is required");
        return false;
    }
    return true;
}