#include <getopt.h>
#include <spdlog/spdlog.h>

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
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "s:l:f:t:a:", long_options, nullptr)) != -1)
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