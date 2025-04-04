#pragma once

#include <string>
#include <cstdint>

/**
 * @brief Structure to hold the parameters for the IMU publisher and subscriber.
 */
struct Parameters
{
    /** The path to the socket that needs to be created */
    std::string mSocketPath = "";
    /** The publication frequency in Hz */
    uint16_t mFrequencyHz = 500;
    /** Timeout in us for subscriber */
    uint16_t mTimeoutUs = 100;
};