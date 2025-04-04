#pragma once

#include <string>

/**
 * @brief Parameters structure for IMU publisher and subscriber
 * 
 * This structure holds configuration parameters that are
 * passed to both the publisher and subscriber components.
 */
struct Parameters
{
    std::string mSocketPath; ///< Path to the Unix domain socket
    int mFrequencyHz;        ///< Publication frequency in Hz
    ulong mTimeoutMs;        ///< Timeout for socket operations in milliseconds

    /**
     * @brief Initialise all parameters with default values.
     */
    Parameters() : mSocketPath(""), mFrequencyHz(500), mTimeoutMs(100) {}
};