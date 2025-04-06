#pragma once

#include <string> 
#include "core/AHRSType.h"

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
    AHRSType mAhrsType;      ///< AHRS algorithm to use
    bool mRealTime;          ///< Flag for real-time thread configuration
    int mPriority;           ///< Thread priority (1-99 for real-time)
    int mPolicy;             ///< Scheduling policy (SCHED_FIFO or SCHED_RR) for real-time

    /**
     * @brief Initialise all parameters with default values.
     */
    Parameters() 
    : mSocketPath(""),
      mFrequencyHz(500),
      mTimeoutMs(100),
      mAhrsType(AHRSType::NONE),
      mRealTime(false),
      mPriority(50),
      mPolicy(SCHED_FIFO)
    {}
};