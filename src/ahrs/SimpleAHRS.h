#pragma once

#include "ahrs/AHRS.h"

/**
 * @brief Simple AHRS algorithm implementation
 */
class SimpleAHRS : public AHRS
{
public:
    /**
     * @brief Constructor
     * 
     * @param updateFrequencyHz The update frequency in Hz
     */
    SimpleAHRS(const float updateFrequencyHz);
    
    /**
     * @brief Process IMU data using Simple algorithm
     * 
     * @param payload The IMU payload data
     */
    void update(const Payload_IMU_t& payload) override;
    
private:
    // Algorithm parameters
    float mKp; ///< Proportional gain
    float mKi; ///< Integral gain
    float mExInt, mEyInt, mEzInt; ///< Integral error terms
}; 