#pragma once

#include "ahrs/AHRS.h"

/**
 * @brief Madgwick AHRS algorithm implementation
 * 
 * Implementation of Madgwick's IMU and AHRS algorithms.
 * See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 */
class MadgwickAHRS : public AHRS
{
public:
    /**
     * @brief Constructor
     * 
     * @param updateFrequencyHz The update frequency in Hz
     */
    MadgwickAHRS(const float updateFrequencyHz);
    
    /**
     * @brief Process IMU data using Madgwick algorithm
     * 
     * @param payload The IMU payload data
     */
    void update(const Payload_IMU_t& payload) override;
    
private:
    // Algorithm parameters
    float mBeta; ///< Algorithm gain
}; 