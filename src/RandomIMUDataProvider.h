#pragma once

#include <random>
#include "IMUDataProvider.h"

/**
 * @brief Implementation of IMUDataProvider that generates random IMU data
 */
class RandomIMUDataProvider : public IMUDataProvider
{
public:
    RandomIMUDataProvider();
    virtual ~RandomIMUDataProvider() = default;
    
    virtual bool initialize() override;
    virtual void getIMUData(Payload_IMU_t& data) override;
    
private:
    std::mt19937 mGen;
    std::uniform_real_distribution<float> mAccDist;   // For accelerometer in mg
    std::uniform_real_distribution<float> mGyroDist;  // For gyroscope in mdeg/s
    std::uniform_real_distribution<float> mMagDist;   // For magnetometer in mGauss
}; 