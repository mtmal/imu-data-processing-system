#pragma once

#include <random>
#include "IMUDataProvider.h"

/**
 * @brief Implementation of IMUDataProvider that generates random IMU data
 * 
 * This class provides a simulated IMU data source by generating random
 * values within realistic ranges for accelerometer, gyroscope, and
 * magnetometer readings.
 */
class RandomIMUDataProvider : public IMUDataProvider
{
public:
    /**
     * @brief Constructor initializes the random distributions with appropriate ranges
     */
    RandomIMUDataProvider();
    
    /**
     * @brief Virtual destructor
     */
    virtual ~RandomIMUDataProvider() = default;
    
    /**
     * @brief Initialize the random number generator
     * 
     * Sets up the random number generator with a random seed.
     * 
     * @return true always, as initialization should not fail
     */
    virtual bool initialize() override;
    
    /**
     * @brief Generate random IMU data
     * 
     * Fills the provided data structure with random values for
     * accelerometer, gyroscope, and magnetometer readings within
     * realistic ranges.
     * 
     * @param data Reference to the IMU data structure to fill
     */
    virtual void getIMUData(Payload_IMU_t& data) override;
    
private:
    std::mt19937 mGen;                               ///< Mersenne Twister random generator
    std::uniform_real_distribution<float> mAccDist;  ///< Distribution for accelerometer in mg
    std::uniform_real_distribution<float> mGyroDist; ///< Distribution for gyroscope in mdeg/s
    std::uniform_real_distribution<float> mMagDist;  ///< Distribution for magnetometer in mGauss
}; 