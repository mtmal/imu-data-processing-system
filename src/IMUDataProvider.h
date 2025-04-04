#pragma once

typedef struct Payload_IMU_s Payload_IMU_t;

/**
 * @brief Interface for providing IMU data
 * 
 * This interface abstracts the source of IMU data, allowing
 * for different implementations (random generation, hardware, etc.)
 */
class IMUDataProvider
{
public:
    virtual ~IMUDataProvider() = default;
    
    /**
     * @brief Initialize the data provider
     * @return true if initialization was successful
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief Get the latest IMU data
     * @param data Reference to the IMU data structure to fill
     */
    virtual void getIMUData(Payload_IMU_t& data) = 0;
}; 