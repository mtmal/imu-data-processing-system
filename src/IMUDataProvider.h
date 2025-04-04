#pragma once

typedef struct Payload_IMU_s Payload_IMU_t;

/**
 * @brief Interface for providing IMU data
 * 
 * This interface abstracts the source of IMU data, allowing
 * for different implementations (random generation, hardware, etc.).
 * It defines the contract that any IMU data provider must fulfill.
 */
class IMUDataProvider
{
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup in derived classes
     */
    virtual ~IMUDataProvider() = default;
    
    /**
     * @brief Initialize the data provider
     * 
     * This method should perform any setup required for the data provider,
     * such as initializing hardware, setting up random generators, etc.
     * 
     * @return true if initialization was successful, false otherwise
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief Get the latest IMU data
     * 
     * This method should fill the provided data structure with the latest
     * IMU sensor readings (accelerometer, gyroscope, magnetometer).
     * 
     * @param data Reference to the IMU data structure to fill
     */
    virtual void getIMUData(Payload_IMU_t& data) = 0;
}; 