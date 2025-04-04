#pragma once

typedef struct Payload_IMU_s Payload_IMU_t;

/**
 * @brief Interface for AHRS (Attitude and Heading Reference System) algorithms
 * 
 * This interface defines the common functionality for all AHRS implementations.
 * Each implementation will process IMU data and provide orientation information.
 */
class AHRS
{
public:
    /**
     * @brief Constructor with update frequency
     * 
     * @param updateFrequencyHz The update frequency in Hz
     */
    explicit AHRS(const float updateFrequencyHz);
    
    /**
     * @brief Virtual destructor
     */
    virtual ~AHRS() = default;
    
    /**
     * @brief Process IMU data to update orientation
     * 
     * @param payload The IMU payload data
     */
    virtual void update(const Payload_IMU_t& payload) = 0;
    
    /**
     * @brief Get the quaternion representing orientation
     * 
     * @return Pointer to the quaternion array [w, x, y, z]
     */
    inline const float* getQuaternion() const { return mQuat; }
    
    /**
     * @brief Get the Euler angles representing orientation
     * 
     * @return Pointer to the angles array [roll, pitch, yaw] in degrees
     */
    inline const float* getAngles() const { return mAngles; }

protected:
    /**
     * @brief Convert quaternion to Euler angles
     */
    void quatToAngles();
    
    /**
     * @brief Fast inverse square-root
     * 
     * @param x Input value
     * @return Inverse square root of x
     */
    static float invSqrt(const float x);
    
    float mQuat[4];        ///< Quaternion [w, x, y, z]
    float mAngles[3];      ///< Euler angles [roll, pitch, yaw] in degrees
    float mUpdatePeriod;   ///< Update period in seconds
}; 