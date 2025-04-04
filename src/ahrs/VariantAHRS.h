#pragma once

#include <variant>
#include <optional>
#include <functional>
#include "core/PayloadIMU.h"
#include "core/AHRSType.h"
#include "ahrs/MadgwickAHRS.h"
#include "ahrs/SimpleAHRS.h"

/**
 * @brief A variant-based AHRS implementation that uses std::variant instead of inheritance
 * 
 * This class demonstrates an alternative approach to polymorphism using C++17's std::variant.
 * Instead of using inheritance and virtual functions, it uses std::variant to store
 * different AHRS algorithm implementations and std::visit to dispatch operations.
 */
class VariantAHRS
{
public:
    /**
     * @brief Variant type that can hold any of the AHRS algorithm implementations
     */
    using AHRSVariant = std::variant<MadgwickAHRS, SimpleAHRS>;
    
    /**
     * @brief Create a VariantAHRS instance
     * 
     * @param type The AHRS algorithm type
     * @param updateFrequencyHz The update frequency in Hz
     * @return Optional VariantAHRS (empty if type is NONE)
     */
    static std::optional<VariantAHRS> create(AHRSType type, float updateFrequencyHz)
    {
        switch (type)
        {
            case AHRSType::MADGWICK:
                return VariantAHRS(MadgwickAHRS(updateFrequencyHz));
                
            case AHRSType::SIMPLE:
                return VariantAHRS(SimpleAHRS(updateFrequencyHz));
                
            case AHRSType::NONE:
            default:
                return std::nullopt;
        }
    }
    
    /**
     * @brief Process IMU data to update orientation
     * 
     * @param payload The IMU payload data
     */
    void update(const Payload_IMU_t& payload)
    {
        std::visit([&payload](auto& ahrs) { ahrs.update(payload); }, mVariant);
    }
    
    /**
     * @brief Get the quaternion representing orientation
     * 
     * @return Pointer to the quaternion array [w, x, y, z]
     */
    const float* getQuaternion() const
    {
        return std::visit([](const auto& ahrs) { return ahrs.getQuaternion(); }, mVariant);
    }
    
    /**
     * @brief Get the Euler angles representing orientation
     * 
     * @return Pointer to the angles array [roll, pitch, yaw] in degrees
     */
    const float* getAngles() const
    {
        return std::visit([](const auto& ahrs) { return ahrs.getAngles(); }, mVariant);
    }
    
private:
    /**
     * @brief Constructor with variant
     * 
     * @param variant The AHRS variant
     */
    template <typename T>
    explicit VariantAHRS(T&& ahrs) : mVariant(std::forward<T>(ahrs)) {}
    
    AHRSVariant mVariant; ///< The variant holding the AHRS implementation
};
