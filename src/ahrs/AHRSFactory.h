#pragma once

#include "core/AHRSType.h"
#include <memory>
#include <optional>

class AHRS;

/**
 * @brief Factory for creating AHRS instances
 */
class AHRSFactory
{
public:
    /**
     * @brief Create an AHRS instance based on type
     * 
     * @param type The AHRS algorithm type
     * @param updateFrequencyHz The update frequency in Hz
     * @return Optional unique pointer to the created AHRS instance (empty if type is NONE)
     */
    static std::optional<std::unique_ptr<AHRS>> create(const AHRSType type, const float updateFrequencyHz);
}; 