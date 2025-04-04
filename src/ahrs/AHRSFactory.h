#pragma once

#include "core/AHRSType.h"
#include <memory>

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
     * @return Unique pointer to the created AHRS instance
     */
    static std::unique_ptr<AHRS> create(const AHRSType type, const float updateFrequencyHz);
}; 