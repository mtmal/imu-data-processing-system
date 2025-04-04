#pragma once

/**
 * @brief Enumeration of available AHRS algorithms
 */
enum class AHRSType
{
    NONE,       ///< No AHRS processing
    MADGWICK,   ///< Madgwick AHRS algorithm
    SIMPLE      ///< Simple AHRS algorithm
}; 