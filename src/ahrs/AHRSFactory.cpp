#include "ahrs/AHRSFactory.h"
#include "ahrs/MadgwickAHRS.h"
#include "ahrs/SimpleAHRS.h"

std::unique_ptr<AHRS> AHRSFactory::create(const AHRSType type, const float updateFrequencyHz)
{
    switch (type)
    {
        case AHRSType::MADGWICK:
            return std::make_unique<MadgwickAHRS>(updateFrequencyHz);
            
        case AHRSType::SIMPLE:
            return std::make_unique<SimpleAHRS>(updateFrequencyHz);
            
        case AHRSType::NONE:
        default:
            return nullptr;
    }
} 