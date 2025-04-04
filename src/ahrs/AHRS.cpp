#include "ahrs/AHRS.h"
#include <cmath>

AHRS::AHRS(const float updateFrequencyHz) 
    : mUpdatePeriod(1.0f / updateFrequencyHz)
{
    // Initialize quaternion to identity
    mQuat[0] = 1.0f;
    mQuat[1] = 0.0f;
    mQuat[2] = 0.0f;
    mQuat[3] = 0.0f;
    
    // Initialize angles to zero
    mAngles[0] = 0.0f;
    mAngles[1] = 0.0f;
    mAngles[2] = 0.0f;
}

float AHRS::invSqrt(const float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void AHRS::quatToAngles()
{
    static const float RAD_TO_DEG = 180.0f / M_PI;
    
    mAngles[1] = 2 * (mQuat[0] * mQuat[2] - mQuat[1] * mQuat[3]);
    if (fabs(mAngles[1]) >= 1)
    {
        mAngles[1] = std::copysign(M_PI / 2, mAngles[1]) * RAD_TO_DEG;
    }
    else
    {
        mAngles[1] = asin(mAngles[1]) * RAD_TO_DEG;
    }
    mAngles[0] = atan2(2 * (mQuat[0] * mQuat[1] + mQuat[2] * mQuat[3]), 1 - 2 * (mQuat[1] * mQuat[1] + mQuat[2] * mQuat[2])) * RAD_TO_DEG;
    mAngles[2] = atan2(2 * (mQuat[0] * mQuat[3] + mQuat[1] * mQuat[2]), 1 - 2 * (mQuat[2] * mQuat[2] + mQuat[3] * mQuat[3])) * RAD_TO_DEG;
} 