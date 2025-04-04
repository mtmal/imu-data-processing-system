#include <cmath>
#include "ahrs/MadgwickAHRS.h"
#include "core/PayloadIMU.h"

MadgwickAHRS::MadgwickAHRS(const float updateFrequencyHz)
    : AHRS(updateFrequencyHz)
    , mBeta(0.1f) // Default algorithm gain
{
}

void MadgwickAHRS::update(const Payload_IMU_t& payload)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Convert gyro data from mdeg/s to rad/s
    const float DEG_TO_RAD = 0.017453292f;
    const float MDEG_TO_RAD = DEG_TO_RAD / 1000.0f;
    float gx = payload.xGyro * MDEG_TO_RAD;
    float gy = payload.yGyro * MDEG_TO_RAD;
    float gz = payload.zGyro * MDEG_TO_RAD;
    
    // Local copies of acceleration and magnetometer data
    float ax = payload.xAcc;
    float ay = payload.yAcc;
    float az = payload.zAcc;
    float mx = payload.xMag;
    float my = payload.yMag;
    float mz = payload.zMag;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-mQuat[1] * gx - mQuat[2] * gy - mQuat[3] * gz);
    qDot2 = 0.5f * (mQuat[0] * gx + mQuat[2] * gz - mQuat[3] * gy);
    qDot3 = 0.5f * (mQuat[0] * gy - mQuat[1] * gz + mQuat[3] * gx);
    qDot4 = 0.5f * (mQuat[0] * gz + mQuat[1] * gy - mQuat[2] * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * mQuat[0] * mx;
        _2q0my = 2.0f * mQuat[0] * my;
        _2q0mz = 2.0f * mQuat[0] * mz;
        _2q1mx = 2.0f * mQuat[1] * mx;
        _2q0 = 2.0f * mQuat[0];
        _2q1 = 2.0f * mQuat[1];
        _2q2 = 2.0f * mQuat[2];
        _2q3 = 2.0f * mQuat[3];
        _2q0q2 = 2.0f * mQuat[0] * mQuat[2];
        _2q2q3 = 2.0f * mQuat[2] * mQuat[3];
        q0q0 = mQuat[0] * mQuat[0];
        q0q1 = mQuat[0] * mQuat[1];
        q0q2 = mQuat[0] * mQuat[2];
        q0q3 = mQuat[0] * mQuat[3];
        q1q1 = mQuat[1] * mQuat[1];
        q1q2 = mQuat[1] * mQuat[2];
        q1q3 = mQuat[1] * mQuat[3];
        q2q2 = mQuat[2] * mQuat[2];
        q2q3 = mQuat[2] * mQuat[3];
        q3q3 = mQuat[3] * mQuat[3];

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * mQuat[3] + _2q0mz * mQuat[2] + mx * q1q1 + _2q1 * my * mQuat[2] + _2q1 * mz * mQuat[3] - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * mQuat[3] + my * q0q0 - _2q0mz * mQuat[1] + _2q1mx * mQuat[2] - my * q1q1 + my * q2q2 + _2q2 * mz * mQuat[3] - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * mQuat[2] + _2q0my * mQuat[1] + mz * q0q0 + _2q1mx * mQuat[3] - mz * q1q1 + _2q2 * my * mQuat[3] - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * mQuat[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * mQuat[3] + _2bz * mQuat[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * mQuat[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * mQuat[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * mQuat[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * mQuat[2] + _2bz * mQuat[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * mQuat[3] - _4bz * mQuat[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * mQuat[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * mQuat[2] - _2bz * mQuat[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * mQuat[1] + _2bz * mQuat[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * mQuat[0] - _4bz * mQuat[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * mQuat[3] + _2bz * mQuat[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * mQuat[0] + _2bz * mQuat[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * mQuat[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= mBeta * s0;
        qDot2 -= mBeta * s1;
        qDot3 -= mBeta * s2;
        qDot4 -= mBeta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    mQuat[0] += qDot1 * mUpdatePeriod;
    mQuat[1] += qDot2 * mUpdatePeriod;
    mQuat[2] += qDot3 * mUpdatePeriod;
    mQuat[3] += qDot4 * mUpdatePeriod;

    // Normalise quaternion
    recipNorm = invSqrt(mQuat[0] * mQuat[0] + mQuat[1] * mQuat[1] + mQuat[2] * mQuat[2] + mQuat[3] * mQuat[3]);
    mQuat[0] *= recipNorm;
    mQuat[1] *= recipNorm;
    mQuat[2] *= recipNorm;
    mQuat[3] *= recipNorm;

    // Update Euler angles
    quatToAngles();
} 