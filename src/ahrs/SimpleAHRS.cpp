#include <cmath>
#include "ahrs/SimpleAHRS.h"
#include "core/PayloadIMU.h"

SimpleAHRS::SimpleAHRS(const float updateFrequencyHz)
    : AHRS(updateFrequencyHz)
    , mKp(4.50f)
    , mKi(1.0f)
    , mExInt(0.0f)
    , mEyInt(0.0f)
    , mEzInt(0.0f)
{
}

void SimpleAHRS::update(const Payload_IMU_t& payload)
{
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez, halfT = mUpdatePeriod / 2.0f;

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

    float q0q0 = mQuat[0] * mQuat[0];
    float q0q1 = mQuat[0] * mQuat[1];
    float q0q2 = mQuat[0] * mQuat[2];
    float q0q3 = mQuat[0] * mQuat[3];
    float q1q1 = mQuat[1] * mQuat[1];
    float q1q2 = mQuat[1] * mQuat[2];
    float q1q3 = mQuat[1] * mQuat[3];
    float q2q2 = mQuat[2] * mQuat[2];
    float q2q3 = mQuat[2] * mQuat[3];
    float q3q3 = mQuat[3] * mQuat[3];

    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    norm = invSqrt(mx * mx + my * my + mz * mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;

    // compute reference direction of flux
    hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    // estimated direction of gravity and flux (v and w)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        mExInt = mExInt + ex * mKi * halfT;
        mEyInt = mEyInt + ey * mKi * halfT;
        mEzInt = mEzInt + ez * mKi * halfT;

        gx = gx + mKp * ex + mExInt;
        gy = gy + mKp * ey + mEyInt;
        gz = gz + mKp * ez + mEzInt;
    }

    mQuat[0] = mQuat[0] + (-mQuat[1] * gx - mQuat[2] * gy - mQuat[3] * gz) * halfT;
    mQuat[1] = mQuat[1] + (mQuat[0] * gx + mQuat[2] * gz - mQuat[3] * gy) * halfT;
    mQuat[2] = mQuat[2] + (mQuat[0] * gy - mQuat[1] * gz + mQuat[3] * gx) * halfT;
    mQuat[3] = mQuat[3] + (mQuat[0] * gz + mQuat[1] * gy - mQuat[2] * gx) * halfT;

    norm = invSqrt(mQuat[0] * mQuat[0] + mQuat[1] * mQuat[1] + mQuat[2] * mQuat[2] + mQuat[3] * mQuat[3]);
    mQuat[0] = mQuat[0] * norm;
    mQuat[1] = mQuat[1] * norm;
    mQuat[2] = mQuat[2] * norm;
    mQuat[3] = mQuat[3] * norm;

    quatToAngles();
} 