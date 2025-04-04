#pragma once

#include <string>
#include <random>
#include <sys/un.h>

#include <generic_thread.h>

#include "IMUTypes.h"

struct Parameters;

class IMUPublisher : public GenericThread<IMUPublisher>
{
public:
    IMUPublisher();
    virtual ~IMUPublisher();

    bool initialise(const Parameters& params);

    void* threadBody();

private:
    void disconnect();
    bool setupSocket();
    void setupRandomGenerator();
    void generateRandomIMUData(Payload_IMU_t& imuData);

    std::string mSocketPath;
    long mPeriodNs;
    int mSocket;
    std::mt19937 mGen;
    std::uniform_real_distribution<float> mAccDist;   // For accelerometer in mg
    std::uniform_real_distribution<float> mGyroDist;  // For gyroscope in mdeg/s
    std::uniform_real_distribution<float> mMagDist;   // For magnetometer in mGauss
};
