#pragma once

#include <string>
#include <random>
#include <sys/un.h>
#include <vector>
#include "IMUSocketHandler.h"

struct Parameters;
typedef struct Payload_IMU_s Payload_IMU_t;

class IMUPublisher : public IMUSocketHandler<IMUPublisher>
{
public:
    IMUPublisher();
    virtual ~IMUPublisher();

    bool initialise(const Parameters& params) override;

    void* threadBody();

private:
    void setupRandomGenerator();
    void generateRandomIMUData(Payload_IMU_t& imuData);
    void checkForRegistrations();
    void sendData(const Payload_IMU_t& imuData);
    void disconnect() override;

    long mPeriodNs;
    std::mt19937 mGen;
    std::uniform_real_distribution<float> mAccDist;   // For accelerometer in mg
    std::uniform_real_distribution<float> mGyroDist;  // For gyroscope in mdeg/s
    std::uniform_real_distribution<float> mMagDist;   // For magnetometer in mGauss
    std::vector<struct sockaddr_un> mSubscribers;
    pthread_mutex_t mSubscribersMutex;
};
