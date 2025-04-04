#pragma once

#include <string>
#include <sys/un.h>
#include <vector>
#include "IMUSocketHandler.h"
#include "IMUDataProvider.h"

struct Parameters;

class IMUPublisher : public IMUSocketHandler<IMUPublisher>
{
public:
    /**
     * @brief Constructor with IMU data provider
     * @param dataProvider The IMU data provider to use
     */
    IMUPublisher(IMUDataProvider& dataProvider);
    virtual ~IMUPublisher();

    bool initialise(const Parameters& params) override;

    void* threadBody();

private:
    void checkForRegistrations();
    void sendData(const Payload_IMU_t& imuData);
    void disconnect() override;

    IMUDataProvider& mDataProvider;
    long mPeriodNs;
    std::vector<struct sockaddr_un> mSubscribers;
    pthread_mutex_t mSubscribersMutex;
};
