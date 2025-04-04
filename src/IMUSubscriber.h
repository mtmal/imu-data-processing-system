#pragma once

#include "IMUSocketHandler.h"

struct Parameters;

class IMUSubscriber : public IMUSocketHandler<IMUSubscriber>
{
public:
    IMUSubscriber();
    virtual ~IMUSubscriber();

    bool initialise(const Parameters& params) override;

    void* threadBody();

private:
    /**
     * @brief Registers this subscriber to publisher
     * @return true if the registration was successful
     */
    bool registerToServer();
    void disconnect() override;

    std::string mClientSocketPath;
    ulong mTimeoutMs;
};