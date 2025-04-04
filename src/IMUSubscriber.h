#pragma once

#include <string>
#include <generic_thread.h>

struct Parameters;

class IMUSubscriber : public GenericThread<IMUSubscriber>
{
public:
    IMUSubscriber();
    virtual ~IMUSubscriber();

    bool initialise(const Parameters& params);

    void* threadBody();

private:
    void disconnect();
    bool setupSocket();

    std::string mServerSocketPath;
    std::string mClientSocketPath;
    int mSocket;
    ulong mTimeoutMs;
};