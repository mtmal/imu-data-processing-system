#include <filesystem>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "communication/IMUPublisher.h"
#include "core/PayloadIMU.h"

namespace
{
inline constexpr long NSEC_PER_SEC = 1000000000L;

class ScopedLock
{
public:
    explicit ScopedLock(pthread_mutex_t& lock) : mLock(lock)
    {
        pthread_mutex_lock(&mLock);
    }

    virtual ~ScopedLock()
    {
        pthread_mutex_unlock(&mLock);
    }

private:
    pthread_mutex_t& mLock;
};
} // end of anonymous namespace

IMUPublisher::IMUPublisher(IMUDataProvider& dataProvider) 
: IMUSocketHandler(),
  mDataProvider(dataProvider),
  mPeriodNs(0)
{
}

IMUPublisher::~IMUPublisher()
{
    disconnect();
}

bool IMUPublisher::initialise(const Parameters& params)
{
    IMUSocketHandler::initialise(params);
    mPeriodNs = NSEC_PER_SEC / params.mFrequencyHz;
    
    // Initialize the data provider
    if (!mDataProvider.initialize())
    {
        spdlog::error("Failed to initialize IMU data provider");
        return false;
    }
    
    disconnect();
    return setupSocket(params.mSocketPath);
}

void IMUPublisher::threadBody()
{
    Payload_IMU_t imuData;
    struct timespec startTime;
    struct timespec endTime;
    struct timespec sleepTime;
    long processingTimeNs;
    long sleepTimeNs;
    pthread_mutexattr_t mutexAttr;

    pthread_mutexattr_init(&mutexAttr);
    
    // Enable priority inheritance for real-time operation
    if (mParameters.mRealTime)
    {
        pthread_mutexattr_setprotocol(&mutexAttr, PTHREAD_PRIO_INHERIT);
    }
    
    pthread_mutex_init(&mSubscribersMutex, &mutexAttr);

    while (isRunning())
    {
        // Get time at the start of the loop
        clock_gettime(CLOCK_MONOTONIC, &startTime);
        
        // Check for new subscriber registrations
        checkForRegistrations();
        
        // Get IMU data from the provider
        mDataProvider.getIMUData(imuData);
        
        // Send data to all subscribers
        sendData(imuData);
        
        // Get time after data was generated and published
        clock_gettime(CLOCK_MONOTONIC, &endTime);
        
        // Calculate processing time
        processingTimeNs = (endTime.tv_sec - startTime.tv_sec) * NSEC_PER_SEC + 
                           (endTime.tv_nsec - startTime.tv_nsec);
        
        // Calculate sleep time by subtracting processing time from period
        sleepTimeNs = mPeriodNs - processingTimeNs;
        if (sleepTimeNs > 0)
        {
            sleepTime.tv_sec = sleepTimeNs / NSEC_PER_SEC;
            sleepTime.tv_nsec = sleepTimeNs % NSEC_PER_SEC;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &sleepTime, nullptr);
        }
        else
        {
            spdlog::warn("Processing time exceeded period by {} us", -sleepTimeNs / 1000);
        }
    }
    
    pthread_mutex_destroy(&mSubscribersMutex);
    pthread_mutexattr_destroy(&mutexAttr);
}

void IMUPublisher::disconnect()
{
    IMUSocketHandler::disconnect();
    
    if (!mParameters.mSocketPath.empty() && std::filesystem::exists(mParameters.mSocketPath))
    {
        spdlog::info("Unlinking existing socket at {}", mParameters.mSocketPath);
        std::filesystem::remove(mParameters.mSocketPath);
    }
}

void IMUPublisher::checkForRegistrations()
{
    struct sockaddr_un client_addr;
    socklen_t addrlen = sizeof(client_addr);
    char buffer[10];
    
    // Non-blocking receive to check for registrations
    ssize_t bytes_read = recvfrom(mSocket, buffer, sizeof(buffer), MSG_DONTWAIT,
                                 reinterpret_cast<struct sockaddr*>(&client_addr), &addrlen);
    
    if (bytes_read > 0)
    {
        // Got a registration message
        ScopedLock lock(mSubscribersMutex);
        
        // Check if this subscriber is already registered
        bool found = false;
        for (const auto& addr : mSubscribers)
        {
            if (strcmp(addr.sun_path, client_addr.sun_path) == 0)
            {
                found = true;
                break;
            }
        }
        // The subscriber was not found so add it to the list
        if (!found)
        {
            mSubscribers.push_back(client_addr);
            spdlog::info("New subscriber registered: {}", client_addr.sun_path);
        }
    }
}

void IMUPublisher::sendData(const Payload_IMU_t& imuData)
{
    ssize_t bytes_sent;
    ScopedLock lock(mSubscribersMutex);
    for (auto it = mSubscribers.begin(); it != mSubscribers.end(); )
    {
        bytes_sent = sendto(mSocket, &imuData, sizeof(Payload_IMU_t), 0, 
            reinterpret_cast<const struct sockaddr*>(&(*it)), sizeof(*it));
        
        if (bytes_sent < 0)
        {
            if (errno == ENOENT || errno == ECONNREFUSED)
            {
                // Subscriber socket no longer exists or connection refused
                spdlog::warn("Subscriber disconnected: {}", it->sun_path);
                it = mSubscribers.erase(it);
                continue;
            }
            else
            {
                spdlog::error("Error sending data: {}", strerror(errno));
            }
        }
        else if (bytes_sent != sizeof(Payload_IMU_t))
        {
            spdlog::warn("Warning: Not all bytes were sent");
        }
        else
        {
            spdlog::info("Sent {} bytes to {}", bytes_sent, it->sun_path);
        }
        ++it;
    }
}