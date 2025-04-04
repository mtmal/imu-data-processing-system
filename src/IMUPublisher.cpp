#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <scoped_lock.h>

#include "IMUPublisher.h"
#include "IMUTypes.h"
#include "Parameters.h"

constexpr long NSEC_PER_SEC = 1000000000L;

IMUPublisher::IMUPublisher() 
    : mSocketPath("")
    , mPeriodNs(0)
    , mSocket(-1)
{
    pthread_mutex_init(&mSubscribersMutex, nullptr);
}

IMUPublisher::~IMUPublisher()
{
    disconnect();
    pthread_mutex_destroy(&mSubscribersMutex);
}

bool IMUPublisher::initialise(const Parameters& params)
{
    mSocketPath = params.mSocketPath;
    mPeriodNs = NSEC_PER_SEC / params.mFrequencyHz;
    setupRandomGenerator();
    disconnect();
    return setupSocket();
}

void* IMUPublisher::threadBody()
{
    Payload_IMU_t imuData;
    struct timespec startTime;
    struct timespec endTime;
    struct timespec sleepTime;
    long processingTimeNs;
    long sleepTimeNs;

    while (isRunning())
    {
        // Get time at the start of the loop
        clock_gettime(CLOCK_MONOTONIC, &startTime);
        // Check for new subscriber registrations
        checkForRegistrations();
        // Generate new random data
        generateRandomIMUData(imuData);
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
    return nullptr;
}

void IMUPublisher::disconnect()
{
    if (mSocket >= 0)
    {
        spdlog::info("Closing exising socket.");
        close(mSocket);
    }
    if (!mSocketPath.empty())
    {
        spdlog::info("Unlinking exising socket.");
        unlink(mSocketPath.c_str());
    }
}

bool IMUPublisher::setupSocket()
{
    spdlog::info("Creating socket at path: {}", mSocketPath);
    mSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (mSocket < 0)
    {
        spdlog::error("Failed to create the socket. Error code: {}", errno);
        return false;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, mSocketPath.c_str(), sizeof(addr.sun_path)-1);

    if (bind(mSocket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        spdlog::error("Failed to bind the socket. Error code: {}", errno);
        close(mSocket);
        return false;
    }

    spdlog::info("Socket created successfully.");
    return true;
}

void IMUPublisher::setupRandomGenerator()
{
    std::random_device rd;
    mGen = std::mt19937(rd());
    mAccDist = std::uniform_real_distribution<float>(-16000.0f, 16000.0f);
    mGyroDist = std::uniform_real_distribution<float>(-2000000.0f, 2000000.0f);
    mMagDist = std::uniform_real_distribution<float>(200.0f, 600.0f);
    spdlog::info("Random generator setup complete.");
}

void IMUPublisher::generateRandomIMUData(Payload_IMU_t& imuData)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    imuData.xAcc = mAccDist(mGen);
    imuData.yAcc = mAccDist(mGen);
    imuData.zAcc = mAccDist(mGen);
    imuData.timestampAcc = static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);

    imuData.xGyro = mGyroDist(mGen);
    imuData.yGyro = mGyroDist(mGen);
    imuData.zGyro = mGyroDist(mGen);
    imuData.timestampGyro = imuData.timestampAcc;

    imuData.xMag = mMagDist(mGen);
    imuData.yMag = mMagDist(mGen);
    imuData.zMag = mMagDist(mGen);
    imuData.timestampMag = imuData.timestampAcc;
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