#include <csignal>
#include <iostream>
#include <iomanip>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <time.h>

#include "IMUSubscriber.h"
#include "IMUTypes.h"
#include "Parameters.h"


constexpr char REG_MSG[9] = "REGISTER";


void printIMUData(const Payload_IMU_t& data)
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "=== IMU Data ===\n";
    std::cout << "TimestampAcc: " << data.timestampAcc << " ms\n";
    std::cout << "Accel: [" << data.xAcc << ", " << data.yAcc << ", " << data.zAcc << "]\n";
    std::cout << "TimestampGyro: " << data.timestampGyro << " ms\n";
    std::cout << "Gyro:  [" << data.xGyro << ", " << data.yGyro << ", " << data.zGyro << "]\n";
    std::cout << "TimestampMag: " << data.timestampMag << " ms\n";
    std::cout << "Mag:   [" << data.xMag << ", " << data.yMag << ", " << data.zMag << "]\n";
    std::cout << "----------------------------------\n";
}

IMUSubscriber::IMUSubscriber()
: mServerSocketPath(""),
  mClientSocketPath(""),
  mSocket(-1),
  mTimeoutMs(0)
{
}

IMUSubscriber::~IMUSubscriber()
{
    disconnect();
}

bool IMUSubscriber::initialise(const Parameters& params)
{
    mServerSocketPath = params.mSocketPath;
    mClientSocketPath = mServerSocketPath + "_client" + std::to_string(getpid());
    mTimeoutMs = params.mTimeoutUs;
    disconnect();
    return setupSocket();
}

void* IMUSubscriber::threadBody()
{
    Payload_IMU_t imuData;
    ssize_t bytes_read;
    struct sockaddr_un src_addr;
    socklen_t addrlen = sizeof(src_addr);
    
    // Set up timeout for the socket
    if (mTimeoutMs > 0)
    {
        struct timeval tv;
        tv.tv_sec = mTimeoutMs / 1000;  // Convert ms to seconds
        tv.tv_usec = (mTimeoutMs % 1000) * 1000;  // Remaining ms to microseconds
        
        if (setsockopt(mSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
        {
            spdlog::error("Failed to set socket timeout: {}", strerror(errno));
        }
    }
    
    while (isRunning())
    {
        bytes_read = recvfrom(mSocket, &imuData, sizeof(Payload_IMU_t), 0,
                            reinterpret_cast<struct sockaddr*>(&src_addr), &addrlen);
        
        if (bytes_read < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // Timeout occurred. Log error and rais SIGALRM
                spdlog::error("Timeout, the publisher might be down. Exiting...");
                raise(SIGALRM);
            }
            else
            {
                spdlog::error("Error reading from socket: {}", strerror(errno));
            }
        }
        else if (bytes_read == 0)
        {
            spdlog::warn("No data was read!");
        }
        else if (bytes_read != sizeof(Payload_IMU_t))
        {
            spdlog::warn("Incomplete data received: {} bytes", bytes_read);
        }
        else
        {
            // Process received data
            printIMUData(imuData);
        }
    }
    return nullptr;
}

void IMUSubscriber::disconnect()
{
    if (mSocket >= 0)
    {
        spdlog::info("Closing exising socket.");
        close(mSocket);
    }
    if (!mClientSocketPath.empty())
    {
        spdlog::info("Unlinking exising socket.");
        unlink(mClientSocketPath.c_str());
    }
}

bool IMUSubscriber::setupSocket()
{
    spdlog::info("Creating socket at path: {}", mServerSocketPath);
    mSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (mSocket < 0)
    {
        spdlog::error("Failed to create the socket. Error code: {}", strerror(errno));
        return false;
    }
        
    // Set up client address
    struct sockaddr_un client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sun_family = AF_UNIX;
    strncpy(client_addr.sun_path, mClientSocketPath.c_str(), sizeof(client_addr.sun_path) - 1);
    
    // Remove if it exists
    unlink(mClientSocketPath.c_str());
    
    // Bind to client address
    if (bind(mSocket, reinterpret_cast<struct sockaddr*>(&client_addr), sizeof(client_addr)) < 0)
    {
        spdlog::error("Error binding client socket: {}", strerror(errno));
        return false;
    }
    
    // Set up server address
    struct sockaddr_un server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, mServerSocketPath.c_str(), sizeof(server_addr.sun_path) - 1);
    
    // Send registration message to publisher
    if (sendto(mSocket, REG_MSG, strlen(REG_MSG), 0, 
               reinterpret_cast<struct sockaddr*>(&server_addr), sizeof(server_addr)) < 0)
    {
        spdlog::error("Failed to send registration message: {}", strerror(errno));
        return false;
    }
    
    spdlog::info("Socket created successfully and registered with publisher");
    return true;
}