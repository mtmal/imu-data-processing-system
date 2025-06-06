#include <csignal>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <sys/socket.h>
#include <sys/un.h>

#include "communication/IMUSubscriber.h"
#include "core/PayloadIMU.h"

inline constexpr char REG_MSG[9] = "REGISTER";

namespace
{
/**
 * @brief Print IMU data to console
 * 
 * @param data The IMU data to print
 * @param ahrs The AHRS processor (if available)
 */
void printIMUData(const Payload_IMU_t& data, const std::optional<VariantAHRS>& ahrs = std::nullopt)
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "=== IMU Data ===\n";
    std::cout << "TimestampAcc: " << data.timestampAcc << " ms\n";
    std::cout << "Accel: [" << data.xAcc << ", " << data.yAcc << ", " << data.zAcc << "]\n";
    std::cout << "TimestampGyro: " << data.timestampGyro << " ms\n";
    std::cout << "Gyro:  [" << data.xGyro << ", " << data.yGyro << ", " << data.zGyro << "]\n";
    std::cout << "TimestampMag: " << data.timestampMag << " ms\n";
    std::cout << "Mag:   [" << data.xMag << ", " << data.yMag << ", " << data.zMag << "]\n";
    
    if (ahrs)
    {
        const float* quat = ahrs->getQuaternion();
        const float* angles = ahrs->getAngles();
        
        std::cout << "=== AHRS Data ===\n";
        std::cout << "Quaternion: [" << quat[0] << ", " 
                                     << quat[1] << ", " 
                                     << quat[2] << ", " 
                                     << quat[3] << "]\n";
        std::cout << "Angles: [Roll: " << angles[0] << "°, "
                  << "Pitch: " << angles[1] << "°, "
                  << "Yaw: " << angles[2] << "°]\n";
    }
    
    std::cout << "----------------------------------\n";
}
} // end of anonymous namespace

IMUSubscriber::IMUSubscriber()
: IMUSocketHandler()
, mClientSocketPath("")
, mAhrs(std::nullopt)
{
}

IMUSubscriber::~IMUSubscriber()
{
    disconnect();
}

bool IMUSubscriber::initialise(const Parameters& params)
{
    IMUSocketHandler::initialise(params);
    mClientSocketPath = params.mSocketPath + "_client" + std::to_string(getpid());
    
    // Create AHRS instance based on parameters
    mAhrs = VariantAHRS::create(params.mAhrsType, params.mFrequencyHz);
    
    disconnect();
    return setupSocket(mClientSocketPath) && registerToServer() && setSocketTimeout();
}

void IMUSubscriber::threadBody()
{
    Payload_IMU_t imuData;
    ssize_t bytes_read;
    struct sockaddr_un src_addr;
    socklen_t addrlen = sizeof(src_addr);
    
    while (isRunning())
    {
        bytes_read = recvfrom(mSocket, &imuData, sizeof(Payload_IMU_t), 0,
                            reinterpret_cast<struct sockaddr*>(&src_addr), &addrlen);
        
        if (bytes_read < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // Timeout occurred. Log error and raise SIGALRM
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
            if (mAhrs)
            {
                // Process received data with AHRS
                mAhrs->update(imuData);
            }
            // Print the data
            printIMUData(imuData, mAhrs);
        }
    }
}

void IMUSubscriber::disconnect()
{
    IMUSocketHandler::disconnect();
    
    if (!mClientSocketPath.empty() && std::filesystem::exists(mClientSocketPath))
    {
        spdlog::info("Unlinking existing socket at {}", mClientSocketPath);
        std::filesystem::remove(mClientSocketPath);
    }
}

bool IMUSubscriber::registerToServer()
{   
    // Set up server address
    struct sockaddr_un server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, mParameters.mSocketPath.c_str(), sizeof(server_addr.sun_path) - 1);
    
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

bool IMUSubscriber::setSocketTimeout()
{
    if (mParameters.mTimeoutMs > 0)
    {
        struct timeval tv;
        tv.tv_sec = mParameters.mTimeoutMs / 1000;  // Convert ms to seconds
        tv.tv_usec = (mParameters.mTimeoutMs % 1000) * 1000;  // Remaining ms to microseconds
        
        if (setsockopt(mSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
        {
            spdlog::error("Failed to set socket timeout: {}", strerror(errno));
            return false;
        }
    }
    return true;
}
