#pragma once

#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <generic_thread.h>
#include <spdlog/spdlog.h>

// Forward declarations
struct Parameters;

/**
 * @brief Base class for IMU socket communication handling
 * 
 * This class provides common functionality for socket-based IPC
 * used by both publisher and subscriber components.
 */
template <typename T>
class IMUSocketHandler : public GenericThread<T>
{
public:
    IMUSocketHandler() : mSocketPath(""), mSocket(-1) {}

    virtual ~IMUSocketHandler()
    {
        disconnect();
    }

    /**
     * @brief Initialize the socket handler with parameters
     * @param params The parameters structure
     * @return true if initialization was successful, false otherwise
     */
    virtual bool initialise(const Parameters& params) = 0;

protected:
    /**
     * @brief Disconnect and clean up socket resources
     */
    virtual void disconnect()
    {
        if (mSocket >= 0)
        {
            spdlog::info("Closing existing socket.");
            close(mSocket);
            mSocket = -1;
        }
    }
    
    /**
     * @brief Set up the socket for communication
     * @param socketToBind the address of a socket to bind
     * @return true if socket setup was successful, false otherwise
     */
    virtual bool setupSocket(const std::string& socketToBind)
    {
        spdlog::info("Creating socket at path: {}", mSocketPath);
        mSocket = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (mSocket < 0)
        {
            spdlog::error("Failed to create the socket. Error code: {}", strerror(errno));
            return false;
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, socketToBind.c_str(), sizeof(addr.sun_path) - 1);
        
        // Bind to the address
        if (bind(mSocket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            spdlog::error("Failed to bind a socket: {0}, error message: {1}", socketToBind, strerror(errno));
            return false;
        }

        spdlog::info("Socket created successfully.");
        return true;
    }

    /** Path to the socket */
    std::string mSocketPath;
    /** Socket file descriptor */
    int mSocket;
};
