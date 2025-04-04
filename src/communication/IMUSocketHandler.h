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
 * used by both publisher and subscriber components. It handles
 * socket creation, binding, and cleanup operations.
 */
template <typename T>
class IMUSocketHandler : public GenericThread<T>
{
public:
    /**
     * @brief Constructor initializes socket-related members
     */
    IMUSocketHandler() : mSocketPath(""), mSocket(-1) {}

    /**
     * @brief Destructor ensures socket resources are cleaned up
     */
    virtual ~IMUSocketHandler()
    {
        disconnect();
    }

    /**
     * @brief Initialize the socket handler with parameters
     * 
     * This method should be implemented by derived classes to
     * set up the socket handler with specific parameters.
     * 
     * @param params The parameters structure
     * @return true if initialization was successful, false otherwise
     */
    virtual bool initialise(const Parameters& params) = 0;

protected:
    /**
     * @brief Disconnect and clean up socket resources
     * 
     * Closes the socket if it's open. Derived classes should
     * override this to perform additional cleanup if needed.
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
     * 
     * Creates a Unix domain datagram socket and binds it to the specified path.
     * 
     * @param socketToBind The path to bind the socket to
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

    std::string mSocketPath; ///< Path to the socket
    int mSocket;             ///< Socket file descriptor
};
