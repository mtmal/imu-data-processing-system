#pragma once

#include <atomic>
#include <pthread.h>

#include "core/Parameters.h"

/**
 * @brief Base class for IMU socket communication handling
 * 
 * This class provides common functionality for socket-based IPC
 * used by both publisher and subscriber components. It handles
 * socket creation, binding, and cleanup operations.
 */
class IMUSocketHandler
{
public:
    /**
     * @brief Constructor initializes socket and thread-related members
     * @param realTime Flag to enable real-time thread configuration
     */
    IMUSocketHandler(const bool realTime = false);

    /**
     * @brief Destructor ensures proper cleanup
     */
    virtual ~IMUSocketHandler();

    /**
     * @brief Initialize the socket handler with parameters
     * 
     * This method should be implemented by derived classes to
     * set up the socket handler with specific parameters.
     * 
     * @param params The parameters structure
     * @return true if initialization was successful, false otherwise
     */
    virtual bool initialise(const Parameters& params);

    /**
     * @brief Starts the handler thread with real-time settings if enabled
     * 
     * @param priority Thread priority (1-99 for real-time, ignored if realTime was set to false)
     * @param policy Scheduling policy (SCHED_FIFO or SCHED_RR for real-time)
     * @return true if the thread was successfully started
     */
    bool startThread(const int priority = 50, const int policy = SCHED_FIFO);

    /**
     * @brief Stops the handler thread
     */
    void stopThread();

    /**
     * @brief Check if the thread is running
     * 
     * @return true if the thread is running
     */
    inline bool isRunning() const
    {
        return mRun.load(std::memory_order_acquire);
    }

    /**
     * @brief the main body of the thread
     */
    virtual void threadBody() = 0;

protected:
    /**
     * @brief Disconnect and clean up socket resources
     * 
     * Closes the socket if it's open. Derived classes should
     * override this to perform additional cleanup if needed.
     */
    virtual void disconnect();
    
    /**
     * @brief Set up the socket for communication
     * 
     * Creates a Unix domain datagram socket and binds it to the specified path.
     * 
     * @param socketToBind The path to bind the socket to
     * @return true if socket setup was successful, false otherwise
     */
    virtual bool setupSocket(const std::string& socketToBind);

    int mSocket;             ///< Socket file descriptor
    Parameters mParameters;  ///< Configuration parameters

private:
    /**
     * @brief Static thread entry point
     */
    static void* startThread(void* instance);

    pthread_t mThread;        ///< Thread handle
    std::atomic<bool> mRun;   ///< Thread running flag
};
