#pragma once

#include <vector>
#include "IMUSocketHandler.h"
#include "providers/IMUDataProvider.h"

struct Parameters;

/**
 * @brief IMU data publisher using Unix domain sockets
 * 
 * This class publishes IMU data obtained from an IMUDataProvider
 * to subscribers using Unix domain datagram sockets. It handles
 * subscriber registration and data distribution.
 */
class IMUPublisher : public IMUSocketHandler
{
public:
    /**
     * @brief Constructor with IMU data provider
     * 
     * @param dataProvider The IMU data provider to use for data generation
     */
    IMUPublisher(IMUDataProvider& dataProvider);
    
    /**
     * @brief Destructor ensures proper cleanup of resources
     */
    virtual ~IMUPublisher();

    /**
     * @brief Initialize the publisher with parameters
     * 
     * Sets up the socket, initializes the data provider, and
     * prepares for publishing.
     * 
     * @param params The parameters structure
     * @return true if initialization was successful, false otherwise
     */
    bool initialise(const Parameters& params) override;

    /**
     * @brief Thread body implementation for the publisher
     * 
     * This method runs in a separate thread and handles the
     * periodic publishing of IMU data to subscribers.
     */
    void threadBody() override;

private:
    /**
     * @brief Check for new subscriber registrations
     * 
     * Listens for registration messages from subscribers and
     * adds them to the list of active subscribers.
     */
    void checkForRegistrations();
    
    /**
     * @brief Send IMU data to all registered subscribers
     * 
     * @param imuData The IMU data to send
     */
    void sendData(const Payload_IMU_t& imuData);
    
    /**
     * @brief Extended disconnect method to clean up socket files
     */
    void disconnect() override;

    IMUDataProvider& mDataProvider;               ///< Source of IMU data
    long mPeriodNs;                               ///< Publishing period in nanoseconds
    std::vector<struct sockaddr_un> mSubscribers; ///< List of subscriber addresses
    pthread_mutex_t mSubscribersMutex;            ///< Mutex to protect subscriber list
};
