#pragma once

#include <memory>
#include <optional>

#include "ahrs/VariantAHRS.h"
#include "IMUSocketHandler.h"

struct Parameters;

/**
 * @brief IMU data subscriber using Unix domain sockets
 * 
 * This class subscribes to IMU data published by an IMUPublisher
 * using Unix domain datagram sockets. It handles registration with
 * the publisher and processing of received IMU data.
 */
class IMUSubscriber : public IMUSocketHandler<IMUSubscriber>
{
public:
    /**
     * @brief Constructor initializes the subscriber
     */
    IMUSubscriber();
    
    /**
     * @brief Destructor ensures proper cleanup of resources
     */
    virtual ~IMUSubscriber();

    /**
     * @brief Initialize the subscriber with parameters
     * 
     * Sets up the socket, registers with the publisher, and
     * prepares for receiving data.
     * 
     * @param params The parameters structure
     * @return true if initialization was successful, false otherwise
     */
    bool initialise(const Parameters& params) override;

    /**
     * @brief Thread body implementation for the subscriber
     * 
     * This method runs in a separate thread and handles the
     * reception and processing of IMU data from the publisher.
     * 
     * @return Thread result (always nullptr)
     */
    void* threadBody();

private:
    /**
     * @brief Registers this subscriber to publisher
     * 
     * Sends a registration message to the publisher to establish
     * the connection and start receiving data.
     * 
     * @return true if the registration was successful
     */
    bool registerToServer();

    /**
     * @brief Sets the tiemout for the socket.
     * 
     * @return true if the timeout was successfully set
     */
    bool setSocketTimeout(const ulong timeoutMs);
    
    /**
     * @brief Extended disconnect method to clean up socket files
     */
    void disconnect() override;

    std::string mClientSocketPath; ///< Path to the client socket
    std::optional<VariantAHRS> mAhrs; ///< AHRS processor using variant approach
};