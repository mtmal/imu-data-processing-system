# IMU Publisher-Subscriber System

## Overview

This project implements a publisher-subscriber system for IMU (Inertial Measurement Unit) data using Unix domain sockets. The system consists of a publisher that generates IMU data and broadcasts it to multiple subscribers. Subscribers can register with the publisher and receive real-time IMU data updates.

## Architecture

The system is built with a modular, object-oriented design following these key principles:

- **Separation of Concerns**: Each component has a specific responsibility
- **Dependency Injection**: Components depend on abstractions, not concrete implementations
- **Interface-Based Design**: Abstractions define contracts that implementations must fulfill

### Key Components

1. **IMUSocketHandler**: Base class providing common socket functionality
2. **IMUPublisher**: Publishes IMU data to registered subscribers
3. **IMUSubscriber**: Receives IMU data from the publisher
4. **IMUDataProvider**: Interface for obtaining IMU data
5. **RandomIMUDataProvider**: Implementation that generates random IMU data

### Communication Protocol

The system uses Unix domain datagram sockets for IPC (Inter-Process Communication). The protocol includes:

- Registration: Subscribers send a "REGISTER" message to the publisher
- Data Transfer: Publisher sends IMU data structures to registered subscribers
- Error Handling: Detection of disconnected subscribers and publisher timeouts

## Building the Project

### Prerequisites

- C++17 compatible compiler
- CMake 3.10 or higher
- spdlog library
- pthread library

### Build Instructions

```bash
mkdir build
cd build
cmake ..
make
```

## Running the Applications

### Publisher

```bash
./publisher --socket-path /tmp/imu_socket --log-level INFO --frequency-hz 500
```

Options:
- `--socket-path`: Path for the Unix domain socket (required)
- `--log-level`: Logging level (TRACE, DEBUG, INFO, WARN, ERROR)
- `--frequency-hz`: Publication frequency in Hz (default: 500)

### Subscriber

```bash
./subscriber --socket-path /tmp/imu_socket --log-level INFO --timeout-ms 100
```

Options:
- `--socket-path`: Path to the publisher's socket (required)
- `--log-level`: Logging level (TRACE, DEBUG, INFO, WARN, ERROR)
- `--timeout-ms`: Timeout for receiving data in milliseconds (default: 100)

## Design Features

### Extensibility

The system is designed to be easily extended:

1. **Custom IMU Data Sources**: Implement the `IMUDataProvider` interface to integrate with real IMU hardware
2. **Multiple Subscribers**: The publisher supports any number of concurrent subscribers
3. **Error Handling**: Robust error detection and recovery mechanisms

### Thread Safety

- The publisher uses mutex protection for the subscriber list
- The subscriber implements timeout detection for publisher failures
- Both components handle graceful termination with proper resource cleanup

### Resource Management

- Automatic cleanup of socket resources in destructors
- Detection and removal of disconnected subscribers
- Proper signal handling for graceful shutdown

## Class Documentation

### IMUDataProvider

Interface for providing IMU data from any source (random generator, hardware, etc.).

### RandomIMUDataProvider

Implementation of IMUDataProvider that generates random IMU data within realistic ranges.

### IMUSocketHandler

Base class providing common socket functionality for both publisher and subscriber.

### IMUPublisher

Publishes IMU data to registered subscribers using Unix domain sockets.

### IMUSubscriber

Receives IMU data from the publisher and processes it.

## Future Enhancements

Potential areas for future development:

1. Support for multiple data types beyond IMU data
2. Network socket support for distributed systems
3. Quality of Service (QoS) parameters for data delivery
4. Data filtering and validation mechanisms
5. Integration with real IMU hardware drivers
