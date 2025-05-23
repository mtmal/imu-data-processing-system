# IMU Data Processing System

This project implements a publisher-subscriber system for IMU (Inertial Measurement Unit) data using Unix domain sockets. It includes AHRS (Attitude and Heading Reference System) algorithms for orientation estimation.

## Overview

The system consists of two main components:

1. **Publisher**: Generates IMU data (either random or from a file) and publishes it to subscribers.
2. **Subscriber**: Receives IMU data from the publisher, processes it with AHRS algorithms, and displays the results.

## Features

- Unix domain socket communication between publisher and subscriber
- Multiple subscriber support
- Configurable data generation frequency
- Timeout handling for detecting disconnected publishers
- AHRS algorithms for orientation estimation
- Clean object-oriented design with factory pattern for AHRS creation
- Real-time execution support (experimental)

## Architecture

The system is built with a modular, object-oriented design following these key principles:

- **Separation of Concerns**: Each component has a specific responsibility
- **Dependency Injection**: Components depend on abstractions, not concrete implementations
- **Interface-Based Design**: Abstractions define contracts that implementations must fulfill

### Communication

The system uses Unix domain datagram sockets for communication. The publisher creates a socket and listens for registration messages from subscribers. Once a subscriber registers, the publisher sends IMU data to all registered subscribers.

### Key Components

1. **IMUSocketHandler**: Base class providing common socket functionality
2. **IMUPublisher**: Publishes IMU data to registered subscribers
3. **IMUSubscriber**: Receives IMU data from the publisher
4. **IMUDataProvider**: Interface for obtaining IMU data
5. **RandomIMUDataProvider**: Implementation that generates random IMU data
6. **AHRS**: Abstract base class for orientation estimation algorithms
7. **AHRSFactory**: Factory for creating AHRS instances based on user selection

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

### Publisher

```bash
./publisher --socket-path /tmp/imu_socket --frequency-hz 100 --log-level INFO [--real-time] [--priority 80] [--policy FIFO]
```

Options:
- `--socket-path`: Path to the Unix domain socket
- `--frequency-hz`: Data generation frequency in Hz
- `--log-level`: Logging level (TRACE, DEBUG, INFO, WARN, ERROR)
- `--real-time`: Enable real-time thread configuration
- `--priority`: Thread priority (1-99, only with --real-time)
- `--policy`: Scheduling policy (FIFO or RR, only with --real-time)

### Subscriber

```bash
./subscriber --socket-path /tmp/imu_socket --log-level INFO --timeout-ms 5000 --ahrs-type madgwick [--real-time] [--priority 75] [--policy FIFO]
```

Options:
- `--socket-path`: Path to the Unix domain socket (must match publisher)
- `--log-level`: Logging level (TRACE, DEBUG, INFO, WARN, ERROR)
- `--timeout-ms`: Timeout in milliseconds for detecting disconnected publisher
- `--ahrs-type`: AHRS algorithm to use (none, madgwick, simple)
- `--real-time`: Enable real-time thread configuration
- `--priority`: Thread priority (1-99, only with --real-time)
- `--policy`: Scheduling policy (FIFO or RR, only with --real-time)

## Real-Time Execution Support (Experimental)

**⚠️ IMPORTANT: The real-time features have not been tested in a real-time environment. Use at your own risk.**

The system includes experimental support for real-time execution with the following features:

- Real-time thread scheduling (SCHED_FIFO or SCHED_RR)
- Configurable thread priorities (1-99)
- Memory locking to prevent paging
- Priority inheritance for mutexes
- Separate priority levels for publisher and subscriber

### Real-Time Requirements

To use real-time features:

1. **System Requirements**:
   - Real-time kernel (PREEMPT_RT patch or similar)
   - Appropriate user permissions for real-time scheduling

2. **System Configuration**:
   ```bash
   # Add to /etc/security/limits.conf:
   your_username    hard    rtprio    99
   your_username    soft    rtprio    99
   ```

3. **Recommended Practices**:
   - Disable CPU frequency scaling
   - Disable power management features
   - Consider CPU isolation using kernel boot parameters
   - Configure network interfaces with appropriate priorities

### Real-Time Usage Example

```bash
# Publisher with real-time enabled
sudo ./publisher --socket-path /tmp/imu_socket --frequency-hz 100 --real-time --priority 80 --policy FIFO

# Subscriber with real-time enabled
sudo ./subscriber --socket-path /tmp/imu_socket --timeout-ms 5000 --ahrs-type madgwick --real-time --priority 75 --policy FIFO
```

## Design Patterns

The project uses several design patterns:

1. **Factory Pattern**: The `AHRSFactory` creates appropriate AHRS instances based on user selection.
2. **Strategy Pattern**: Different AHRS algorithms implement the same interface, allowing them to be used interchangeably.
3. **Observer Pattern**: The publisher-subscriber model follows the observer pattern.
4. **Template Method Pattern**: Base classes define the skeleton of operations, with specific steps implemented by derived classes.

## Modern C++ Features

The project demonstrates several modern C++17 features:

1. **std::optional** and **std::variant**: Demonstrated as an alternative to inheritance-based polymorphism in the `VariantAHRS` class.

## Thread Safety

- The publisher uses mutex protection for the subscriber list
- The subscriber implements timeout detection for publisher failures
- Both components handle graceful termination with proper resource cleanup

## Resource Management

- Automatic cleanup of socket resources in destructors
- Detection and removal of disconnected subscribers
- Proper signal handling for graceful shutdown

## Dependencies

- C++17 compatible compiler
- CMake 3.10 or higher
- spdlog for logging
- pthread for threading
