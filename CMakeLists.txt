cmake_minimum_required(VERSION 3.10)
project(CloneCodingTask)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add pthread library
find_package(Threads REQUIRED)

# Add logging library (spdlog)
find_package(spdlog REQUIRED)

# Create publisher executable
add_executable(publisher 
    src/publisher.cpp
    src/communication/IMUPublisher.cpp
    src/communication/IMUSocketHandler.cpp
    src/utils/utils.cpp
    src/providers/RandomIMUDataProvider.cpp
)
# Add include directories for publisher
target_include_directories(publisher PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/providers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
)
target_link_libraries(publisher PRIVATE pthread spdlog::spdlog)

# Create subscriber executable
add_executable(subscriber
    src/subscriber.cpp
    src/communication/IMUSubscriber.cpp
    src/communication/IMUSocketHandler.cpp
    src/utils/utils.cpp
    src/ahrs/AHRS.cpp
    src/ahrs/MadgwickAHRS.cpp
    src/ahrs/SimpleAHRS.cpp
)
# Add include directories for subscriber
target_include_directories(subscriber PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/providers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/communication
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ahrs
)
target_link_libraries(subscriber PRIVATE pthread spdlog::spdlog)
