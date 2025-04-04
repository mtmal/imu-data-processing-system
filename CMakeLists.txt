cmake_minimum_required(VERSION 3.10)
project(CloneCodingTask)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add pthread library
find_package(Threads REQUIRED)

# Add logging library (spdlog)
find_package(spdlog REQUIRED)

# Add utils directory to include paths
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/utils/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create publisher executable
add_executable(publisher 
    src/publisher.cpp
    src/IMUPublisher.cpp
    src/utils.cpp
    src/RandomIMUDataProvider.cpp
)
target_link_libraries(publisher 
    PRIVATE 
    pthread 
    spdlog::spdlog
)

# Create subscriber executable
add_executable(subscriber 
    src/subscriber.cpp
    src/IMUSubscriber.cpp
    src/utils.cpp
)
target_link_libraries(subscriber 
    PRIVATE 
    pthread 
    spdlog::spdlog
)

# Set include directories for both targets
target_include_directories(publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(subscriber PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
