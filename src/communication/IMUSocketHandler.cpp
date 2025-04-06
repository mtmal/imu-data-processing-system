#include <spdlog/spdlog.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include "communication/IMUSocketHandler.h"

IMUSocketHandler::IMUSocketHandler(const bool realTime) 
: mSocket(-1),
  mParameters(),
  mThread(0),
  mRun(false)
{
}

IMUSocketHandler::~IMUSocketHandler()
{
    stopThread();
    disconnect();
}

bool IMUSocketHandler::initialise(const Parameters& params)
{
    mParameters = params;
    return true;
}

bool IMUSocketHandler::startThread(const int priority, const int policy)
{
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    
    if (mParameters.mRealTime)
    {
        // Set real-time scheduling policy
        pthread_attr_setschedpolicy(&attr, policy);
        
        // Set priority for real-time scheduling
        struct sched_param param;
        param.sched_priority = priority;
        pthread_attr_setschedparam(&attr, &param);
        
        // Explicitly state that we want to set scheduling attributes
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        
        // Lock memory to prevent paging
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        {
            spdlog::error("Failed to lock memory pages: {}", strerror(errno));
            return false;
        }
    }
    
    bool retVal = (0 == pthread_create(&mThread, &attr, IMUSocketHandler::startThread, this));
    mRun.store(retVal, std::memory_order_release);
    
    pthread_attr_destroy(&attr);
    return retVal;
}

void IMUSocketHandler::stopThread()
{
    mRun.store(false, std::memory_order_release);
    if (mThread > 0)
    {
        pthread_join(mThread, nullptr);
        mThread = 0;
        
        if (mParameters.mRealTime)
        {
            munlockall(); // Unlock memory pages
        }
    }
}

void IMUSocketHandler::disconnect()
{
    if (mSocket >= 0)
    {
        spdlog::info("Closing existing socket.");
        close(mSocket);
        mSocket = -1;
    }
}

bool IMUSocketHandler::setupSocket(const std::string& socketToBind)
{
    spdlog::info("Creating socket at path: {}", mParameters.mSocketPath);
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

void* IMUSocketHandler::startThread(void* instance)
{
    IMUSocketHandler* socketHandler = static_cast<IMUSocketHandler*>(instance);
    socketHandler->threadBody();
    return nullptr;
}
