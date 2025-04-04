#include <spdlog/spdlog.h>
#include "IMUTypes.h"
#include "RandomIMUDataProvider.h"

RandomIMUDataProvider::RandomIMUDataProvider()
: mGen(),
  mAccDist(-16000.0f, 16000.0f),
  mGyroDist(-2000000.0f, 2000000.0f),
  mMagDist(200.0f, 600.0f)
{
}

bool RandomIMUDataProvider::initialize()
{
    std::random_device rd;
    mGen = std::mt19937(rd());
    spdlog::info("Random IMU data generator initialized");
    return true;
}

void RandomIMUDataProvider::getIMUData(Payload_IMU_t& imuData)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    imuData.xAcc = mAccDist(mGen);
    imuData.yAcc = mAccDist(mGen);
    imuData.zAcc = mAccDist(mGen);
    imuData.timestampAcc = static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);

    imuData.xGyro = mGyroDist(mGen);
    imuData.yGyro = mGyroDist(mGen);
    imuData.zGyro = mGyroDist(mGen);
    imuData.timestampGyro = imuData.timestampAcc;

    imuData.xMag = mMagDist(mGen);
    imuData.yMag = mMagDist(mGen);
    imuData.zMag = mMagDist(mGen);
    imuData.timestampMag = imuData.timestampAcc;
} 