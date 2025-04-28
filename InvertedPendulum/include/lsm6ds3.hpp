#ifndef LSM6DS3_HPP
#define LSM6DS3_HPP

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

#define GRAVITY 9.80665

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

namespace RegelungstechnikUniA
{
    class LSM6DS3
    {
    public:
        enum class AccelerometerOutputDataRate
        {
            POWER_DOWN = 0x00,
            LOW_POWER_12_5Hz = 0x10,
            LOW_POWER_26Hz = 0x20,
            LOW_POWER_52Hz = 0x30,
            NORMAL_MODE_104Hz = 0x40,
            NORMAL_MODE_208Hz = 0x50,
            HIGH_PERFORMANCE_416Hz = 0x60,
            HIGH_PERFORMANCE_833Hz = 0x70,
            HIGH_PERFORMANCE_1660Hz = 0x80,
            HIGH_PERFORMANCE_3330Hz = 0x90,
            HIGH_PERFORMANCE_6660Hz = 0xA0,
        };
    
        enum class AccelerometerFullScale
        {
            FS_UNKNOWN = -1,
            FS_2G = 0x00,
            FS_4G = 0x08,
            FS_8G = 0x0C,
            FS_16G = 0x04
        };

        enum class AccelerometerFilterBandwidth
        {
            BW_50Hz = 0x03,
            BW_100Hz = 0x02,
            BW_200Hz = 0x01,
            BW_400Hz = 0x00,
        };
    
        enum class GyroscopeOutputDataRate
        {
            POWER_DOWN = 0x00,
            LOW_POWER_12_5Hz = 0x10,
            LOW_POWER_26Hz = 0x20,
            LOW_POWER_52Hz = 0x30,
            NORMAL_MODE_104Hz = 0x40,
            NORMAL_MODE_208Hz = 0x50,
            HIGH_PERFORMANCE_416Hz = 0x60,
            HIGH_PERFORMANCE_833Hz = 0x70,
            HIGH_PERFORMANCE_1660Hz = 0x80,
        };
    
        enum class GyroscopeFullScale
        {
            FS_UNKNOWN = -1,
            FS_125DPS = 0x02,
            FS_250DPS = 0x00,
            FS_500DPS = 0x04,
            FS_1000DPS = 0x08,
            FS_2000DPS = 0x0C
        };

        enum class AccelerometerUnit
        {
            IN_MG,
            IN_G,
            IN_M_PER_S2
        };

        enum class GyroscopeUnit
        {
            IN_MDEG_PER_S,
            IN_DEG_PER_S,
            IN_RAD_PER_S
        };

    private:
        const uint8_t WHO_I_WOULD_LIKE_TO_BE = 0x6A;
        const uint8_t WHO_AM_I_REGISTER = 0x0F;
        const uint8_t ACCELEROMETER_CONFIG_REGISTER = 0x10;
        const uint8_t GYROSCOPE_CONFIG_REGISTER = 0x11;
        const uint8_t ACCELEROMETER_DATA_REGISTER = 0x28;
        const uint8_t GYROSCOPE_DATA_REGISTER = 0x22;

    private:
        int m_fileDescriptor = -1;
        uint8_t m_address = 0x6A;
        AccelerometerFullScale m_accelFullScale = AccelerometerFullScale::FS_UNKNOWN;
        GyroscopeOutputDataRate m_gyroOutputDataRate = GyroscopeOutputDataRate::POWER_DOWN;
        GyroscopeFullScale m_gyroFullScale = GyroscopeFullScale::FS_UNKNOWN;
        std::vector<double> m_gyroOffset = {0.0, 0.0, 0.0};

        void writeRegister(const uint8_t t_register, const uint8_t t_value);
        void readRegister(const uint8_t t_register, uint8_t *t_buffer, const uint8_t t_length);

    public:
        LSM6DS3(const char *t_device = "/dev/i2c-1", const uint8_t t_address = 0x6A);
        void configureAccelerometer(const AccelerometerOutputDataRate t_odr, const AccelerometerFullScale t_fs, const AccelerometerFilterBandwidth t_bw);
        void configureGyroscope(const GyroscopeOutputDataRate t_odr, const GyroscopeFullScale t_fs);
        void calibrateGyroscope(const time_t t_duration);
        std::vector<double> readAccelerometer(const AccelerometerUnit t_type);
        std::vector<double> readGyroscope(const GyroscopeUnit t_type);
        const bool amITalkingToTheSensor();
    };
}

#endif