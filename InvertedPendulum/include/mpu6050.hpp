#ifndef MPU6050_HPP
#define MPU6050_HPP

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
#include <iostream>

#define SMPRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_OUTPUT 0x3B
#define TEMP_OUTPUT 0x41
#define GYRO_OUTPUT 0x43
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75

#define FS_SEL_250 0x00
#define FS_SEL_500 0x08
#define FS_SEL_1000 0x10
#define FS_SEL_2000 0x18

#define AFS_SEL_2G 0x00
#define AFS_SEL_4G 0x08
#define AFS_SEL_8G 0x10
#define AFS_SEL_16G 0x18

#define LSB_SENSITIVITY_250 131.0
#define LSB_SENSITIVITY_500 65.5
#define LSB_SENSITIVITY_1000 32.8
#define LSB_SENSITIVITY_2000 16.4

#define LSB_SENSITIVITY_2G 16384.0
#define LSB_SENSITIVITY_4G 8192.0
#define LSB_SENSITIVITY_8G 4096.0
#define LSB_SENSITIVITY_16G 2048.0

#define IN_G 0
#define IN_M_PER_S2 1
#define IN_KELVIN 0
#define IN_CELSIUS 1
#define IN_FARENHEIT 2
#define IN_DEG_PER_S 0
#define IN_RAD_PER_S 1

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

namespace RegelungstechnikUniA
{
    class MPU6050
    {
    private:
        int m_file = 0;
        const uint8_t m_address = 0x68;
        int m_gyroRange = -1;
        int m_accelRange = -1;
        double m_gyroOffset[3] = {0.0, 0.0, 0.0};

    public:
        void init(const char *t_device);
        void powerUp(const bool t_deviceReset);
        void setSampleFrequency(const int t_frequency);
        void setGyroRange(const uint8_t t_range);
        void setAccelRange(const uint8_t t_range);
        std::vector<double> readAccel(const int t_type);
        double readTemp(const int t_type);
        std::vector<double> readGyro(const int t_type);
        void calibrateGyro(const time_t t_duration);

    public:
        static const uint8_t whoAmI(const char *t_device);
    };
}

#endif