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

        void LSM6DS3::writeRegister(const uint8_t t_register, const uint8_t t_value)
        {
            uint8_t data[2] = {t_register, t_value};
            if (write(m_fileDescriptor, data, 2) != 2)
            {
                throw std::runtime_error("Failed to write to the register");
            }
        }

        void LSM6DS3::readRegister(const uint8_t t_register, uint8_t *t_buffer, const uint8_t t_length)
        {
            uint8_t data[1] = {t_register};
            if (write(m_fileDescriptor, data, 1) != 1)
            {
                throw std::runtime_error("Failed to request the register");
            }
            if (read(m_fileDescriptor, t_buffer, t_length) != t_length)
            {
                throw std::runtime_error("Failed to read from the register");
            }
        }

    public:
        LSM6DS3::LSM6DS3(const char *t_device, const uint8_t t_address)
        {
            m_address = t_address;
            m_fileDescriptor = open(t_device, O_RDWR);
            if (m_fileDescriptor < 0)
            {
                throw std::runtime_error("Failed to open the i2c bus");
            }
            if (ioctl(m_fileDescriptor, I2C_SLAVE, m_address) < 0)
            {
                throw std::runtime_error("Failed to acquire bus access and/or talk to slave");
            }
        }

        void LSM6DS3::configureAccelerometer(const AccelerometerOutputDataRate t_odr, const AccelerometerFullScale t_fs, const AccelerometerFilterBandwidth t_bw)
        {
            uint8_t value = static_cast<uint8_t>(t_odr) | static_cast<uint8_t>(t_fs) | static_cast<uint8_t>(t_bw);
            writeRegister(ACCELEROMETER_CONFIG_REGISTER, value);
            m_accelFullScale = t_fs;
        }

        void LSM6DS3::configureGyroscope(const GyroscopeOutputDataRate t_odr, const GyroscopeFullScale t_fs)
        {
            uint8_t value = static_cast<uint8_t>(t_odr) | static_cast<uint8_t>(t_fs);
            writeRegister(GYROSCOPE_CONFIG_REGISTER, value);
            m_gyroOutputDataRate = t_odr;
            m_gyroFullScale = t_fs;
        }

        void LSM6DS3::calibrateGyroscope(const time_t t_duration)
        {
            // Define the variables for calibration
            std::vector<double> gyro;
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            int frequency = 100;
            int deltaTime = 1 / frequency * 1000000; // in microseconds
            int samples = t_duration * frequency * 10; 

            // Loop over the samples
            for (uint i = 0; i < samples; i++)
            {
                // Read the values from the sensor in mDeg/s
                gyro = readGyroscope(GyroscopeUnit::IN_MDEG_PER_S);

                // Add the values to the sum
                x += gyro[X_AXIS];
                y += gyro[Y_AXIS];
                z += gyro[Z_AXIS];

                // Sleep for the time of the sample rate
                usleep(deltaTime);
            }

            // Calculate the average values
            m_gyroOffset[X_AXIS] = (double)(x / samples);
            m_gyroOffset[Y_AXIS] = (double)(y / samples);
            m_gyroOffset[Z_AXIS] = (double)(z / samples);
        }

        std::vector<double> LSM6DS3::readAccelerometer(const AccelerometerUnit t_type)
        {
            // Get the resolution, depending on the FullScale and check if the FullScale was set
            double scale = 0.0;
            switch (m_accelFullScale)
            {
                case AccelerometerFullScale::FS_2G:
                    scale = 0.061;
                    break;
                case AccelerometerFullScale::FS_4G:
                    scale = 0.122;
                    break;
                case AccelerometerFullScale::FS_8G:
                    scale = 0.244;
                    break;
                case AccelerometerFullScale::FS_16G:
                    scale = 0.488;
                    break;
                default:
                    throw std::runtime_error("Accelerometer range not set. Call configureAccelerometer(2) first");
            }

            // Read the values from the sensor
            uint8_t buffer[6];
            readRegister(ACCELEROMETER_DATA_REGISTER, buffer, 6);

            // Parse the values from each Axis into one Axis value
            int16_t x = (buffer[1] << 8) | buffer[0];
            int16_t y = (buffer[3] << 8) | buffer[2];
            int16_t z = (buffer[5] << 8) | buffer[4];

            // Transform the values into the according unit
            std::vector<double> accel;
            switch(t_type)
            {
                case AccelerometerUnit::IN_MG:
                    accel.push_back(x * scale);
                    accel.push_back(y * scale);
                    accel.push_back(z * scale);
                    break;

                case AccelerometerUnit::IN_G:
                    accel.push_back(x * scale / 1000.0);
                    accel.push_back(y * scale / 1000.0);
                    accel.push_back(z * scale / 1000.0);
                    break;

                case AccelerometerUnit::IN_M_PER_S2:
                    accel.push_back(x * scale / 1000.0 * GRAVITY);
                    accel.push_back(y * scale / 1000.0 * GRAVITY);
                    accel.push_back(z * scale / 1000.0 * GRAVITY);
                    break;
            }
            return accel;
        }

        std::vector<double> LSM6DS3::readGyroscope(const GyroscopeUnit t_type)
        {
            // Get the resolution, depending on the FullScale and check if the FullScale was set
            double scale = 0.0;
            switch (m_gyroFullScale)
            {
                case GyroscopeFullScale::FS_125DPS:
                    scale = 4.375;
                    break;
                case GyroscopeFullScale::FS_250DPS:
                    scale = 8.75;
                    break;
                case GyroscopeFullScale::FS_500DPS:
                    scale = 17.5;
                    break;
                case GyroscopeFullScale::FS_1000DPS:
                    scale = 35.0;
                    break;
                case GyroscopeFullScale::FS_2000DPS:
                    scale = 70.0;
                    break;
                default:
                    throw std::runtime_error("Accelerometer range not set. Call configureAccelerometer(2) first");
            }

            // Read the values from the sensor
            uint8_t buffer[6];
            readRegister(GYROSCOPE_DATA_REGISTER, buffer, 6);

            // Parse the values from each Axis into one Axis value
            int16_t x = (buffer[1] << 8) | buffer[0];
            int16_t y = (buffer[3] << 8) | buffer[2];
            int16_t z = (buffer[5] << 8) | buffer[4];

            // Transform the values into the according unit
            std::vector<double> gyro;
            switch(t_type)
            {
                case GyroscopeUnit::IN_MDEG_PER_S:
                    gyro.push_back(x * scale - m_gyroOffset[X_AXIS]);
                    gyro.push_back(y * scale - m_gyroOffset[Y_AXIS]);
                    gyro.push_back(z * scale - m_gyroOffset[Z_AXIS]);
                    break;
                case GyroscopeUnit::IN_DEG_PER_S:
                    gyro.push_back((x * scale - m_gyroOffset[X_AXIS]) / 1000.0);
                    gyro.push_back((y * scale - m_gyroOffset[Y_AXIS]) / 1000.0);
                    gyro.push_back((z * scale - m_gyroOffset[Z_AXIS]) / 1000.0);
                    break;
                case GyroscopeUnit::IN_RAD_PER_S:
                    gyro.push_back((x * scale - m_gyroOffset[X_AXIS]) / 1000.0 * (M_PI / 180));
                    gyro.push_back((y * scale - m_gyroOffset[Y_AXIS]) / 1000.0 * (M_PI / 180));
                    gyro.push_back((z * scale - m_gyroOffset[Z_AXIS]) / 1000.0 * (M_PI / 180));
                    break;
            }
            return gyro;
        }

        const bool LSM6DS3::amITalkingToTheSensor()
        {
            uint8_t whoIReallyAm;
            readRegister(WHO_AM_I_REGISTER, &whoIReallyAm, 1);
            if (whoIReallyAm ==  WHO_I_WOULD_LIKE_TO_BE)
            {
                return true;
            }
            return false;
        }
    };
}

#endif
