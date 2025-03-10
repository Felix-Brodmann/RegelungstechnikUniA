#include "../include/mpu6050.hpp"

namespace RegelungstechnikUniA
{
    void MPU6050::init(const char *t_device)
    {
        m_file = open(t_device, O_RDWR);
        if (m_file < 0)
        {
            throw std::runtime_error("Failed to open the i2c bus");
        }

        if (ioctl(m_file, I2C_SLAVE, m_address) < 0)
        {
            throw std::runtime_error("Failed to acquire bus access and/or talk to slave");
        }
    }

    void MPU6050::powerUp(const bool t_deviceReset)
    {
        uint8_t registerAddress = PWR_MGMT_1;
        uint8_t registerValue = t_deviceReset ? 0x80 : 0x00;
        uint8_t data[2] = {registerAddress, registerValue};
        if (write(m_file, data, 2) != 2)
        {
            throw std::runtime_error("Failed to power up the device");
        }
    }

    void MPU6050::setSampleFrequency(const int t_frequency)
    {
        if (t_frequency < 1 || t_frequency > 8000)
        {
            throw std::runtime_error("Invalid sample rate");
        }
        uint8_t registerAddress = SMPRT_DIV;
        uint8_t registerValue = static_cast<uint8_t>(std::floor(8000 / t_frequency - 1));
        uint8_t data[2] = {registerAddress, registerValue};
        if (write(m_file, data, 2) != 2)
        {
            throw std::runtime_error("Failed to configure the sample rate");
        }
    }

    void MPU6050::setGyroRange(const uint8_t t_range)
    {
        if (t_range != FS_SEL_250 && t_range != FS_SEL_500 && t_range != FS_SEL_1000 && t_range != FS_SEL_2000)
        {
            throw std::runtime_error("Invalid gyro range");
        }
        uint8_t registerAddress = GYRO_CONFIG;
        uint8_t registerValue = t_range;
        uint8_t data[2] = {registerAddress, registerValue};
        if (write(m_file, data, 2) != 2)
        {
            throw std::runtime_error("Failed to configure the gyro range");
        }
        m_gyroRange = t_range;
    }

    void MPU6050::setAccelRange(const uint8_t t_range)
    {
        if (t_range != AFS_SEL_2G && t_range != AFS_SEL_4G && t_range != AFS_SEL_8G && t_range != AFS_SEL_16G)
        {
            throw std::runtime_error("Invalid accel range");
        }
        uint8_t registerAddress = ACCEL_CONFIG;
        uint8_t registerValue = t_range;
        uint8_t data[2] = {registerAddress, registerValue};
        if (write(m_file, data, 2) != 2)
        {
            throw std::runtime_error("Failed to configure the accel range");
        }
        m_accelRange = t_range;
    }

    std::vector<double> MPU6050::readAccel(const int t_type)
    {
        if (t_type != IN_G && t_type != IN_M_PER_S2)
        {
            throw std::runtime_error("Invalid type for accel");
        }
        if (m_accelRange == -1)
        {
            throw std::runtime_error("Accel range not set. Call setAccelRange() first");
        }
        double lsbSensitivity = 0;
        switch (m_accelRange)
        {
            case AFS_SEL_2G:
                lsbSensitivity = LSB_SENSITIVITY_2G;
                break;
            case AFS_SEL_4G:
                lsbSensitivity = LSB_SENSITIVITY_4G;
                break;
            case AFS_SEL_8G:
                lsbSensitivity = LSB_SENSITIVITY_8G;
                break;
            case AFS_SEL_16G:
                lsbSensitivity = LSB_SENSITIVITY_16G;
                break;
            default:
                throw std::runtime_error("Invalid accel range");
        }
        uint8_t registerAddress = ACCEL_OUTPUT;
        if (write(m_file, &registerAddress, 1) != 1)
        {
            throw std::runtime_error("Failed to access the accel output register");
        }
        uint8_t data[6];
        if (read(m_file, data, 6) != 6)
        {
            throw std::runtime_error("Failed to read from the accel output register");
        }
        int16_t x = (data[0] << 8) | data[1];
        int16_t y = (data[2] << 8) | data[3];
        int16_t z = (data[4] << 8) | data[5];
        std::vector<double> accel;
        if (t_type == IN_G)
        {
            accel.push_back(x / lsbSensitivity);
            accel.push_back(y / lsbSensitivity);
            accel.push_back(z / lsbSensitivity);
        }
        else if (t_type == IN_M_PER_S2)
        {
            accel.push_back(x / lsbSensitivity * 9.81);
            accel.push_back(y / lsbSensitivity * 9.81);
            accel.push_back(z / lsbSensitivity * 9.81);
        }
        return accel;
    }

    double MPU6050::readTemp(const int t_type)
    {
        if (t_type != IN_KELVIN && t_type != IN_CELSIUS && t_type != IN_FARENHEIT)
        {
            throw std::runtime_error("Invalid type for temp");
        }
        uint8_t registerAddress = TEMP_OUTPUT;
        if (write(m_file, &registerAddress, 1) != 1)
        {
            throw std::runtime_error("Failed to access the temp output register");
        }
        uint8_t data[2];
        if (read(m_file, data, 2) != 2)
        {
            throw std::runtime_error("Failed to read from the temp output register");
        }
        int16_t temp = (data[0] << 8) | data[1];
        double temperature = temp / 340.0 + 36.53;
        if (t_type == IN_CELSIUS)
        {
            return temperature;
        }
        else if (t_type == IN_FARENHEIT)
        {
            return temperature * 1.8 + 32;
        }
        else if (t_type == IN_KELVIN)
        {
            return temperature + 273.15;
        }
        return 0;
    }

    std::vector<double> MPU6050::readGyro(const int t_type)
    {
        if (t_type != IN_DEG_PER_S && t_type != IN_RAD_PER_S)
        {
            throw std::runtime_error("Invalid type for gyro");
        }
        if (m_gyroRange == -1)
        {
            throw std::runtime_error("Gyro range not set. Call setGyroRange() first");
        }
        double lsbSensitivity = 0;
        switch (m_gyroRange)
        {
            case FS_SEL_250:
                lsbSensitivity = LSB_SENSITIVITY_250;
                break;
            case FS_SEL_500:
                lsbSensitivity = LSB_SENSITIVITY_500;
                break;
            case FS_SEL_1000:
                lsbSensitivity = LSB_SENSITIVITY_1000;
                break;
            case FS_SEL_2000:
                lsbSensitivity = LSB_SENSITIVITY_2000;
                break;
            default:
                throw std::runtime_error("Invalid gyro range");
        }
        uint8_t registerAddress = GYRO_OUTPUT;
        if (write(m_file, &registerAddress, 1) != 1)
        {
            throw std::runtime_error("Failed to access the gyro output register");
        }
        uint8_t data[6];
        if (read(m_file, data, 6) != 6)
        {
            throw std::runtime_error("Failed to read from the gyro output register");
        }
        int16_t x = (data[0] << 8) | data[1];
        int16_t y = (data[2] << 8) | data[3];
        int16_t z = (data[4] << 8) | data[5];
        std::vector<double> gyro;
        if (t_type == IN_DEG_PER_S)
        {
            gyro.push_back(x / lsbSensitivity - m_gyroOffset[X_AXIS]);
            gyro.push_back(y / lsbSensitivity - m_gyroOffset[Y_AXIS]);
            gyro.push_back(z / lsbSensitivity - m_gyroOffset[Z_AXIS]);
        }
        else if (t_type == IN_RAD_PER_S)
        {
            gyro.push_back((x / lsbSensitivity - m_gyroOffset[X_AXIS]) / 180.0 * M_PI);
            gyro.push_back((y / lsbSensitivity - m_gyroOffset[Y_AXIS]) / 180.0 * M_PI);
            gyro.push_back((z / lsbSensitivity - m_gyroOffset[Z_AXIS]) / 180.0 * M_PI);
        }
        return gyro;
    }

    void MPU6050::calibrateGyro(const time_t t_duration)
    {
        if (m_gyroRange == -1)
        {
            throw std::runtime_error("Gyro range not set. Call setGyroRange() first");
        }
        if (t_duration < 1)
        {
            throw std::runtime_error("Invalid calibration duration");
        }
        double x_offset, y_offset, z_offset;
        x_offset = y_offset = z_offset = 0;
        for (int i = 0; i < t_duration * 50; i++)
        {
            std::vector<double> gyro = readGyro(IN_DEG_PER_S);
            x_offset += gyro[X_AXIS];
            y_offset += gyro[Y_AXIS];
            z_offset += gyro[Z_AXIS];
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        m_gyroOffset[X_AXIS] = x_offset / (t_duration * 50);
        m_gyroOffset[Y_AXIS] = y_offset / (t_duration * 50);
        m_gyroOffset[Z_AXIS] = z_offset / (t_duration * 50);
    }

    const uint8_t MPU6050::whoAmI(const char *t_device)
    {
        int file = open(t_device, O_RDWR);
        if (file < 0)
        {
            throw std::runtime_error("Failed to open the i2c bus");
        }
        uint8_t registerAddress = WHO_AM_I;
        if (write(file, &registerAddress, 1) != 1)
        {
            throw std::runtime_error("Failed to access the WHO_AM_I register");
        }
        uint8_t data;
        if (read(file, &data, 1) != 1)
        {
            throw std::runtime_error("Failed to read from the WHO_AM_I register");
        }
        close(file);
        return data;
    }
}