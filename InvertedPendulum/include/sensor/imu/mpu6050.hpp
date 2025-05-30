#ifndef SENSOR_IMU_MPU6050_HPP
#define SENSOR_IMU_MPU6050_HPP

// Checks if the platform is Linux and ARM-based (e.g., Raspberry Pi)
#if defined(__linux__) && (defined(__arm__) || defined(__aarch64__))

#include <stdint.h>
#include <string>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <unistd.h>

#include "sensor/sensor.hpp"
#include "utility/types.hpp"
#include "utility/units.hpp"
#include "utility/constants.hpp"
#include "utility/i2c-device.hpp"

namespace control_engineering_uni_a
{

/**
 * @class MPU6050
 * @brief Driver class for the MPU6050 6-axis IMU sensor (accelerometer + gyroscope + thermometer).
 * @details This class provides an interface for configuring and reading data from the MPU6050 sensor via I2C. It supports configuration of accelerometer and gyroscope full-scale ranges, unit selection for all sensor types, sample frequency adjustment, and gyroscope calibration. Sensor data is read and converted to user-specified units.
 * @author Felix Brodmann
 * @date 2025-05-26
 */
class MPU6050 : public Sensor
{
public:
    enum class AccelerometerFullScale
    {
        FS_2G = 0x00,
        FS_4G = 0x08,
        FS_8G = 0x10,
        FS_16G = 0x18
    }

    enum class GyroscopeFullScale
    {
        FS_250DPS = 0x00,
        FS_500DPS = 0x08,
        FS_1000DPS = 0x10,
        FS_2000DPS = 0x18
    }

private:
    constexpr uint8_t WHO_AM_I_REGISTER = 0x75;
    constexpr uint8_t ACCELEROMETER_CONFIG_REGISTER = 0x1C;
    constexpr uint8_t GYROSCOPE_CONFIG_REGISTER = 0x1B;
    constexpr uint8_t TEMPERATURE_DATA_REGISTER = 0x41;
    constexpr uint8_t GYROSCOPE_DATA_REGISTER = 0x43;
    constexpr uint8_t ACCELEROMETER_DATA_REGISTER = 0x3B;
    constexpr uint8_t SAMPLE_RATE_REGISTER = 0x19;
    constexpr uint8_t POWER_REGISTER = 0x6B;

    std::unique_ptr<I2CDevice> m_i2cDevice;
    AccelerometerFullScale m_accelFullScale = AccelerometerFullScale::FS_2G; // Accelerometer full-scale range
    AccelerometerUnit m_accelerometerUnit = AccelerometerUnit::IN_G; // Accelerometer unit of measurement
    GyroscopeFullScale m_gyroFullScale = GyroscopeFullScale::FS_500DPS; // Gyroscope full-scale range
    GyroscopeUnit m_gyroscopeUnit = GyroscopeUnit::IN_DEG_PER_S; // Gyroscope unit of measurement
    TemperatureUnit m_temperatureUnit = TemperatureUnit::IN_CELSIUS; // Temperature unit of measurement
    Vector3D m_gyroscopeOffset; // Gyroscope offset for calibration
    int m_sampleRateHz = 100; // Default sample rate in Hz

    /**
     * @brief Read the accelerometer data from the MPU6050 sensor.
     * @details This method reads the accelerometer data from the sensor, applies the configured full-scale range and unit conversion, and returns the acceleration vector.
     * @return A Vector3D object containing the accelerometer data in the configured unit.
     * @throws std::runtime_error if reading the accelerometer data fails.
     */
    Vector3D readAccelerometer()
    {
        // Get the resolution, depending on the FullScale
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
        }

        // Read the values from the sensor
        uint8_t buffer[6];
        try
        {
            m_i2cDevice->readRegister(ACCELEROMETER_DATA_REGISTER, buffer, 6);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to read accelerometer data: " + std::string(e.what()));
        }

        // Parse the values from each Axis into one Axis value
        int16_t x = (buffer[1] << 8) | buffer[0];
        int16_t y = (buffer[3] << 8) | buffer[2];
        int16_t z = (buffer[5] << 8) | buffer[4];

        // Cast to double to avoid Wnarrowing
        double x_d = static_cast<double>(x);
        double y_d = static_cast<double>(y);
        double z_d = static_cast<double>(z);

        // Transform the values into the according unit
        Vector3D accelerometerValue;
        switch(m_accelerometerUnit)
        {
            case AccelerometerUnit::IN_MG:
                accelerometerValue = Vector3D{x, y, z} * scale;
                break;

            case AccelerometerUnit::IN_G:
                accelerometerValue = Vector3D{x, y, z} * scale / 1000.0;
                break;

            case AccelerometerUnit::IN_M_PER_S2:
                accelerometerValue = Vector3D{x, y, z} * scale / 1000.0 * GRAVITY;
                break;
        }
        return accelerometerValue;
    }

    /**
     * @brief Read the gyroscope data from the MPU6050 sensor.
     * @details This method reads the gyroscope data from the sensor, applies the configured full-scale range and unit conversion, and returns the gyroscope vector.
     * @return A Vector3D object containing the gyroscope data in the configured unit.
     * @throws std::runtime_error if reading the gyroscope data fails.
     */
    Vector3D readGyroscope()
    {
        // Get the resolution, depending on the FullScale and check if the FullScale was set
        double scale = 0.0;
        switch (m_gyroFullScale)
        {
            case GyroscopeFullScale::FS_250DPS:
                scale = 7.634;
                break;
            case GyroscopeFullScale::FS_500DPS:
                scale = 15.267;
                break;
            case GyroscopeFullScale::FS_1000DPS:
                scale = 30.488;
                break;
            case GyroscopeFullScale::FS_2000DPS:
                scale = 60.976;
                break;
        }

        // Read the values from the sensor
        uint8_t buffer[6];
        try
        {
            m_i2cDevice->readRegister(GYROSCOPE_DATA_REGISTER, buffer, 6);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to read gyroscope data: " + std::string(e.what()));
        }

        // Parse the values from each Axis into one Axis value
        int16_t x = (buffer[1] << 8) | buffer[0];
        int16_t y = (buffer[3] << 8) | buffer[2];
        int16_t z = (buffer[5] << 8) | buffer[4];

        // Transform the values into the according unit
        Vector3D gyroscopeValue;
        switch(m_gyroscopeUnit)
        {
            case GyroscopeUnit::IN_MDEG_PER_S:
                gyroscopeValue = Vector3D{x, y, z} * scale - m_gyroscopeOffset;
                break;
            case GyroscopeUnit::IN_DEG_PER_S:
                gyroscopeValue = Vector3D{x, y, z} * scale / 1000.0 - m_gyroscopeOffset;
                break;
            case GyroscopeUnit::IN_RAD_PER_S:
                gyroscopeValue = Vector3D{x, y, z} * scale / 1000.0 * RAD_SCALE - m_gyroscopeOffset;
                break;
        }
        return gyroscopeValue;
    }

    /**
     * @brief Read the temperature data from the MPU6050 sensor.
     * @details This method reads the temperature data from the sensor, applies the configured unit conversion, and returns the temperature value.
     * @return The temperature value in the configured unit.
     * @throws std::runtime_error if reading the temperature data fails.
     */
    double readThermometer()
    {
        // Set the resolution and offset
        double scale = 0.00294;
        double offset = 36.53; // [°C]

        // Read the values from the sensor
        uint8_t buffer[2];
        try
        {
            m_i2cDevice->readRegister(TEMPERATURE_DATA_REGISTER, buffer, 2);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to read temperature data: " + std::string(e.what()));
        }
        
        // Parse the value
        int16_t temperature = (buffer[1] << 8) | buffer[0];

        // Transform the value into the according unit
        double temperatureValue;
        switch(m_temperatureUnit)
        {
            case TemperatureUnit::IN_CELSIUS:
                temperatureValue = temperature * scale + offset;
                break;
            case TemperatureUnit::IN_KELVIN:
                temperatureValue = temperature * scale + offset + KELVIN_OFFSET;
                break;
            case TemperatureUnit::IN_FARENHEIT:
                temperatureValue = (temperature * scale + offset) * FAHRENHEIT_SCALE + FAHRENHEIT_OFFSET;
        }
        return temperatureValue;
    }

public:
    /**
     * @brief Constructor for the MPU6050 sensor.
     * @details Initializes the MPU6050 sensor with the provided I2C device and validates the connection by reading the WHO_AM_I register.
     * @param t_i2cDevice The I2C device used for communication with the MPU6050 sensor.
     * @throws std::runtime_error if the WHO_AM_I register does not return the expected value or if any I2C communication fails.
     */
    MPU6050::MPU6050(const I2CDevice &t_i2cDevice) : Sensor("MPU6050")
    {
        m_i2cDevice = t_i2cDevice;

        // Validate the I2CDevice by reading the WHO_AM_I register
        uint8_t value;
        try
        {
            m_i2cDevice->readRegister(WHO_AM_I_REGISTER, &value, 1);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to read WHO_AM_I register: " + std::string(e.what()));
        }
        if (value != 0x68)
        {
            // Get the output value and check if the sensor is connected
            std::stringstream error;
            error << "MPU6050 not found. WHO_AM_I register value found: " << std::hex << (int)value;
            throw std::runtime_error(error.str());
        }

        // wake up
        value = 0x00;
        try
        {
            m_i2cDevice->writeRegister(POWER_REGISTER, value);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to wake up MPU6050: " + std::string(e.what()));
        }

        // Configure the sensor with default values
    }

    /**
     * @brief Destructor for the MPU6050 sensor.
     * @details Cleans up any resources used by the sensor.
     */
    ~MPU6050() override = default;

    /**
     * @brief Delete copy constructor and assignment operator to prevent copying.
     * @details The copy constructor is deleted to prevent copying of the MPU6050 instance, as it contains an I2CDevice that should not be copied.
     */
    MPU6050(const MPU6050 &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying.
     * @details This operator is deleted to prevent copying of the MPU6050 instance, as it contains an I2CDevice that should not be copied.
     */
    MPU6050 &operator=(const MPU6050 &) = delete;

    /**
     * @brief Configure the accelerometer with the specified full-scale range.
     * @param t_fs The full-scale range for the accelerometer.
     * @throws std::runtime_error if writing to the accelerometer configuration register fails.
     */
    void configureAccelerometer(const AccelerometerFullScale t_fs)
    {
        uint8_t value = static_cast<uint8_t>(t_fs);
        try
        {
            m_i2cDevice->writeRegister(ACCELEROMETER_CONFIG_REGISTER, value);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to configure accelerometer: " + std::string(e.what()));
        }
        m_accelFullScale = t_fs;
    }

    /**
     * @brief Set the unit for the accelerometer data.
     * @param t_accelerometerUnit The unit to be used for accelerometer data.
     */
    void useAccelerometerUnit(const AccelerometerUnit t_accelerometerUnit)
    {
        m_accelerometerUnit = t_accelerometerUnit;
    }

    /**
     * @brief Configure the gyroscope with the specified full-scale range.
     * @param t_fs The full-scale range for the gyroscope.
     * @throws std::runtime_error if writing to the gyroscope configuration register fails.
     */
    void configureGyroscope(const GyroscopeFullScale t_fs)
    {
        uint8_t value = static_cast<uint8_t>(t_fs);
        try
        {
            m_i2cDevice->writeRegister(GYROSCOPE_CONFIG_REGISTER, value);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to configure gyroscope: " + std::string(e.what()));
        }
        m_gyroFullScale = t_fs;
    }

    /**
     * @brief Set the unit for the gyroscope data.
     * @param t_gyroscopeUnit The unit to be used for gyroscope data.
     */
    void useGyroscopeUnit(const GyroscopeUnit t_gyroscopeUnit)
    {
        m_gyroscopeUnit = t_gyroscopeUnit;
    }

    /**
     * @brief Set the unit for the temperature data.
     * @param t_temperatureUnit The unit to be used for temperature data.
     */
    void useSampleFrequency(const int t_frequency)
    {
        if (t_frequency < 1 || t_frequency > 8000)
        {
            throw std::invalid_argument("Invalid sample rate");
        }
        uint8_t value = static_cast<uint8_t>(std::floor(8000 / t_frequency - 1));
        try
        {
            m_i2cDevice->writeRegister(SAMPLE_RATE_REGISTER, value);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to configure sample rate: " + std::string(e.what()));
        }
        return *this;
    }

    /**
     * @brief Set the unit for the temperature data.
     * @param t_temperatureUnit The unit to be used for temperature data.
     */
    void calibrateGyroscope(const time_t t_duration)
    {
        // Check if the duration is valid
        if (t_duration <= 0)
        {
            throw std::invalid_argument("Duration must be greater than 0");
        }

        // Define the variables for calibration
        Vector3D currentGyroscopeValue;
        Vector3D aggregatedGyroscopeValue;
        int deltaTime = 1'000'000 / m_sampleRateHz; // in microseconds
        int samples = t_duration * m_sampleRateHz;

        // Loop over the samples
        for (uint i = 0; i < samples; i++)
        {
            // Read the values from the sensor in mDeg/s
            currentGyroscopeValue = readGyroscope();

            // Add the values to the sum
            aggregatedGyroscopeValue = currentGyroscopeValue;

            // Sleep for the time of the sample rate
            usleep(deltaTime);
        }

        // Calculate the average values
        m_gyroscopeOffset = (aggregatedGyroscopeValue / static_cast<double>(samples));
    }

    /**
     * @brief Set the unit for the temperature data.
     * @param t_temperatureUnit The unit to be used for temperature data.
     */
    void useTemperatureUnit(const TemperatureUnit t_temperatureUnit)
    {
        m_temperatureUnit = t_temperatureUnit;
    }

    // Sensor implementation

    /**
     * @brief Read data from the MPU6050 sensor and update the sensor data.
     * @details This method reads the accelerometer, gyroscope, and temperature data from the sensor, converts them to the configured units, and updates the sensor data.
     * @throws std::runtime_error if reading any of the sensor data fails.
     */
    void readData() override
    {
        IMUData sensorData;
        sensorData.acceleration = readAccelerometer();
        sensorData.gyroscope = readGyroscope();
        sensorData.temperature = readThermometer();
        setSensorData(sensorData);
        setSensorDataWithTimestamp({sensorData, std::chrono::system_clock::now()});
    }
};

} // namespace control_engineering_uni_a


#elif defined(ARDUINO)

    #error "Support for the Arduino platform is not implemented yet. This library is currently only supported for Raspberry Pi 3/4"

#else

    #error "Unsupported platform. This library is only supported on the Raspberry Pi 3/4 (Arduino is not supported yet)."

#endif // defined(__linux__) && (defined(__arm__) || defined(__aarch64__))

#endif // SENSOR_IMU_MPU6050_HPP