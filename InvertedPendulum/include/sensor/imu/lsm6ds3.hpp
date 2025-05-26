#ifndef SENSOR_IMU_LSM6DS3_HPP
#define SENSOR_IMU_LSM6DS3_HPP

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
 * @class LSM6DS3
 * @brief Driver class for the LSM6DS3 3D accelerometer and 3D gyroscope sensor.
 * @details This class provides an interface to configure and read data from the LSM6DS3 IMU sensor via I2C. It supports configuration of accelerometer and gyroscope output data rates, full-scale ranges, and filter bandwidths. The class also allows reading acceleration, gyroscope, and temperature data in various units, and provides gyroscope calibration functionality.
 * @note The LSM6DS3 sensor is a 3D accelerometer and 3D gyroscope sensor with a digital output interface. It is commonly used in applications such as motion tracking, gesture recognition, and orientation detection.
 * @author Felix Brodmann
 * @date 2025-05-19
 */
class LSM6DS3 : public Sensor
{
private:
    constexpr uint8_t WHO_AM_I_REGISTER = 0x0F;
    constexpr uint8_t ACCELEROMETER_CONFIG_REGISTER = 0x10;
    constexpr uint8_t GYROSCOPE_CONFIG_REGISTER = 0x11;
    constexpr uint8_t TEMPERATURE_DATA_REGISTER = 0x20;
    constexpr uint8_t GYROSCOPE_DATA_REGISTER = 0x22;
    constexpr uint8_t ACCELEROMETER_DATA_REGISTER = 0x28;

    /**
     * @brief Enumeration for accelerometer output data rates.
     * @details This enumeration defines the available output data rates for the accelerometer. The values correspond to the settings in the LSM6DS3 sensor.
     */
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

    /**
     * @brief Enumeration for accelerometer full-scale ranges.
     * @details This enumeration defines the available full-scale ranges for the accelerometer. The values correspond to the settings in the LSM6DS3 sensor.
     */
    enum class AccelerometerFullScale
    {
        FS_2G = 0x00,
        FS_4G = 0x08,
        FS_8G = 0x0C,
        FS_16G = 0x04
    };

    /**
     * @brief Enumeration for accelerometer filter bandwidths.
     * @details This enumeration defines the available filter bandwidths for the accelerometer. The values correspond to the settings in the LSM6DS3 sensor.
     */
    enum class AccelerometerFilterBandwidth
    {
        BW_50Hz = 0x03,
        BW_100Hz = 0x02,
        BW_200Hz = 0x01,
        BW_400Hz = 0x00,
    };

    /**
     * @brief Enumeration for accelerometer units.
     * @details This enumeration defines the available units for accelerometer data. The values correspond to the settings in the LSM6DS3 sensor.
     */
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

    /**
     * @brief Enumeration for gyroscope units.
     * @details This enumeration defines the available units for gyroscope data. The values correspond to the settings in the LSM6DS3 sensor.
     */
    enum class GyroscopeFullScale
    {
        FS_125DPS = 0x02,
        FS_250DPS = 0x00,
        FS_500DPS = 0x04,
        FS_1000DPS = 0x08,
        FS_2000DPS = 0x0C
    };
    I2CDevice m_i2cDevice;
    AccelerometerFullScale m_accelFullScale = AccelerometerFullScale::FS_2G;
    AccelerometerUnit m_accelerometerUnit = AccelerometerUnit::IN_M_PER_S2;
    GyroscopeFullScale m_gyroFullScale = GyroscopeFullScale::FS_500DPS;
    GyroscopeUnit m_gyroscopeUnit = GyroscopeUnit::IN_DEG_PER_S;
    GyroscopeOutputDataRate m_gyroOutputDataRate = GyroscopeOutputDataRate::HIGH_PERFORMANCE_1660Hz;
    TemperatureUnit m_temperatureUnit = TemperatureUnit::IN_CELSIUS;
    Vector3D m_gyroscopeOffset;

    /**
     * @brief Read the accelerometer data from the sensor.
     * @details This method reads the accelerometer data from the sensor and converts it to the specified unit. It also checks if the full-scale range is set before reading the data.
     * @return The accelerometer data in the specified unit.
     * @throws `std::runtime_error` if the accelerometer range is not set or if there is an error reading the data from the sensor.
     */
    Vector3D readAccelerometer()
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
                throw std::runtime_error("Accelerometer range not set. Call configureAccelerometer(3) first");
        }

        // Read the values from the sensor
        uint8_t buffer[6];
        try
        {
            m_i2cDevice.readRegister(ACCELEROMETER_DATA_REGISTER, buffer, 6);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to read accelerometer data: " + std::string(e.what()));
        }

        // Parse the values from each Axis into one Axis value
        int16_t x = (buffer[1] << 8) | buffer[0];
        int16_t y = (buffer[3] << 8) | buffer[2];
        int16_t z = (buffer[5] << 8) | buffer[4];

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
     * @brief Read the gyroscope data from the sensor.
     * @details This method reads the gyroscope data from the sensor and converts it to the specified unit. It also checks if the full-scale range is set before reading the data.
     * @return The gyroscope data in the specified unit.
     * @throws `std::runtime_error` if the gyroscope range is not set or if there is an error reading the data from the sensor.
     */
    Vector3D readGyroscope()
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
                throw std::runtime_error("Gyroscope range not set. Call configureGyroscope(2) first");
        }

        // Read the values from the sensor
        uint8_t buffer[6];
        try
        {
            m_i2cDevice.readRegister(GYROSCOPE_DATA_REGISTER, buffer, 6);
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
     * @brief Read the temperature data from the sensor.
     * @details This method reads the temperature data from the sensor and converts it to the specified unit.
     * @return The temperature data in the specified unit.
     * @throws `std::runtime_error` if there is an error reading the data from the sensor.
     */
    double readThermometer()
    {
        // Set the resolution and offset
        double scale = 0.0625;
        double offset = 25.0; // [°C]

        // Read the values from the sensor
        uint8_t buffer[2];
        try
        {
            m_i2cDevice.readRegister(TEMPERATURE_DATA_REGISTER, buffer, 2);
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
                break;
        }
        return temperatureValue;
    }

public:
    /**
     * @brief Constructor for the LSM6DS3 class.
     * @details This constructor initializes the LSM6DS3 sensor with the specified I2C device. It also validates the connection by reading the WHO_AM_I register.
     * @param t_i2cDevice The I2C device used to communicate with the sensor.
     * @throws `std::runtime_error` if the WHO_AM_I register value does not match the expected value or if there is an error reading the register.
     */
    LSM6DS3::LSM6DS3(const I2CDevice &t_i2cDevice) : Sensor("LSM6DS3")
    {
        m_i2cDevice = t_i2cDevice;

        // Validate the I2CDevice by reading the WHO_AM_I register
        uint8_t value;
        try
        {
            m_i2cDevice.readRegister(WHO_AM_I_REGISTER, &value, 1);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to read WHO_AM_I register: " + std::string(e.what()));
        }
        if (value != 0x69)
        {
            // Get the output value and check if the sensor is connected
            std::stringstream error;
            error << "LSM6DS3 not found. WHO_AM_I register value found: " << std::hex << (int)value;
            throw std::runtime_error(error.str());
        }

        // Configure the sensor with default values
        configureAccelerometer(AccelerometerOutputDataRate::HIGH_PERFORMANCE_1660Hz, AccelerometerFullScale::FS_2G, AccelerometerFilterBandwidth::BW_200Hz);
        configureGyroscope(GyroscopeOutputDataRate::HIGH_PERFORMANCE_1660Hz, GyroscopeFullScale::FS_500DPS);
        useAccelerometerUnit(AccelerometerUnit::IN_M_PER_S2);
        useGyroscopeUnit(GyroscopeUnit::IN_DEG_PER_S);
        useTemperatureUnit(TemperatureUnit::IN_CELSIUS);
    }

    /**
     * @brief Destructor for the LSM6DS3 class.
     * @details This destructor does not perform any specific cleanup as the I2CDevice is managed externally.
     */
    ~LSM6DS3() override = default;

    /**
     * @brief Delete copy constructor and assignment operator to prevent copying.
     * @details The copy constructor is deleted to prevent copying of the LSM6DS3 instance, as it contains an I2CDevice that should not be copied.
     */
    LSM6DS3(const LSM6DS3 &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying.
     * @details This operator is deleted to prevent copying of the LSM6DS3 instance, as it contains an I2CDevice that should not be copied.
     */
    LSM6DS3 &operator=(const LSM6DS3 &) = delete;

    /**
     * @brief Configure the accelerometer settings.
     * @details This method configures the accelerometer settings, including output data rate, full-scale range, and filter bandwidth. It also updates the internal state of the sensor.
     * @param t_odr The output data rate for the accelerometer.
     * @param t_fs The full-scale range for the accelerometer.
     * @param t_bw The filter bandwidth for the accelerometer.
     * @throws `std::runtime_error` if there is an error writing the configuration to the sensor.
     */
    void configureAccelerometer(const AccelerometerOutputDataRate t_odr, const AccelerometerFullScale t_fs, const AccelerometerFilterBandwidth t_bw)
    {
        uint8_t value = static_cast<uint8_t>(t_odr) | static_cast<uint8_t>(t_fs) | static_cast<uint8_t>(t_bw);
        try
        {
            m_i2cDevice.writeRegister(ACCELEROMETER_CONFIG_REGISTER, value);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to configure accelerometer: " + std::string(e.what()));
        }
        m_accelFullScale = t_fs;
    }

    /**
     * @brief Set the accelerometer unit.
     * @details This method sets the unit for the accelerometer data. It updates the internal state of the sensor.
     * @param t_accelerometerUnit The unit to be used for accelerometer data.
     */
    void useAccelerometerUnit(const AccelerometerUnit t_accelerometerUnit)
    {
        m_accelerometerUnit = t_accelerometerUnit;
    }

    /**
     * @brief Configure the gyroscope settings.
     * @details This method configures the gyroscope settings, including output data rate and full-scale range. It also updates the internal state of the sensor.
     * @param t_odr The output data rate for the gyroscope.
     * @param t_fs The full-scale range for the gyroscope.
     * @throws `std::runtime_error` if there is an error writing the configuration to the sensor.
     */
    void configureGyroscope(const GyroscopeOutputDataRate t_odr, const GyroscopeFullScale t_fs)
    {
        uint8_t value = static_cast<uint8_t>(t_odr) | static_cast<uint8_t>(t_fs);
        try
        {
            m_i2cDevice.writeRegister(GYROSCOPE_CONFIG_REGISTER, value);
        }
        catch (const std::runtime_error &e)
        {
            throw std::runtime_error("Failed to configure gyroscope: " + std::string(e.what()));
        }
        m_gyroOutputDataRate = t_odr;
        m_gyroFullScale = t_fs;
    }

    /**
     * @brief Set the gyroscope unit.
     * @details This method sets the unit for the gyroscope data. It updates the internal state of the sensor.
     * @param t_gyroscopeUnit The unit to be used for gyroscope data.
     */
    void useGyroscopeUnit(const GyroscopeUnit t_gyroscopeUnit)
    {
        m_gyroscopeUnit = t_gyroscopeUnit;
    }

    /**
     * @brief Calibrate the gyroscope.
     * @details This method calibrates the gyroscope by averaging the readings over a specified duration. It updates the internal offset value for the gyroscope.
     * @param t_duration The duration for calibration in seconds.
     * @throws `std::invalid_argument` if the duration is less than or equal to 0.
     * @throws `std::runtime_error` if there is an error reading the gyroscope data.    
     */
    void calibrateGyroscope(const time_t t_duration)
    {
        int frequency = 0; // Frequency in Hz
        switch (m_gyroOutputDataRate)
        {
            case GyroscopeOutputDataRate::POWER_DOWN:
                frequency = 0;
                break;
            case GyroscopeOutputDataRate::LOW_POWER_12_5Hz:
                frequency = 12;
                break;
            case GyroscopeOutputDataRate::LOW_POWER_26Hz:
                frequency = 26;
                break;
            case GyroscopeOutputDataRate::LOW_POWER_52Hz:
                frequency = 52;
                break;
            case GyroscopeOutputDataRate::NORMAL_MODE_104Hz:
                frequency = 104;
                break;
            case GyroscopeOutputDataRate::NORMAL_MODE_208Hz:
                frequency = 208;
                break;
            case GyroscopeOutputDataRate::HIGH_PERFORMANCE_416Hz:
                frequency = 416;
                break;
            case GyroscopeOutputDataRate::HIGH_PERFORMANCE_833Hz:
                frequency = 833;
                break;
            case GyroscopeOutputDataRate::HIGH_PERFORMANCE_1660Hz:
                frequency = 1660;
                break;
        }

        // Define the variables for calibration
        Vector3D currentGyroscopeValue;
        Vector3D aggregatedGyroscopeValue;
        int deltaTime = 1'000'000 / frequency; // in microseconds
        int samples = t_duration * frequency;

        // Loop over the samples
        for (uint i = 0; i < samples; i++)
        {
            // Read the values from the sensor in mDeg/s
            currentGyroscopeValue = readGyroscope();

            // Add the values to the sum
            aggregatedGyroscopeValue += currentGyroscopeValue;

            // Sleep for the time of the sample rate
            usleep(deltaTime);
        }

        // Calculate the average values
        m_gyroscopeOffset = (aggregatedGyroscopeValue / static_cast<double>(samples));
    }

    /**
     * @brief Set the temperature unit.
     * @details This method sets the unit for the temperature data. It updates the internal state of the sensor.
     * @param t_temperatureUnit The unit to be used for temperature data.
     */
    void useTemperatureUnit(const TemperatureUnit t_temperatureUnit)
    {
        m_temperatureUnit = t_temperatureUnit;
    }

    // Sensor implementation

    /**
     * @brief Read data from the sensor.
     * @details This method reads the accelerometer, gyroscope, and temperature data from the sensor. It updates the internal state of the sensor with the latest readings.
     * @throws `std::runtime_error` if there is an error reading the data from the sensor.
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

#endif // SENSOR_IMU_LSM6DS3_HPP