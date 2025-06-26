#ifndef SENSOR_IMU_MPU6050_BUILDER_HPP
#define SENSOR_IMU_MPU6050_BUILDER_HPP

#include "sensor/imu/mpu6050.hpp"

namespace control_engineering_uni_a
{

/**
 * @class MPU6050Builder
 * @brief Builder class for constructing and configuring an MPU6050 IMU sensor.
 * @details This class provides a fluent interface for setting the parameters of an MPU6050 sensor and building it.
 * @author Felix Brodmann
 * @date 2025-05-26
 */
class MPU6050Builder
{
private:
    I2CDevice m_i2cDevice; // I2C device for communication with the sensor
    MPU6050::AccelerometerFullScale m_accelFS = MPU6050::AccelerometerFullScale::FS_2G; // Accelerometer full-scale range
    MPU6050::GyroscopeFullScale m_gyroFS = MPU6050::GyroscopeFullScale::FS_500DPS; // Gyroscope full-scale range
    AccelerometerUnit m_accelUnit = AccelerometerUnit::IN_G; // Accelerometer unit of measurement
    GyroscopeUnit m_gyroUnit = GyroscopeUnit::IN_DEG_PER_S; // Gyroscope unit of measurement
    TemperatureUnit m_tempUnit = TemperatureUnit::IN_CELSIUS; // Temperature unit of measurement
    int m_sampleRateHz = 0; // Sample rate in Hz, 0 means default
    bool m_calibrateOnTheFly = false; // Flag to indicate if gyroscope calibration should be performed on the fly

public:
    /**
     * @brief Default constructor for MPU6050Builder.
     * @details Initializes an empty MPU6050Builder instance.
     */
    static MPU6050Builder create()
    {
        return MPU6050Builder();
    }

    /**
     * @brief Set the I2C device for the MPU6050 sensor.
     * @param t_device The I2C device to be used for communication with the sensor.
     * @return Reference to the MPU6050Builder instance for method chaining.
     */
    MPU6050Builder& withI2CDevice(const I2CDevice& t_device)
    {
        m_i2cDevice = t_device;
        return *this;
    }

    /**
     * @brief Configure the accelerometer full-scale range.
     * @param fs The full-scale range for the accelerometer.
     * @return Reference to the MPU6050Builder instance for method chaining.
     */
    MPU6050Builder& withAccelerometerFullScale(MPU6050::AccelerometerFullScale fs)
    {
        m_accelFS = fs;
        return *this;
    }

    /**
     * @brief Set the unit for the accelerometer data.
     * @param unit The unit to be used for accelerometer data.
     * @return Reference to the MPU6050Builder instance for method chaining.
     */
    MPU6050Builder& withAccelerometerUnit(AccelerometerUnit unit)
    {
        m_accelUnit = unit;
        return *this;
    }

    /**
     * @brief Configure the gyroscope full-scale range.
     * @param fs The full-scale range for the gyroscope.
     * @return Reference to the MPU6050Builder instance for method chaining.
     */
    MPU6050Builder& withGyroscopeFullScale(MPU6050::GyroscopeFullScale fs)
    {
        m_gyroFS = fs;
        return *this;
    }

    /**
     * @brief Set the unit for the gyroscope data.
     * @param unit The unit to be used for gyroscope data.
     * @return Reference to the MPU6050Builder instance for method chaining.
     */
    MPU6050Builder& withGyroscopeUnit(GyroscopeUnit unit)
    {
        m_gyroUnit = unit;
        return *this;
    }

    /**
     * @brief Set the unit for the temperature data.
     * @param unit The unit to be used for temperature data.
     * @return Reference to the MPU6050Builder instance for method chaining.
     */
    MPU6050Builder& withTemperatureUnit(TemperatureUnit unit)
    {
        m_tempUnit = unit;
        return *this;
    }

    /**
     * @brief Set the sample rate for the MPU6050 sensor.
     * @param hz The sample rate in Hz.
     * @return Reference to the MPU6050Builder instance for method chaining.
     */
    MPU6050Builder& withSampleRate(int hz)
    {
        m_sampleRateHz = hz;
        return *this;
    }

    /**
     * @brief Enable on-the-fly calibration of the gyroscope.
     * @details If enabled, the gyroscope will be calibrated automatically when the sensor is built.
     * @note This takes 5 seconds
     * @return Reference to the MPU6050Builder instance for method chaining.
     */
    MPU6050Builder& calibrateOnTheFly()
    {
        m_calibrateOnTheFly = true;
        return *this;
    }

    /**
     * @brief Build the MPU6050 sensor with the configured settings.
     * @details This method constructs an MPU6050 sensor instance with the specified I2C device, accelerometer and gyroscope configurations, sample rate, and temperature unit.
     * @return  unique pointer to the constructed MPU6050 sensor.
     */
    std::unique_ptr<MPU6050> build()
    {
        std::unique_ptr<MPU6050> sensor = std::make_unique<MPU6050>(m_i2cDevice);
        sensor->configureAccelerometer(m_accelFS);
        sensor->useAccelerometerUnit(m_accelUnit);
        sensor->configureGyroscope(m_gyroFS);
        sensor->useGyroscopeUnit(m_gyroUnit);
        sensor->useSampleFrequency(m_sampleRateHz);
        sensor->useTemperatureUnit(m_tempUnit);
        if (m_calibrateOnTheFly)
        {

            // Default calibration for 5 seconds
            sensor->calibrateGyroscope(5);
        }
        return sensor;
    }
};

} // namespace control_engineering_uni_a

#endif // SENSOR_IMU_MPU6050_BUILDER_HPP