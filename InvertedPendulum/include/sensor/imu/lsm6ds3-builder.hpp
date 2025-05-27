#ifndef SENSOR_IMU_LSM6DS3_BUILDER_HPP
#define SENSOR_IMU_LSM6DS3_BUILDER_HPP

#include <string>
#include <memory>

#include "sensor/imu/lsm6ds3.hpp"

namespace control_engineering_uni_a
{

/**
 * @class LSM6DS3Builder
 * @brief Builder class for constructing and configuring an LSM6DS3 IMU sensor.
 * @details This class provides a fluent interface for setting the parameters of a LSM6DS3 sensor and building it.
 * @author Felix Brodmann
 * @date 2025-05-26
 */
class LSM6DS3Builder
{
private:
    std::unique_ptr<I2CDevice> m_i2cDevice; // I2C device for communication with the sensor
    LSM6DS3::AccelerometerFullScale m_accelFs = LSM6DS3::AccelerometerFullScale::FS_2G; // Accelerometer full-scale range
    AccelerometerUnit m_accelUnit = AccelerometerUnit::IN_M_PER_S2; // Accelerometer unit of measurement
    LSM6DS3::AccelerometerOutputDataRate m_accelOdr = LSM6DS3::AccelerometerOutputDataRate::HIGH_PERFORMANCE_1660Hz; // Accelerometer output data rate
    LSM6DS3::AccelerometerFilterBandwidth m_accelBw = LSM6DS3::AccelerometerFilterBandwidth::BW_200Hz; // Accelerometer filter bandwidth
    LSM6DS3::GyroscopeFullScale m_gyroFs = LSM6DS3::GyroscopeFullScale::FS_500DPS; // Gyroscope full-scale range
    GyroscopeUnit m_gyroUnit = GyroscopeUnit::IN_DEG_PER_S; // Gyroscope unit of measurement
    LSM6DS3::GyroscopeOutputDataRate m_gyroOdr = LSM6DS3::GyroscopeOutputDataRate::HIGH_PERFORMANCE_1660Hz; // Gyroscope output data rate
    TemperatureUnit m_tempUnit = TemperatureUnit::IN_CELSIUS; // Temperature unit of measurement
    bool m_calibrateOnTheFly = false; // Flag to indicate if gyroscope calibration should be performed on the fly

public:
    /**
     * @brief Default constructor for LSM6DS3Builder.
     * @details Initializes an empty LSM6DS3Builder instance.
     */
    static LSM6DS3Builder create()
    {
        return LSM6DS3Builder();
    }

    /**
     * @brief Set the I2C device for the LSM6DS3 sensor.
     * @param t_i2cDevice The I2C device to be used for communication with the sensor.
     * @return Reference to the LSM6DS3Builder instance for method chaining.
     * @throws std::runtime_error if the provided I2C device is null.
     */
    LSM6DS3Builder &withI2CDevice(std::unique_ptr<I2CDevice> t_i2cDevice)
    {
        m_i2cDevice = std::move(t_i2cDevice);
        if (!m_i2cDevice)
        {
            throw std::runtime_error("I2C device must not be null");
        }
        return *this;
    }

    /**
     * @brief Configure the accelerometer settings.
     * @param t_odr The output data rate for the accelerometer.
     * @param t_fs The full-scale range for the accelerometer.
     * @param t_bw The filter bandwidth for the accelerometer.
     * @return Reference to the LSM6DS3Builder instance for method chaining.
     */
    LSM6DS3Builder &withAccelerometerConfig(LSM6DS3::AccelerometerOutputDataRate t_odr, LSM6DS3::AccelerometerFullScale t_fs, LSM6DS3::AccelerometerFilterBandwidth t_bw)
    {
        m_accelOdr = t_odr;
        m_accelFs = t_fs;
        m_accelBw = t_bw;
        return *this;
    }

    /**
     * @brief Set the unit for the accelerometer data.
     * @param t_unit The unit to be used for accelerometer data.
     * @return Reference to the LSM6DS3Builder instance for method chaining.
     */
    LSM6DS3Builder &withAccelerometerUnit(AccelerometerUnit t_unit)
    {
        m_accelUnit = t_unit;
        return *this;
    }

    /**
     * @brief Configure the gyroscope settings.
     * @param t_odr The output data rate for the gyroscope.
     * @param t_fs The full-scale range for the gyroscope.
     * @return Reference to the LSM6DS3Builder instance for method chaining.
     */
    LSM6DS3Builder &withGyroscopeConfig(LSM6DS3::GyroscopeOutputDataRate t_odr, LSM6DS3::GyroscopeFullScale t_fs)
    {
        m_gyroOdr = t_odr;
        m_gyroFs = t_fs;
        return *this;
    }

    /**
     * @brief Set the unit for the gyroscope data.
     * @param t_unit The unit to be used for gyroscope data.
     * @return Reference to the LSM6DS3Builder instance for method chaining.
     */
    LSM6DS3Builder &withGyroscopeUnit(GyroscopeUnit t_unit)
    {
        m_gyroUnit = t_unit;
        return *this;
    }

    /**
     * @brief Set the unit for the temperature data.
     * @param t_unit The unit to be used for temperature data.
     * @return Reference to the LSM6DS3Builder instance for method chaining.
     */
    LSM6DS3Builder &withTemperatureUnit(TemperatureUnit t_unit)
    {
        m_tempUnit = t_unit;
        return *this;
    }

    /**
     * @brief Enable gyroscope calibration on the fly.
     * @details If enabled, the gyroscope will be calibrated automatically during operation.
     * @return Reference to the LSM6DS3Builder instance for method chaining.
     */
    LSM6DS3Builder &calibrateOnTheFly()
    {
        m_calibrateOnTheFly = true;
        return *this;
    }

    /**
     * @brief Build and configure the LSM6DS3 sensor instance.
     * @details This method constructs a new LSM6DS3 sensor with the specified configurations and returns a unique pointer to it.
     * @return A unique pointer to the constructed LSM6DS3 sensor.
     */
    std::unique_ptr<LSM6DS3> build()
    {
        std::unique_ptr<LSM6DS3> sensor = std::make_unique<LSM6DS3>(std::move(m_i2cDevice));
        sensor->configureAccelerometer(m_accelOdr, m_accelFs, m_accelBw);
        sensor->useAccelerometerUnit(m_accelUnit);
        sensor->configureGyroscope(m_gyroOdr, m_gyroFs);
        sensor->useGyroscopeUnit(m_gyroUnit);
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

#endif // SENSOR_IMU_LSM6DS3_BUILDER_HPP