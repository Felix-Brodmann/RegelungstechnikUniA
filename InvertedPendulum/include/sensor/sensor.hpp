#ifndef SENSOR_SENSOR_HPP
#define SENSOR_SENSOR_HPP

#include <string>
#include <memory>

#include "utility/types.hpp"
#include "utility/filter.hpp"
#include "utility/output-stream.hpp"
#include "utility/output-stream/void-output-stream.hpp"

namespace control_engineering_uni_a
{
    
/**
 * @class Sensor
 * @brief Abstract base class representing a generic sensor with optional filtering capability.
 * @details The Sensor class provides an interface for sensors that can store and retrieve sensor data, optionally process the data through a filter, and support timestamped data. Derived classes must implement the readData() method to acquire new sensor data.
 * @note This class is intended to be subclassed for specific sensor implementations.
 * @author Felix Brodmann
 * @date 2025-05-19
 */
class Sensor
{
private:
    std::string m_sensorName; // Name of the sensor
    SensorData m_latestSensorData; // Latest sensor data
    SensorDataWithTimestamp m_latestSensorDataWithTimestamp; // Latest sensor data with timestamp
    std::unique_ptr<Filter> m_filter = nullptr; // Optional filter for processing sensor data
    std::shared_ptr<OutputStream> m_outputStream; // Optional output stream for logging or debugging

    /**
     * @brief Check if there is any filter set for the sensor.
     * @return True if a filter is set, false otherwise.
     */
    bool isThereAnyFilter() const
    {
        return m_filter != nullptr;
    }

protected:
    /**
     * @brief Set the sensor data.
     * @param t_sensorData The sensor data to be set.
     */
    void setSensorData(const SensorData &t_sensorData)
    {
        if (isThereAnyFilter())
        {
            m_filter->update(t_sensorData);
        }
        m_latestSensorData = t_sensorData;
    }

    /**
     * @brief Set the sensor data with a timestamp.
     * @param t_sensorDataWithTimestamp The sensor data with a timestamp.
     */
    void setSensorDataWithTimestamp(const SensorDataWithTimestamp &t_sensorDataWithTimestamp)
    {
        if (isThereAnyFilter())
        {
            m_filter->update(t_sensorDataWithTimestamp.data);
        }
        m_latestSensorDataWithTimestamp = t_sensorDataWithTimestamp;
    }

public:
    /**
     * @brief Constructor for the Sensor class.
     * @param t_sensorName The name of the sensor.
     */
    Sensor(const std::string &t_sensorName)
    {
        m_sensorName = t_sensorName;
        setOutputStream(std::make_shared<VoidOutputStream>()); // Default output stream
    }

    virtual ~Sensor() = default;

    /**
     * @brief Get the name of the sensor.
     * @return The name of the sensor.
     */
    std::string getSensorName() const
    {
        return m_sensorName;
    }

    /**
     * @brief Add a filter to the sensor.
     * @details This method allows the user to set a filter for the sensor. If a filter is already set, it will be replaced with the new one.
     * @param t_filter A unique pointer to the filter to be added.
     */
    void addFilter(std::unique_ptr<Filter> t_filter)
    {
        m_filter = std::move(t_filter);
    }

    /**
     * @brief Get the latest sensor data.
     * @return The latest sensor data.
     */
    SensorData getSensorData() const
    {
        if (isThereAnyFilter())
        {
            return m_filter->getFilteredData();
        }
        return m_latestSensorData;
    }

    /**
     * @brief Get the latest sensor data with a timestamp.
     * @return The latest sensor data with a timestamp.
     */
    SensorDataWithTimestamp getSensorDataWithTimestamp() const
    {
        if (isThereAnyFilter())
        {
            return m_filter->getFilteredDataWithTimestamp();
        }
        return m_latestSensorDataWithTimestamp;
    }

    /**
     * @brief Get the output stream for the sensor.
     * @return A reference to the output stream.
     */
    OutputStream& getOutputStream()
    {
        if (m_outputStream == nullptr)
        {
            throw std::runtime_error("Output stream is not set");
        }
        return *m_outputStream;
    }

    /**
     * @brief Set the output stream for the sensor.
     * @param t_outputStream A shared pointer to the output stream to be set.
     */
    void setOutputStream(std::shared_ptr<OutputStream> t_outputStream)
    {
        if (t_outputStream == nullptr)
        {
            throw std::invalid_argument("Output stream cannot be null");
        }
        m_outputStream = t_outputStream;
    }

    /**
     * @brief Read data from the sensor.
     * @details This method must be implemented by derived classes to acquire new sensor data.
     */
    virtual void readData() = 0;
};

} // namespace control_engineering_uni_a

#endif // SENSOR_SENSOR_HPP