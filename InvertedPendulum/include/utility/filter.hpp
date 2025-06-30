#ifndef UTILITY_FILTER_HPP
#define UTILITY_FILTER_HPP

#include <string>
#include <chrono>

#include "utility/types.hpp"
#include "utility/output-stream.hpp"
#include "utility/output-stream/void-output-stream.hpp"

namespace control_engineering_uni_a
{
/**
 * @class Filter
 * @brief Abstract base class for implementing sensor data filters.
 * @details The Filter class provides an interface and common functionality for filters that process sensor data. It stores the latest filtered data and its timestamp, and requires derived classes to implement the update method for processing new sensor data.
 * @note This class is intended to be subclassed for specific filter implementations.
 * @author Felix Brodmann
 * @date 2025-05-19
 */
class Filter
{
private:
    std::string m_filterName;
    SensorData m_latestFilteredData;
    SensorDataWithTimestamp m_latestFilteredDataWithTimestamp;
    std::shared_ptr<OutputStream> m_outputStream; // Optional output stream for logging or debugging

protected:
    /**
     * @brief Set the filtered data.
     * @param t_filteredData The filtered data to be set.
     */
    void setFilteredData(const SensorData &t_filteredData)
    {
        m_latestFilteredData = t_filteredData;
    }

    /**
     * @brief Set the filtered data with a timestamp.
     * @param t_filteredDataWithTimestamp The filtered data with a timestamp.
     */
    void setFilteredDataWithTimestamp(const SensorDataWithTimestamp &t_filteredDataWithTimestamp)
    {
        m_latestFilteredDataWithTimestamp = t_filteredDataWithTimestamp;
    }

public:
    /**
     * @brief Constructor for the Filter class.
     * @param t_filterName The name of the filter.
     */
    Filter(const std::string &t_filterName)
    {
        m_filterName = t_filterName;
        m_outputStream = std::make_shared<VoidOutputStream>(); // Default output stream
    }

    virtual ~Filter() = default;

    /**
     * @brief Get the name of the filter.
     * @return The name of the filter.
     */
    std::string getFilterName() const
    {
        return m_filterName;
    }

    /**
     * @brief Get the latest filtered data.
     * @return The latest filtered data.
     */
    SensorData getFilteredData() const
    {
        return m_latestFilteredData;
    }

    /**
     * @brief Get the latest filtered data with a timestamp.
     * @return The latest filtered data with a timestamp.
     */
    SensorDataWithTimestamp getFilteredDataWithTimestamp() const
    {
        return m_latestFilteredDataWithTimestamp;
    }

    /**
     * @brief Get the output stream for logging or debugging.
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
     * @brief Set the output stream for logging or debugging.
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
     * @brief Update the filter with new sensor data.
     * @param data The new sensor data to be processed by the filter.
     * @details This method must be implemented by derived classes to define how the filter processes new data.
     */
    virtual void update(const SensorData& data) = 0;
};

} // namespace control_engineering_uni_a

#endif // UTILITY_FILTER_HPP