#ifndef REGULATOR_REGULATOR_HPP
#define REGULATOR_REGULATOR_HPP

#include <string>
#include <stdexcept>
#include <memory>
#include <atomic>

#include "utility/types.hpp"
#include "utility/filter.hpp"
#include "sensor/sensor.hpp"
#include "actuator/actuator.hpp"

namespace control_engineering_uni_a
{
/**
 * @class Regulator
 * @brief Abstract base class for implementing control regulators.
 * @details The Regulator class provides a common interface and basic functionality for control regulators, managing a sensor and actuator, sampling rate, and regulation state. Derived classes must implement the startRegulation() and stopRegulation() methods.
 * @note This class is intended to be subclassed for specific regulator implementations.
 * @author Felix Brodmann
 * @date 2025-05-19
 */
class Regulator
{
private:
    std::string m_regulatorName;
    std::unique_ptr<Sensor> m_sensor;
    std::unique_ptr<Actuator> m_actuator;
    int m_samplingRate = 100;
    std::atomic_bool m_isRegulating = false;

protected:
    /**
     * @brief Set the sensor for the regulator.
     * @param t_sensor A unique pointer to the sensor to be set.
     */
    void setIsRegulating(bool t_isRegulating)
    {
        m_isRegulating = t_isRegulating;
    }

    /**
     * @brief Check if the regulator is currently regulating.
     * @return True if the regulator is regulating, false otherwise.
     */
    bool isRegulating() const
    {
        return m_isRegulating;
    }

public:
    /**
     * @brief Constructor for the Regulator class.
     * @param t_regulatorName The name of the regulator.
     * @param t_sensor A unique pointer to the sensor to be used by the regulator.
     * @param t_actuator A unique pointer to the actuator to be used by the regulator.
     */
    Regulator(const std::string& t_regulatorName, std::unique_ptr<Sensor> t_sensor, std::unique_ptr<Actuator> t_actuator)
    {
        m_regulatorName = t_regulatorName;
        m_sensor = std::move(t_sensor);
        m_actuator = std::move(t_actuator);
    }

    ~Regulator() = default;

    /**
     * @brief Get the name of the regulator.
     * @return The name of the regulator.
     */
    const std::string getRegulatorName() const { return m_regulatorName; }

    /**
     * @brief Get the sensor and actuator for the regulator.
     * @return A reference to the sensor and actuator.
     */
    Sensor& getSensor()
    {
        if (m_sensor == nullptr)
        {
            throw std::runtime_error("Sensor is not set");
        }
        return *m_sensor;
    }

    /**
     * @brief Get the actuator for the regulator.
     * @return A reference to the actuator.
     */
    Actuator& getActuator()
    {
        if (m_actuator == nullptr)
        {
            throw std::runtime_error("Actuator is not set");
        }
        return *m_actuator;
    }

    /**
     * @brief Set the sampling rate for the regulator.
     * @param t_samplingRate The sampling rate in milliseconds.
     * @throws std::invalid_argument if the sampling rate is not positive.
     */
    void setSamplingRate(int t_samplingRate)
    {
        if (t_samplingRate <= 0)
        {
            throw std::invalid_argument("Sampling rate must be positive");
        }
        m_samplingRate = t_samplingRate;
    }

    /**
     * @brief Get the sampling rate for the regulator.
     * @return The sampling rate in milliseconds.
     */
    const int getSamplingRate() const { return m_samplingRate; }

    /**
     * @brief Get the sensor for the regulator.
     * @return A reference to the sensor.
     */
    virtual void startRegulation() = 0;

    /**
     * @brief Stop the regulation process.
     * @details This method must be implemented by derived classes to stop the regulation process.
     */
    virtual void stopRegulation() = 0;
    
};

} // namespace control_engineering_uni_a

#endif // REGULATOR_REGULATOR_HPP