#ifndef REGULATOR_PD_REGULATOR_BUILDER_HPP
#define REGULATOR_PD_REGULATOR_BUILDER_HPP

#include <string>
#include <memory>
#include <optional>

#include "regulator/linear/pd-regulator.hpp"

namespace control_engineering_uni_a
{

/**
 * @class PDRegulatorBuilder
 * @brief Builder class for constructing a PDRegulator instance.
 * @details This class provides a fluent interface for setting the parameters of a PDRegulator and building it.
 * @author Felix Brodmann
 * @date 2025-05-26
 */
class PDRegulatorBuilder
{
private:
    std::string m_name; // Name of the regulator
    std::unique_ptr<Sensor> m_sensor; // Sensor used by the regulator
    std::unique_ptr<Actuator> m_actuator; // Actuator used by the regulator
    std::optional<double> m_kp; // Proportional gain (Kp) for the regulator
    std::optional<double> m_kd; // Derivative gain (Kd) for the regulator
    std::optional<SensorData> m_setpoint; // Setpoint value for the regulator
    std::optional<SensorData> m_outputMin; // Minimum output value for the regulator
    std::optional<SensorData> m_outputMax; // Maximum output value for the regulator

public:
    /**
     * @brief Default constructor for PDRegulatorBuilder.
     * @details Initializes an empty PDRegulatorBuilder instance.
     */
    static PDRegulatorBuilder create();

    /**
     * @brief Sets the name of the regulator.
     * @param name The name to be set for the regulator.
     * @return Reference to the PDRegulatorBuilder instance for method chaining.
     */
    PDRegulatorBuilder& setName(const std::string& name)
    {
        m_name = name;
        return *this;
    }

    /**
     * @brief Sets the sensor for the regulator.
     * @param sensor A unique pointer to the sensor to be set.
     * @return Reference to the PDRegulatorBuilder instance for method chaining.
     */
    PDRegulatorBuilder& setSensor(std::unique_ptr<Sensor> sensor)
    {
        m_sensor = std::move(sensor);
        return *this;
    }

    /**
     * @brief Sets the actuator for the regulator.
     * @param actuator A unique pointer to the actuator to be set.
     * @return Reference to the PDRegulatorBuilder instance for method chaining.
     */
    PDRegulatorBuilder& setActuator(std::unique_ptr<Actuator> actuator)
    {
        m_actuator = std::move(actuator);
        return *this;
    }

    /**
     * @brief Sets the proportional gain (Kp) for the regulator.
     * @param kp The proportional gain to be set.
     * @return Reference to the PDRegulatorBuilder instance for method chaining.
     */
    PDRegulatorBuilder& setKp(double kp)
    {
        m_kp = kp;
        return *this;
    }

    /**
     * @brief Sets the derivative gain (Kd) for the regulator.
     * @param kd The derivative gain to be set.
     * @return Reference to the PDRegulatorBuilder instance for method chaining.
     */
    PDRegulatorBuilder& setKd(double kd)
    {
        m_kd = kd;
        return *this;
    }

    /**
     * @brief Sets the setpoint for the regulator.
     * @param setpoint The setpoint value to be set.
     * @return Reference to the PDRegulatorBuilder instance for method chaining.
     */
    PDRegulatorBuilder& setSetpoint(SensorData setpoint)
    {
        m_setpoint = setpoint;
        return *this;
    }

    /**
     * @brief Sets the minimum output value for the regulator.
     * @param outputMin The minimum output value to be set.
     * @return Reference to the PDRegulatorBuilder instance for method chaining.
     */
    PDRegulatorBuilder& setOutputMin(const SensorData outputMin)
    {
        m_outputMin = outputMin;
        return *this;
    }

    /**
     * @brief Sets the maximum output value for the regulator.
     * @param outputMax The maximum output value to be set.
     * @return Reference to the PDRegulatorBuilder instance for method chaining.
     */
    PDRegulatorBuilder& setOutputMax(const SensorData outputMax)
    {
        m_outputMax = outputMax;
        return *this;
    }

    /**
     * @brief Builds the PDRegulator instance.
     * @details This method checks if all required parameters are set and then creates a PDRegulator instance.
     * @return A unique pointer to the created PDRegulator instance.
     * @throws std::runtime_error If any required parameter is not set.
     */
    std::unique_ptr<PDRegulator> build()
    {
        if (m_name.empty()) {
            throw std::runtime_error("Regulator name not set");
        }
        if (!m_sensor) {
            throw std::runtime_error("Sensor not set");
        }
        if (!m_actuator) {
            throw std::runtime_error("Actuator not set");
        }
        if (!m_kp.has_value()) {
            throw std::runtime_error("Kp not set");
        }
        if (!m_kd.has_value()) {
            throw std::runtime_error("Kd not set");
        }
        if (!m_setpoint.has_value()) {
            throw std::runtime_error("Setpoint not set");
        }
        if (!m_outputMin.has_value()) {
            throw std::runtime_error("Output minimum not set");
        }
        if (!m_outputMax.has_value()) {
            throw std::runtime_error("Output maximum not set");
        }

        std::string nameCopy = m_name;

        return std::make_unique<PDRegulator>(
            nameCopy,
            std::move(m_sensor),
            std::move(m_actuator),
            m_kp.value(),
            m_kd.value(),
            m_setpoint.value(),
            m_outputMin.value(),
            m_outputMax.value()
        );
    }
};

} // namespace control_engineering_uni_a

#endif // REGULATOR_PD_REGULATOR_BUILDER_HPP