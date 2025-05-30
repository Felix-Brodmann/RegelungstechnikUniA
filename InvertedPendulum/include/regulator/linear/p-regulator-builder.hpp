#ifndef REGULATOR_LINEAR_P_REGULATOR_BUILDER_HPP
#define REGULATOR_LINEAR_P_REGULATOR_BUILDER_HPP

#include <string>
#include <memory>
#include <optional>

#include "regulator/linear/p-regulator.hpp"

namespace control_engineering_uni_a
{
    
/**
 * @class PRegulatorBuilder
 * @brief Builder class for constructing a PRegulator instance.
 * @details This class provides a fluent interface for setting the parameters of a PRegulator and building it.
 * @author Felix Brodmann
 * @date 2025-05-26
 */
class PRegulatorBuilder
{
private:
    std::string m_name; // Name of the regulator
    std::unique_ptr<Sensor> m_sensor; // Sensor used by the regulator
    std::unique_ptr<Actuator> m_actuator; // Actuator used by the regulator
    std::optional<double> m_kp; // Proportional gain (Kp) for the regulator
    std::optional<SensorData> m_setpoint; // Setpoint value for the regulator

public:
    /**
     * @brief Default constructor for PRegulatorBuilder.
     * @details Initializes an empty PRegulatorBuilder instance.
     */
    static PRegulatorBuilder create()
    {
        return PRegulatorBuilder();
    }

    /**
     * @brief Sets the name of the regulator.
     * @param name The name to be set for the regulator.
     * @return Reference to the PRegulatorBuilder instance for method chaining.
     */
    PRegulatorBuilder& setName(const std::string& name)
    {
        m_name = name;
        return *this;
    }

    /**
     * @brief Sets the sensor for the regulator.
     * @param sensor A unique pointer to the sensor to be set.
     * @return Reference to the PRegulatorBuilder instance for method chaining.
     */
    PRegulatorBuilder& setSensor(std::unique_ptr<Sensor> sensor)
    {
        m_sensor = std::move(sensor);
        return *this;
    }

    /**
     * @brief Sets the actuator for the regulator.
     * @param actuator A unique pointer to the actuator to be set.
     * @return Reference to the PRegulatorBuilder instance for method chaining.
     */
    PRegulatorBuilder& setActuator(std::unique_ptr<Actuator> actuator)
    {
        m_actuator = std::move(actuator);
        return *this;
    }

    /**
     * @brief Sets the proportional gain (Kp) for the regulator.
     * @param kp The proportional gain to be set.
     * @return Reference to the PRegulatorBuilder instance for method chaining.
     */
    PRegulatorBuilder& setKp(double kp)
    {
        m_kp = kp;
        return *this;
    }

    /**
     * @brief Sets the setpoint for the regulator.
     * @param setpoint The setpoint value to be set.
     * @return Reference to the PRegulatorBuilder instance for method chaining.
     */
    PRegulatorBuilder& setSetpoint(const SensorData setpoint)
    {
        m_setpoint = setpoint;
        return *this;
    }

    /**
     * @brief Builds the PRegulator instance.
     * @details This method checks if all required parameters are set and constructs a PRegulator instance.
     * @return A unique pointer to the constructed PRegulator instance.
     * @throws std::runtime_error if any required parameter is not set.
     */
    std::unique_ptr<PRegulator> build()
    {
        if (m_name.empty()) {
            throw std::runtime_error("Regulator name must be set.");
        }
        if (!m_sensor) {
            throw std::runtime_error("Sensor must be set.");
        }
        if (!m_actuator) {
            throw std::runtime_error("Actuator must be set.");
        }
        if (!m_kp.has_value()) {
            throw std::runtime_error("Proportional gain (Kp) must be set.");
        }
        if (!m_setpoint.has_value()) {
            throw std::runtime_error("Setpoint must be set.");
        }

        std::string nameCopy = m_name;

        return std::make_unique<PRegulator>(
                nameCopy,
                std::move(m_sensor),
                std::move(m_actuator),
                m_kp.value(),
                m_setpoint.value()
            );
    }
};

} // namespace control_engineering_uni_a

#endif // REGULATOR_LINEAR_P_REGULATOR_BUILDER_HPP