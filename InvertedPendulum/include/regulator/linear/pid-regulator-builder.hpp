#ifndef REGULATOR_PID_REGULATOR_BUILDER_HPP
#define REGULATOR_PID_REGULATOR_BUILDER_HPP

#include <string>
#include <memory>
#include <optional>

#include "regulator/linear/pid-regulator.hpp"

namespace control_engineering_uni_a
{

/**
 * @class PIDRegulatorBuilder
 * @brief Builder class for constructing a PIDRegulator instance.
 * @details This class provides a fluent interface for setting the parameters of a PIDRegulator and building it.
 * @author Felix Brodmann
 * @date 2025-05-26
 */
class PIDRegulatorBuilder
{
private:
    std::string m_name; // Name of the regulator
    std::unique_ptr<Sensor> m_sensor; // Sensor used by the regulator
    std::unique_ptr<Actuator> m_actuator; // Actuator used by the regulator
    std::optional<double> m_kp; // Proportional gain (Kp) for the regulator
    std::optional<double> m_ki; // Integral gain (Ki) for the regulator
    std::optional<double> m_kd; // Derivative gain (Kd) for the regulator
    std::optional<SensorData> m_setpoint; // Setpoint value for the regulator
    std::optional<SensorData> m_integralMin; // Minimum integral value (for anti-windup)
    std::optional<SensorData> m_integralMax; // Maximum integral value (for anti-windup)

public:
    /**
     * @brief Default constructor for PIDRegulatorBuilder.
     * @details Initializes an empty PIDRegulatorBuilder instance.
     */
    static PIDRegulatorBuilder create()
    {
        return PIDRegulatorBuilder();
    }

    /**
     * @brief Sets the name of the regulator.
     * @param name The name to be set for the regulator.
     * @return Reference to the PIDRegulatorBuilder instance for method chaining.
     */
    PIDRegulatorBuilder& setName(const std::string& name)
    {
        m_name = name;
        return *this;
    }

    /**
     * @brief Sets the sensor for the regulator.
     * @param sensor A unique pointer to the sensor to be set.
     * @return Reference to the PIDRegulatorBuilder instance for method chaining.
     */
    PIDRegulatorBuilder& setSensor(std::unique_ptr<Sensor> sensor)
    {
        m_sensor = std::move(sensor);
        return *this;
    }

    /**
     * @brief Sets the actuator for the regulator.
     * @param actuator A unique pointer to the actuator to be set.
     * @return Reference to the PIDRegulatorBuilder instance for method chaining.
     */
    PIDRegulatorBuilder& setActuator(std::unique_ptr<Actuator> actuator)
    {
        m_actuator = std::move(actuator);
        return *this;
    }

    /**
     * @brief Sets the proportional gain (Kp) for the regulator.
     * @param kp The proportional gain to be set.
     * @return Reference to the PIDRegulatorBuilder instance for method chaining.
     */
    PIDRegulatorBuilder& setKp(double kp)
    {
        m_kp = kp;
        return *this;
    }

    /**
     * @brief Sets the integral gain (Ki) for the regulator.
     * @param ki The integral gain to be set.
     * @return Reference to the PIDRegulatorBuilder instance for method chaining.
     */
    PIDRegulatorBuilder& setKi(double ki)
    {
        m_ki = ki;
        return *this;
    }

    /**
     * @brief Sets the derivative gain (Kd) for the regulator.
     * @param kd The derivative gain to be set.
     * @return Reference to the PIDRegulatorBuilder instance for method chaining.
     */
    PIDRegulatorBuilder& setKd(double kd)
    {
        m_kd = kd;
        return *this;
    }

    /**
     * @brief Sets the setpoint for the regulator.
     * @param setpoint The setpoint value to be set.
     * @return Reference to the PIDRegulatorBuilder instance for method chaining.
     */
    PIDRegulatorBuilder& setSetpoint(const SensorData setpoint)
    {
        m_setpoint = setpoint;
        return *this;
    }
    /**
     * @brief Sets the minimum integral value (for anti-windup).
     * @param integralMin The minimum integral value to be set.
     * @return Reference to the PIDRegulatorBuilder instance for method chaining.
     */
    PIDRegulatorBuilder& setIntegralMin(const SensorData integralMin)
    {
        m_integralMin = integralMin;
        return *this;
    }

    /**
     * @brief Sets the maximum integral value (for anti-windup).
     * @param integralMax The maximum integral value to be set.
     * @return Reference to the PIDRegulatorBuilder instance for method chaining.
     */
    PIDRegulatorBuilder& setIntegralMax(const SensorData integralMax)
    {
        m_integralMax = integralMax;
        return *this;
    }

    /**
     * @brief Builds and returns a PIDRegulator instance with the specified parameters.
     * @return A unique pointer to the constructed PIDRegulator instance.
     * @throws std::runtime_error if any required parameter is not set.
     */
    std::unique_ptr<PIDRegulator> build()
    {
        if (!m_kp.has_value())
        {
            throw std::runtime_error("Proportional gain (Kp) is not set");
        }
        if (!m_ki.has_value())
        {
            throw std::runtime_error("Integral gain (Ki) is not set");
        }
        if (!m_kd.has_value())
        {
            throw std::runtime_error("Derivative gain (Kd) is not set");
        }
        if (!m_setpoint.has_value())
        {
            throw std::runtime_error("Setpoint is not set");
        }
        if (!m_integralMin.has_value())
        {
            throw std::runtime_error("Minimum integral value is not set");
        }
        if (!m_integralMax.has_value())
        {
            throw std::runtime_error("Maximum integral value is not set");
        }

        std::string nameCopy = m_name;

        return std::make_unique<PIDRegulator>(
            nameCopy,
            std::move(m_sensor),
            std::move(m_actuator),
            m_kp.value(),
            m_ki.value(),
            m_kd.value(),
            m_setpoint.value(),
            m_integralMin.value(),
            m_integralMax.value());
    }
};

} // namespace control_engineering_uni_a

#endif // REGULATOR_PID_REGULATOR_BUILDER_HPP