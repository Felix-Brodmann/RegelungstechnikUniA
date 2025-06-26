#ifndef ACTUATOR_ACTUATOR_HPP
#define ACTUATOR_ACTUATOR_HPP

#include <string>

#include "utility/types.hpp"

namespace control_engineering_uni_a
{
    
/**
 * @class Actuator
 * @brief Abstract base class representing a generic actuator.
 * @details This class provides an interface for actuators, encapsulating the actuator's name and defining pure virtual methods for setting control values and stopping the actuator. Derived classes must implement the setControlValue and stop methods.
 * @note This class is intended to be subclassed for specific actuator implementations.
 * @author Felix Brodmann
 * @date 2025-05-19
 */
class Actuator
{
private:
    std::string m_actuatorName;

public:
    /**
     * @brief Constructor for the Actuator class.
     * @param t_actuatorName The name of the actuator.
     */
    Actuator(const std::string &t_actuatorName)
    {
        m_actuatorName = t_actuatorName;
    }

    virtual ~Actuator() = default;

    /**
     * @brief Get the name of the actuator.
     * @return The name of the actuator.
     */
    std::string getActuatorName() const
    {
        return m_actuatorName;
    }

    /**
     * @brief Set the control value for the actuator.
     * @param u The control value to be set.
     * @details This method must be implemented by derived classes to set the control value for the actuator.
     */
    virtual void setControlValue(SensorData u) = 0;

    /**
     * @brief Stop the actuator.
     * @details This method must be implemented by derived classes to stop the actuator.
     */
    virtual void stop() = 0;
};

} // namespace control_engineering_uni_a

#endif // ACTUATOR_ACTUATOR_HPP