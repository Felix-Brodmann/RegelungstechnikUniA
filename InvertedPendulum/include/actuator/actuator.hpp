#ifndef ACTUATOR_ACTUATOR_HPP
#define ACTUATOR_ACTUATOR_HPP

#include <string>

#include "utility/types.hpp"
#include "utility/output-stream.hpp"
#include "utility/output-stream/void-output-stream.hpp"

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
    std::shared_ptr<OutputStream> m_outputStream; // Optional output stream for logging or debugging

public:
    /**
     * @brief Constructor for the Actuator class.
     * @param t_actuatorName The name of the actuator.
     */
    Actuator(const std::string &t_actuatorName)
    {
        m_actuatorName = t_actuatorName;
        m_outputStream = std::make_shared<VoidOutputStream>(); // Default output stream
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
     * @brief Get the output stream for the actuator.
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
     * @brief Set the output stream for the actuator.
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