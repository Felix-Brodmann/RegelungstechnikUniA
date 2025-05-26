#ifndef ACTOR_MOTOR_EV3_MOTOR_BUILDER_HPP
#define ACTOR_MOTOR_EV3_MOTOR_BUILDER_HPP

#include <string>
#include <memory>

#include "actuator/motor/ev3-motor.hpp"

namespace control_engineering_uni_a
{

/**
 * @class EV3MotorBuilder
 * @brief Builder class for constructing and configuring an EV3Motor instance.
 * @details This class provides a fluent interface for setting the parameters of an EV3 motor and building it.
 * @author Felix Brodmann
 * @date 2025-05-26
 */
class EV3MotorBuilder
{
private:
    PwmDevice m_pwmDevice; // PWM device for motor control
    int forwardPin = 0; // GPIO pin for forward direction control
    int backwardPin = 0; // GPIO pin for backward direction control
    int tachoAPin = 0; // GPIO pin for tacho A feedback
    int tachoBPin = 0; // GPIO pin for tacho B feedback

public:
    static EV3MotorBuilder create()
    {
        return EV3MotorBuilder();
    }

    /**
     * @brief Sets the PWM device for the motor.
     * @param pwmDevice The PWM device to be set.
     * @return Reference to the EV3MotorBuilder instance for method chaining.
     */
    EV3MotorBuilder& setPwmDevice(const PwmDevice& pwmDevice)
    {
        m_pwmDevice = pwmDevice;
        return *this;
    }

    /**
     * @brief Sets the GPIO pin for forward direction control.
     * @param pin The GPIO pin number for forward direction control.
     * @return Reference to the EV3MotorBuilder instance for method chaining.
     */
    EV3MotorBuilder& setForwardPin(int pin)
    {
        forwardPin = pin;
        return *this;
    }

    /**
     * @brief Sets the GPIO pin for backward direction control.
     * @param pin The GPIO pin number for backward direction control.
     * @return Reference to the EV3MotorBuilder instance for method chaining.
     */
    EV3MotorBuilder& setBackwardPin(int pin)
    {
        backwardPin = pin;
        return *this;
    }

    /**
     * @brief Sets the GPIO pin for tacho A feedback.
     * @param pin The GPIO pin number for tacho A feedback.
     * @return Reference to the EV3MotorBuilder instance for method chaining.
     */
    EV3MotorBuilder& setTachoAPin(int pin)
    {
        tachoAPin = pin;
        return *this;
    }

    /**
     * @brief Sets the GPIO pin for tacho B feedback.
     * @param pin The GPIO pin number for tacho B feedback.
     * @return Reference to the EV3MotorBuilder instance for method chaining.
     */
    EV3MotorBuilder& setTachoBPin(int pin)
    {
        tachoBPin = pin;
        return *this;
    }

    /**
     * @brief Builds the EV3Motor instance with the configured parameters.
     * @return A unique pointer to the constructed EV3Motor instance.
     * @throws std::runtime_error If any required parameter is not set.
     */
    std::unique_ptr<EV3Motor> build()
    {
        if (forwardPin == 0 || backwardPin == 0 || tachoAPin == 0 || tachoBPin == 0)
        {
            throw std::runtime_error("EV3MotorBuilder: Not all required parameters are set.");
        }
        std::unique_ptr<EV3Motor> motor = std::make_unique<EV3Motor>();
        motor->setPWMDevice(m_pwmDevice);
        motor->setForwardPin(forwardPin);
        motor->setBackwardPin(backwardPin);
        motor->setTachoAPin(tachoAPin);
        motor->setTachoBPin(tachoBPin);
        return motor;
    }
};

} // namespace control_engineering_uni_a

#ifndef ACTOR_MOTOR_EV3_MOTOR_BUILDER_HPP