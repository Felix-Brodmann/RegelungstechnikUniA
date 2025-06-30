#ifndef EV3_MOTOR_HPP
#define EV3_MOTOR_HPP

// Checks if the platform is Linux and ARM-based (e.g., Raspberry Pi)
#if defined(__linux__) && (defined(__arm__) || defined(__aarch64__))

#include <pigpio.h>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <cmath>
#include <string>
#include <memory>
#include <map>

#include "actuator/actuator.hpp"
#include "utility/pwm-device.hpp"

namespace control_engineering_uni_a
{

/**
 * @class EV3Motor
 * @brief Represents an EV3 motor actuator with tacho feedback and PWM control.
 * @details This class provides an interface to control an EV3 motor using GPIO pins for direction and tacho feedback, and a PWM device for speed control. It supports real-time monitoring of the motor's rotational speed (omega) using a background thread that processes tacho signals.
 * @note The motor is controlled using two GPIO pins for direction (forward and backward) and two GPIO pins for tacho feedback (A and B). The PWM device is used to set the motor speed.
 * @author Felix Brodmann
 * @date 2025-05-21
 */
class EV3Motor : public Actuator
{
private:
    const double MOTOR_RESISTANCE = 13.0; // [Ohm]
    const double MOTOR_BACK_EMF = 0.26; // [Vs/rad]
    const double SUPPLY_VOLTAGE = 8.3; // [V] (at 10V supply voltage, notice voltage drop)

    int m_forwardPin = 0;
    int m_backwardPin = 0;
    int m_tachoAPin = 0;
    int m_tachoBPin = 0;
    std::map<std::string, bool> m_initilizationList; // Initialization list for the EV3Motor

    bool m_isRunning = true;
    std::unique_ptr<PwmDevice> m_pwmDevice;

    // tacho monitoring
    int m_tachoAValue = 0;
    int m_tachoBValue = 0;
    bool m_tachoAPrevOn = false;
    bool m_tachoBPrevOn = false;
    int m_tachoCounter = 0;
    int m_prevTachoCounter = 0;
    double m_omega = 0;
    std::chrono::time_point<std::chrono::steady_clock> m_timeStemp = std::chrono::steady_clock::now();
    std::thread m_monitorTachoThread;

    /**
     * @brief Monitor tacho signals in a separate thread.
     * @details This method continuously reads the tacho signals from the GPIO pins and updates the tacho counter based on the rising and falling edges of the signals. It runs in a separate thread to avoid blocking the main program.
     */
    void monitorTacho()
    {
        while (m_isRunning)
        {
            int tachoA = gpioRead(m_tachoAPin);
            int tachoB = gpioRead(m_tachoBPin);

            if (tachoA == 1 && !m_tachoAPrevOn) tachoARising();
            if (tachoA == 0 && m_tachoAPrevOn) tachoAFalling();
            m_tachoAPrevOn = tachoA;

            if (tachoB == 1 && !m_tachoBPrevOn) tachoBRising();
            if (tachoB == 0 && m_tachoBPrevOn) tachoBFalling();
            m_tachoBPrevOn = tachoB;

            gpioDelay(100); // 100 µs pause
        }
    }

    /**
     * @brief Handle rising edge of tacho A signal.
     * @details This method is called when the tacho A signal transitions from low to high. It updates the tacho counter based on the state of tacho B.
     */
    void tachoARising()
    {
        m_tachoAValue = 1;
        m_tachoCounter += (m_tachoBValue == 0) ? 1 : -1;
    }

    /**
     * @brief Handle falling edge of tacho A signal.
     * @details This method is called when the tacho A signal transitions from high to low. It updates the tacho A value.
     */
    void tachoAFalling()
    {
        m_tachoAValue = 0;
    }

    /**
     * @brief Handle rising edge of tacho B signal.
     * @details This method is called when the tacho B signal transitions from low to high. It updates the tacho counter based on the state of tacho A.
     */
    void tachoBRising()
    {
        m_tachoBValue = 1;
        m_tachoCounter += (m_tachoAValue == 0) ? -1 : 1;
    }

    /**
     * @brief Handle falling edge of tacho B signal.
     * @details This method is called when the tacho B signal transitions from high to low. It updates the tacho B value.
     */
    void tachoBFalling()
    {
        m_tachoBValue = 0;
    }

    /**
     * @brief Set the initialization status for a given variable name.
     * @param t_name The variable name for which the initialization status is to be set.
     * @param t_status The initialization status to be set (true for initialized, false for not initialized).
     */
    void setInitilizationStatus(const std::string& t_name, bool t_status)
    {
        m_initilizationList[t_name] = t_status;
    }

    /**
     * @brief Check if a variable is initialized.
     * @param t_name The variable name to check for initialization status.
     * @return True if the variable is initialized, false otherwise.
     */
    bool isInitilized(const std::string& t_name) const
    {
        auto it = m_initilizationList.find(t_name);
        if (it != m_initilizationList.end())
        {
            return it->second;
        }
        return false; // Default to false if not found
    }

public:
    /**
     * @brief Complete constructor for the for the EV3Motor class.
     * @param t_pwmDevice The PWM device used for motor control.
     * @param t_forwardPin The GPIO pin for forward direction control.
     * @param t_backwardPin The GPIO pin for backward direction control.
     * @param t_tachoAPin The GPIO pin for tacho A feedback.
     * @param t_tachoBPin The GPIO pin for tacho B feedback.
     */
    EV3Motor(std::unique_ptr<PwmDevice> t_pwmDevice, int t_forwardPin, int t_backwardPin,int t_tachoAPin, int t_tachoBPin) : Actuator("EV3Motor")
    {
        setPWMDevice(std::move(t_pwmDevice));
        setForwardPin(t_forwardPin);
        setBackwardPin(t_backwardPin);
        setTachoAPin(t_tachoAPin);
        setTachoBPin(t_tachoBPin);
    }

    /**
     * @brief Partial constructor for the EV3Motor class.
     */
    EV3Motor() : Actuator("EV3Motor")
    {
        setInitilizationStatus("pwmDevice", false);
        setInitilizationStatus("forwardPin", false);
        setInitilizationStatus("backwardPin", false);
        setInitilizationStatus("tachoAPin", false);
        setInitilizationStatus("tachoBPin", false);
    }

    /**
     * @brief Destructor for the EV3Motor class.
     * @details This method stops the motor and joins the monitoring thread before destroying the object.
     */
    ~EV3Motor()
    {
        m_isRunning = false;
        if (m_monitorTachoThread.joinable())
        {
            m_monitorTachoThread.join();
        }

        // Stop motor
        m_pwmDevice->setDutyCycle(0);
        gpioWrite(m_forwardPin, 0);
        gpioWrite(m_backwardPin, 0);

        gpioTerminate();
    }

     /**
     * @brief Delete copy constructor and assignment operator to prevent copying.
     * @details The copy constructor is deleted to prevent copying of the EV3Motor instance, as it contains a thread that should not be copied.
     */
    EV3Motor(const EV3Motor&) = delete;

    /**
     * @brief Delete assignment operator to prevent copying.
     * @details This operator is deleted to prevent copying of the EV3Motor instance, as it contains a thread that should not be copied.
     */
    EV3Motor& operator=(const EV3Motor&) = delete;

    void initialize()
    {
        // Check if all required components are initialized
        if (!isInitilized("pwmDevice") || !isInitilized("forwardPin") || !isInitilized("backwardPin") ||
            !isInitilized("tachoAPin") || !isInitilized("tachoBPin"))
        {
            throw std::runtime_error("EV3Motor is not initialized properly, make sure to set the PWM device and the Control GPIO pins");
        }

        // Initialize GPIO
        if (gpioSetMode(m_forwardPin, PI_OUTPUT) < 0 ||
            gpioSetMode(m_backwardPin, PI_OUTPUT) < 0 ||
            gpioSetMode(m_tachoAPin, PI_INPUT) < 0 ||
            gpioSetMode(m_tachoBPin, PI_INPUT) < 0)
        {
            throw std::runtime_error("Failed to set GPIO mode");
        }

        gpioWrite(m_forwardPin, 0);
        gpioWrite(m_backwardPin, 0);

        // Start tacho monitoring thread
        m_monitorTachoThread = std::thread(&EV3Motor::monitorTacho, this);
        m_timeStemp = std::chrono::steady_clock::now();

        // Start with 0%
        m_pwmDevice->setDutyCycle(0);
    }

    /**
     * @brief Set the PWM device for the motor.
     * @param t_pwmDevice A unique pointer to the PWM device to be set.
     */
    void setPWMDevice(std::unique_ptr<PwmDevice> t_pwmDevice)
    {
        // Check if the PWM device is already initialized
        if (isInitilized("pwmDevice"))
        {
            throw std::runtime_error("PWM device is already set, cannot set it again");
        }

        // Set the initialization status and store the PWM device
        setInitilizationStatus("pwmDevice", true);
        m_pwmDevice = std::move(t_pwmDevice);
    }

    /**
     * @brief Set the GPIO pin for forward direction control.
     * @param t_forwardPin The GPIO pin number for forward direction control.
     */
    void setForwardPin(int t_forwardPin)
    {
        setInitilizationStatus("forwardPin", true);
        m_forwardPin = t_forwardPin;
    }

    /**
     * @brief Set the GPIO pin for backward direction control.
     * @param t_backwardPin The GPIO pin number for backward direction control.
     */
    void setBackwardPin(int t_backwardPin)
    {
        setInitilizationStatus("backwardPin", true);
        m_backwardPin = t_backwardPin;
    }

    /**
     * @brief Set the GPIO pin for tacho A feedback.
     * @param t_tachoAPin The GPIO pin number for tacho A feedback.
     */
    void setTachoAPin(int t_tachoAPin)
    {
        setInitilizationStatus("tachoAPin", true);
        m_tachoAPin = t_tachoAPin;
    }

    /**
     * @brief Set the GPIO pin for tacho B feedback.
     * @param t_tachoBPin The GPIO pin number for tacho B feedback.
     */
    void setTachoBPin(int t_tachoBPin)
    {
        setInitilizationStatus("tachoBPin", true);
        m_tachoBPin = t_tachoBPin;
    }

    /**
     * @brief Get the current angular velocity of the motor.
     * @return The angular velocity in radians per second.
     * @details This method calculates the angular velocity based on the tacho counter and the time interval since the last update. It uses a moving average to smooth out the readings.
     */
    double getOmega()
    {
        constexpr double WHEEL_AV_INTERVAL = 0.025;
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> deltaTime = currentTime - m_timeStemp;
        if (deltaTime.count() > WHEEL_AV_INTERVAL)
        {
            m_timeStemp = currentTime;
            int pulseCount = m_tachoCounter - m_prevTachoCounter;
            m_prevTachoCounter = m_tachoCounter;
            double cycles = static_cast<double>(pulseCount) / 360.0;
            m_omega = cycles / deltaTime.count() * 2 * M_PI;
        }
        return m_omega;
    }

    /**
     * @brief Set the control value for the motor.
     * @param u The control value to be set.
     * @details This method sets the motor speed based on the control value. It uses the PWM device to set the duty cycle and controls the direction using GPIO pins. The control value must be between -100 and 100.
     * @throws std::runtime_error If the control value is out of range or if the motor is not initialized properly.
     */
    void setControlValue(SensorData u) override
    {
        // Check if the motor is initialized
        if (!isInitilized("pwmDevice") || !isInitilized("forwardPin") || !isInitilized("backwardPin"))
        {
            throw std::runtime_error("EV3Motor is not initialized properly, make sure to set the PWM device and the Control GPIO pins");
        }
        // Check if the control value is of type int or double
        if (!std::holds_alternative<int>(u) && !std::holds_alternative<double>(u))
        {
            throw std::runtime_error("Control value must be of type int or double");
        }

        // Convert the control value to int
        int t_speed;
        if (std::holds_alternative<double>(u))
        {
            t_speed = static_cast<int>(std::get<double>(u));
        }
        else
        {
            t_speed = std::get<int>(u);
        }

        // Check if the control value is within the valid range
        if (t_speed < -100 || t_speed > 100)
            throw std::runtime_error("Speed must be between -100 and 100. Your speed: " + std::to_string(t_speed));

        // Set the direction
        if (t_speed > 0)
        {
            gpioWrite(m_forwardPin, 1);
            gpioWrite(m_backwardPin, 0);
        }
        else if (t_speed < 0)
        {
            gpioWrite(m_forwardPin, 0);
            gpioWrite(m_backwardPin, 1);
        }
        else
        {
            gpioWrite(m_forwardPin, 0);
            gpioWrite(m_backwardPin, 0);
        }

        // Set the duty cycle
        m_pwmDevice->setDutyCycle(std::abs(t_speed));

        // Log the control value
        getOutputStream().write(getActuatorName(), "Set control value: " + std::to_string(t_speed) + "%");
    }

    /**
     * @brief Stop the motor.
     * @details This method stops the motor by setting the duty cycle to 0 and turning off both direction GPIO pins.
     */
    void stop() override
    {
        m_pwmDevice->setDutyCycle(0);
        gpioWrite(m_forwardPin, 0);
        gpioWrite(m_backwardPin, 0);
    }
};

} // namespace control_engineering_uni_a

#elif defined(ARDUINO)

    #error "Support for the Arduino platform is not implemented yet. This library is currently only supported for Raspberry Pi 3/4"

#else

    #error "Unsupported platform. This library is only supported on the Raspberry Pi 3/4 (Arduino is not supported yet)."

#endif // defined(__linux__) && (defined(__arm__) || defined(__aarch64__))

#endif // EV3_MOTOR_HPP