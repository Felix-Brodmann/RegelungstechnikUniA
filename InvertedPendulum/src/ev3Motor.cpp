#include "../include/ev3Motor.hpp"

namespace RegelungstechnikUniA
{
    EV3Motor::EV3Motor(const int t_pwmPin, const int t_forwardPin, const int t_backwardPin, const int t_pwmFrequency)
    {
        if (gpioInitialise() < 0)
        {
            throw std::runtime_error("Failed to initialize pigpio");
        }
        m_pwmPin = t_pwmPin;
        m_forwardPin = t_forwardPin;
        m_backwardPin = t_backwardPin;
        m_pwmFrequency = t_pwmFrequency;
        gpioSetMode(m_pwmPin, PI_OUTPUT);
        gpioSetMode(m_forwardPin, PI_OUTPUT);
        gpioSetMode(m_backwardPin, PI_OUTPUT);
        gpioHardwarePWM(m_pwmPin, t_pwmFrequency, 0);
        gpioWrite(m_forwardPin, 0);
        gpioWrite(m_backwardPin, 0);
    }

    EV3Motor::~EV3Motor()
    {
        gpioHardwarePWM(m_pwmPin, 0, 0);
        gpioWrite(m_forwardPin, 0);
        gpioWrite(m_backwardPin, 0);
        gpioTerminate();
    }

    void EV3Motor::setSpeed(const int t_speed)
    {
        if (t_speed < -100 || t_speed > 100)
        {
            throw std::runtime_error("Invalid speed. Speed must be between -100 and 100");
        }
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
        gpioHardwarePWM(m_pwmPin, m_pwmFrequency, (int)(std::abs(t_speed) / 100 * 1000000));
    }
}