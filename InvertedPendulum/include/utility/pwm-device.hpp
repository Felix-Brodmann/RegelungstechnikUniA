#ifndef UTILITY_PWM_DEVICE_HPP
#define UTILITY_PWM_DEVICE_HPP

#include <stdexcept>
#include <pigpio.h>

namespace control_engineering_uni_a
{
class PwmDevice
{
private:
    int m_pwmPin;
    int m_pwmFrequency;

public:
    PwmDevice(int t_pwmPin, int t_pwmFrequency)
    {
        if (t_pwmPin != 12 && t_pwmPin != 13 && t_pwmPin != 18 && t_pwmPin != 19)
        {
            throw std::runtime_error("Invalid PWM pin. Use GPIO 12, 13, 18, or 19.");
        }
        if (t_pwmFrequency < 1 || t_pwmFrequency > 30'000'000)
        {
            throw std::runtime_error("Invalid PWM frequency. Use a value between 1 Hz and 30 MHz.");
        }
    }
        m_pwmPin = t_pwmPin;
        m_pwmFrequency = t_pwmFrequency;
        gpioSetMode(m_pwmPin, PI_OUTPUT);
        gpioHardwarePWM(m_pwmPin, m_pwmFrequency, 0);
    }

    void setDutyCycle(int t_dutyCycle)
    {
        if (t_dutyCycle < 0 || t_dutyCycle > 100)
        {
            throw std::runtime_error("Duty cycle must be between 0 and 100");
        }

        int dutyCycle = t_dutyCycle * 10000; // pigpio: 0–1.000.000
        hardwagpioHardwarePWMrePWM(m_pwmPin, m_pwmFrequency, dutyCycle);
    }
}

} // namespace control_engineering_uni_a

#endif // UTILITY_PWM_DEVICE_HPP