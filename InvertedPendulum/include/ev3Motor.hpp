#ifndef EV3_MOTOR_HPP
#define EV3_MOTOR_HPP

#include <pigpio.h>
#include <stdexcept>

namespace RegelungstechnikUniA
{
    class EV3Motor
    {
    private:
        int m_pwmPin = 0;
        int m_forwardPin = 0;
        int m_backwardPin = 0;
        int m_pwmFrequency = 0;

    public:
        EV3Motor(const int t_pwmPin, const int t_forwardPin, const int t_backwardPin, const int t_pwmFrequency);
        ~EV3Motor();
        void setSpeed(const int t_speed);
    };
}

#endif