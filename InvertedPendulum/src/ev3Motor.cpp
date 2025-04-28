#include "../include/ev3Motor.hpp"

namespace RegelungstechnikUniA
{
    void EV3Motor::pwmThread(const int t_dutyCycle)
    {
        std::chrono::duration<int64_t, std::micro> period = std::chrono::microseconds(1000000 / m_pwmFrequency);
        std::chrono::duration<int64_t, std::micro> highTime = std::chrono::microseconds(1000000 / m_pwmFrequency) * t_dutyCycle / 100;
        std::chrono::duration<int64_t, std::micro> lowTime = period - highTime;
        while (m_pwmIsRunning)
        {
            gpiod_line_set_value(m_pwmLine, 1);
            std::this_thread::sleep_for(highTime);
            gpiod_line_set_value(m_pwmLine, 0);
            std::this_thread::sleep_for(lowTime);
        }
    }

    void EV3Motor::monitorWhenTachoAActivated()
    {
        while (m_isRunning)
        {
            int tachoAValue = gpiod_line_get_value(m_tachoALine);
            if (tachoAValue == 1)
            {
                if (m_tachoAPrevOn == false)
                {
                    tachoARising();
                }
                m_tachoAPrevOn = true;
            }
            else
            {
                m_tachoAPrevOn = false;
            }
        }
    }

    void EV3Motor::monitorWhenTachoBActivated()
    {
        while (m_isRunning)
        {
            int tachoBValue = gpiod_line_get_value(m_tachoBLine);
            if (tachoBValue == 1)
            {
                if (m_tachoBPrevOn == false)
                {
                    tachoBRising();
                }
                m_tachoBPrevOn = true;
            }
            else
            {
                m_tachoBPrevOn = false;
            }
        }
    }

    void EV3Motor::monitorWhenTachoADeactivated()
    {
        while (m_isRunning)
        {
            int tachoAValue = gpiod_line_get_value(m_tachoALine);
            if (tachoAValue == 0)
            {
                if (m_tachoAPrevOn == true)
                {
                    tachoAFalling();
                }
                m_tachoAPrevOn = false;
            }
        }
    }

    void EV3Motor::monitorWhenTachoBDeactivated()
    {
        while (m_isRunning)
        {
            int tachoBValue = gpiod_line_get_value(m_tachoBLine);
            if (tachoBValue == 0)
            {
                if (m_tachoBPrevOn == true)
                {
                    tachoBFalling();
                }
                m_tachoBPrevOn = false;
            }
        }
    }

    void EV3Motor::tachoARising()
    {
        m_tachoAValue = 1;
        if (m_tachoBValue == 0)
        {
            m_tachoCounter++;
        }
        else
        {
            m_tachoCounter--;
        }
    }

    void EV3Motor::tachoAFalling()
    {
        m_tachoAValue = 0;
    }

    void EV3Motor::tachoBRising()
    {
        m_tachoBValue = 1;
        if (m_tachoAValue == 0)
        {
            m_tachoCounter--;
        }
        else
        {
            m_tachoCounter++;
        }
    }

    void EV3Motor::tachoBFalling()
    {
        m_tachoBValue = 0;
    }

    EV3Motor::EV3Motor(const int t_pwmPin, const int t_forwardPin, const int t_backwardPin, const int t_pwmFrequency, const int t_tachAPin, const int t_tachBPin)
    {
        if (t_pwmFrequency < 0 || t_pwmFrequency > 10000)
        {
            throw std::runtime_error("Invalid PWM frequency. Frequency must be between 0 and 10.000");
        }
        m_chip = gpiod_chip_open(m_chipName);
        if (m_chip == nullptr)
        {
            throw std::runtime_error("Failed to initialize pigpio");
        }
        m_pwmFrequency = t_pwmFrequency;
        m_pwmLine = gpiod_chip_get_line(m_chip, t_pwmPin);
        m_forwardLine = gpiod_chip_get_line(m_chip, t_forwardPin);
        m_backwardLine = gpiod_chip_get_line(m_chip, t_backwardPin);
        m_tachoALine = gpiod_chip_get_line(m_chip, t_tachAPin);
        m_tachoBLine = gpiod_chip_get_line(m_chip, t_tachBPin);
        if (!m_pwmLine || !m_forwardLine || !m_backwardLine || !m_tachoALine || !m_tachoBLine)
        {
            throw std::runtime_error("Failed to initialize the pins");
        }
        if (gpiod_line_request_output(m_pwmLine, "pwm", 0) || gpiod_line_request_output(m_forwardLine, "forward", 0) || gpiod_line_request_output(m_backwardLine, "backward", 0) || gpiod_line_request_input(m_tachoALine, "tachA") || gpiod_line_request_input(m_tachoBLine, "tachB"))
        {
            throw std::runtime_error("Failed to set pin mode");
        }
        m_pwmThread = std::thread(&EV3Motor::pwmThread, this, 0);
        m_timeStemp = std::chrono::steady_clock::now();
        m_monitorWhenTachoAActivated = std::thread(&EV3Motor::monitorWhenTachoAActivated, this);
        m_monitorWhenTachoBActivated = std::thread(&EV3Motor::monitorWhenTachoBActivated, this);
        m_monitorWhenTachoADeactivated = std::thread(&EV3Motor::monitorWhenTachoADeactivated, this);
        m_monitorWhenTachoBDeactivated = std::thread(&EV3Motor::monitorWhenTachoBDeactivated, this);
    }

    EV3Motor::~EV3Motor()
    {
        gpiod_line_release(m_pwmLine);
        gpiod_line_release(m_forwardLine);
        gpiod_line_release(m_backwardLine);
        gpiod_chip_close(m_chip);
        m_isRunning = false;
        m_pwmIsRunning = false;
        m_pwmThread.join();
        m_monitorWhenTachoAActivated.join();
        m_monitorWhenTachoBActivated.join();
        m_monitorWhenTachoADeactivated.join();
        m_monitorWhenTachoBDeactivated.join();
    }

    double EV3Motor::getOmega()
    {
        double WHEEL_AV_INTERVAL = 0.025; // [s]
        std::chrono::time_point<std::chrono::steady_clock> currentTime = std::chrono::steady_clock::now(); // [us]
        std::chrono::duration<double> deltaTimeInMicroseconds = currentTime - m_timeStemp; // [us]
        double deltaTime = deltaTimeInMicroseconds.count(); // [s]
        if (deltaTime > WHEEL_AV_INTERVAL)
        {
            m_timeStemp = currentTime;

            // Berechnen der Umdrehungen
            int pulseCounter = m_tachoCounter - m_prevTachoCounter;
            m_prevTachoCounter = m_tachoCounter;
            double cycles = static_cast<float>(pulseCounter) / 360.0f;

            // Berechnen der Winkelgeschwindigkeit in rad/s
            m_omega = cycles / deltaTime * 2 * M_PI;
        }
        return m_omega;
    }

    void EV3Motor::setSpeed(const int t_speed)
    {
        m_pwmIsRunning = false;
        m_pwmThread.join();
        if (t_speed < -100 || t_speed > 100)
        {
            throw std::runtime_error("Invalid speed. Speed must be between -100 and 100");
        }
        if (t_speed > 0)
        {
            gpiod_line_set_value(m_forwardLine, 1);
            gpiod_line_set_value(m_backwardLine, 0);
        }
        else if (t_speed < 0)
        {
            gpiod_line_set_value(m_forwardLine, 0);
            gpiod_line_set_value(m_backwardLine, 1);
        }
        else
        {
            gpiod_line_set_value(m_forwardLine, 0);
            gpiod_line_set_value(m_backwardLine, 0);
        }
        m_pwmIsRunning = true;
        m_pwmThread = std::thread(&EV3Motor::pwmThread, this, std::abs(t_speed));
    }
}