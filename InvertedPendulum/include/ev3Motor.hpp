#ifndef EV3_MOTOR_HPP
#define EV3_MOTOR_HPP

#include <gpiod.h>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <cmath>

namespace RegelungstechnikUniA
{
    class EV3Motor
    {
    private:

        // Motor-Konstanten
        const int m_motorResistance = 13; // [Ohm]
        const float m_motorBackEMF = 0.26; // [Vs/rad]
        const float m_supplyVoltage = 8.3; // [V] (bei 10V supply voltage)

        // GPIO
        const char* m_chipName = "/dev/gpiochip0";
        gpiod_chip* m_chip = nullptr;
        gpiod_line* m_pwmLine = nullptr;
        gpiod_line* m_forwardLine = nullptr;
        gpiod_line* m_backwardLine = nullptr;
        gpiod_line* m_tachoALine = nullptr;
        gpiod_line* m_tachoBLine = nullptr;

        int m_pwmPin = 0;
        int m_forwardPin = 0;
        int m_backwardPin = 0;
        int m_tachoAPin = 0;
        int m_tachoBPin = 0;

        // PWM-Steuerungsthread
        std::thread m_pwmThread;

        // Tachometer-Überwachungsthreads
        std::thread m_monitorWhenTachoAActivated;
        std::thread m_monitorWhenTachoBActivated;
        std::thread m_monitorWhenTachoADeactivated;
        std::thread m_monitorWhenTachoBDeactivated;

        // Thread Running Flags
        bool m_pwmIsRunning = true;
        bool m_isRunning = true;

        // PWM-Frequenz
        int m_pwmFrequency = 0;

        // Tachometerwerte
        int m_tachoAValue = 0;
        int m_tachoBValue = 0;
        bool m_tachoAPrevOn = false;
        bool m_tachoBPrevOn = false;
        int m_tachoCounter = 0;
        int m_prevTachoCounter = 0;
        double m_omega = 0;
        std::chrono::time_point<std::chrono::steady_clock> m_timeStemp = std::chrono::steady_clock::now();

        void EV3Motor::monitorTacho()
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

                gpioDelay(100); // 100 µs Pause
            }
        }

        void EV3Motor::tachoARising()
        {
            m_tachoAValue = 1;
            m_tachoCounter += (m_tachoBValue == 0) ? 1 : -1;
        }

        void EV3Motor::tachoAFalling()
        {
            m_tachoAValue = 0;
        }

        void EV3Motor::tachoBRising()
        {
            m_tachoBValue = 1;
            m_tachoCounter += (m_tachoAValue == 0) ? -1 : 1;
        }

        void EV3Motor::tachoBFalling()
        {
            m_tachoBValue = 0;
        }

    public:
        EV3Motor::EV3Motor(int t_pwmPin, int t_forwardPin, int t_backwardPin, int t_pwmFrequency, int t_tachAPin, int t_tachBPin)
        {
            if (gpioInitialise() < 0) throw std::runtime_error("pigpio initialisation failed");

            gpioSetMode(m_forwardPin, PI_OUTPUT);
            gpioSetMode(m_backwardPin, PI_OUTPUT);
            gpioSetMode(m_tachoAPin, PI_INPUT);
            gpioSetMode(m_tachoBPin, PI_INPUT);

            gpioWrite(m_forwardPin, 0);
            gpioWrite(m_backwardPin, 0);

            // start tacho monitoring thread
            m_monitorTacho = std::thread(&EV3Motor::monitorTacho, this);
            m_timeStemp = std::chrono::steady_clock::now();

            // start with 0%
            gpioHardwarePWM(m_pwmPin, m_pwmFrequency, 0);
        }

        EV3Motor::~EV3Motor()
        {
            m_isRunning = false;
            m_monitorTacho.join();

            // Stop motor
            gpioHardwarePWM(m_pwmPin, 0, 0);
            gpioWrite(m_forwardPin, 0);
            gpioWrite(m_backwardPin, 0);

            gpioTerminate();
        }

        double EV3Motor::getOmega()
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

        void EV3Motor::setSpeed(int t_speed)
        {
            if (t_speed < -100 || t_speed > 100)
                throw std::runtime_error("Speed must be between -100 and 100");

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

            int dutyCycle = std::abs(t_speed) * 10000; // pigpio: 0–1.000.000
            gpioHardwarePWM(m_pwmPin, m_pwmFrequency, dutyCycle);
        }

    };
}

#endif
