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
        
        // PWM-Steuerungsmethode
        void pwmThread(const int t_dutyCycle);

        // Tachometer-Überwachungsmethoden
        void monitorWhenTachoAActivated();
        void monitorWhenTachoBActivated();
        void monitorWhenTachoADeactivated();
        void monitorWhenTachoBDeactivated();
        void tachoARising();
        void tachoAFalling();
        void tachoBRising();
        void tachoBFalling();

    public:
        EV3Motor(const int t_pwmPin, const int t_forwardPin, const int t_backwardPin, const int t_pwmFrequency, const int t_tachAPin, const int t_tachBPin);
        ~EV3Motor();
        double getOmega();
        void setSpeed(const int t_speed);

    };
}

#endif