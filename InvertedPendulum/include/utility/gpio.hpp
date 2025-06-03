#ifndef UTILITY_GPIO_HPP
#define UTILITY_GPIO_HPP

// Checks if the platform is Linux and ARM-based (e.g., Raspberry Pi)
#if defined(__linux__) && (defined(__arm__) || defined(__aarch64__))

#include <stdexcept>
#include <pigpio.h>

namespace control_engineering_uni_a
{
    static void initializeGpio()
    {
        if (gpioInitialise() < 0)
        {
            throw std::runtime_error("Failed to initialize pigpio library");
        }
    }
}

#elif defined(ARDUINO)

    #error "Support for the Arduino platform is not implemented yet. This library is currently only supported for Raspberry Pi 3/4"

#endif // defined(__linux__) && (defined(__arm__) || defined(__aarch64__))

     #error "Unsupported platform. This library is only supported on the Raspberry Pi 3/4 (Arduino is not supported yet)."

#endif // UTILITY_I2CDEVICE_HPP