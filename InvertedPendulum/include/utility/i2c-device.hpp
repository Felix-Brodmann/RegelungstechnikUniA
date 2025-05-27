#ifndef UTILITY_I2CDEVICE_HPP
#define UTILITY_I2CDEVICE_HPP

// Checks if the platform is Linux and ARM-based (e.g., Raspberry Pi)
#if defined(__linux__) && (defined(__arm__) || defined(__aarch64__))

#include <cstdint> // for uint8_t
#include <unistd.h> // fore write, read, close
#include <sys/ioctl.h> // for ioctl
#include <fcntl.h> // for open
#include <stdexcept> // for std::runtime_error
#include <linux/i2c-dev.h> // for I2C_SLAVE

namespace control_engineering_uni_a
{
using fileDescriptor = int;

/**
 * @class I2CDevice
 * @brief A class to handle I2C device communication.
 * @details This class provides methods to open an I2C device, write to registers, and read from registers.
 * @author Felix Brodmann
 * @date 2025-05-26
 */
class I2CDevice
{
private:
    fileDescriptor m_fileDescriptor;

public:
    /**
     * @brief Constructs an I2CDevice object.
     * @param t_devicePath The path to the I2C device (e.g., "/dev/i2c-1").
     * @param t_address The I2C address of the device.
     * @throws std::invalid_argument If the device path or address is invalid.
     * @throws std::runtime_error If the device cannot be opened or if there is an error in ioctl.
     */
    I2CDevice(const char *t_devicePath, const uint8_t t_address)
    {
        if (t_devicePath == nullptr || t_address == 0)
        {
            throw std::invalid_argument("Invalid device path or address");
        }
        m_fileDescriptor = open(t_devicePath, O_RDWR);
        if (m_fileDescriptor < 0)
        {
            throw std::runtime_error("Failed to open the i2c bus");
        }
        if (ioctl(m_fileDescriptor, I2C_SLAVE, t_address) < 0)
        {
            throw std::runtime_error("Failed to acquire bus access and/or talk to the slave");
        }
    }

    /**
     * @brief Destructor for I2CDevice.
     * @details Closes the file descriptor associated with the I2C device.
     */
    ~I2CDevice()
    {
        close(m_fileDescriptor);
    }

    /**
     * @brief Delete copy constructor and assignment operator to prevent copying.
     * @details The copy constructor is deleted to prevent copying of the I2CDevice instance
     */
    I2CDevice(const I2CDevice &) = delete;

    /**
     * @brief Delete assignment operator to prevent copying.
     * @details This operator is deleted to prevent copying of the I2CDevice instance
     */
    I2CDevice &operator=(const I2CDevice &) = delete;

    /**
     * @brief Write a value to a specific register of the I2C device.
     * @param t_register The register address to write to.
     * @param t_value The value to write to the register.
     * @throws std::runtime_error If the write operation fails.
     */
    const void writeRegister(const uint8_t t_register, const uint8_t t_value) const
    {
        uint8_t data[2] = {t_register, t_value};
        if (write(m_fileDescriptor, data, 2) != 2)
        {
            throw std::runtime_error("Failed to write to the register");
        }
    }
    
    /**
     * @brief Read data from a specific register of the I2C device.
     * @param t_register The register address to read from.
     * @param t_buffer Pointer to a buffer where the read data will be stored.
     * @param t_length The number of bytes to read from the register.
     * @throws std::runtime_error If the read operation fails.
     */
    const void readRegister(const uint8_t t_register, uint8_t *t_buffer, const uint8_t t_length) const
    {
        uint8_t data[1] = {t_register};
        if (write(m_fileDescriptor, data, 1) != 1)
        {
            throw std::runtime_error("Failed to request the register");
        }
        if (read(m_fileDescriptor, t_buffer, t_length) != t_length)
        {
            throw std::runtime_error("Failed to read from the register");
        }
    }
};
} // namespace control_engineering_uni_a

#elif defined(ARDUINO)

    #error "Support for the Arduino platform is not implemented yet. This library is currently only supported for Raspberry Pi 3/4"

#else

    #error "Unsupported platform. This library is only supported on the Raspberry Pi 3/4 (Arduino is not supported yet)."

#endif // defined(__linux__) && (defined(__arm__) || defined(__aarch64__))

#endif // UTILITY_I2CDEVICE_HPP