#include <iostream>
#include <memory>
#include <sensor/imu/lsm6ds3-builder.hpp>
#include <sensor/imu/lsm6ds3.hpp>
#include <actuator/motor/ev3-motor-builder.hpp>
#include <actuator/motor/ev3-motor.hpp>
#include <regulator/linear/pid-regulator-builder.hpp>
#include <regulator/linear/pid-regulator.hpp>
#include <utility/filter/complementary-filter.hpp>
#include <utility/i2c-device.hpp>
#include <utility/pwm-device.hpp>
#include <utility/gpio.hpp>
#include <utility/output-stream/console-output-stream.hpp>

#define EV3_MOTOR_PWM_PIN 18 // GPIO pin for PWM output to the EV3 motor
#define EV3_MOTOR_FORWARD_PIN 27 // GPIO pin for forward direction control
#define EV3_MOTOR_BACKWARD_PIN 17 // GPIO pin for backward direction control
#define EV3_MOTOR_TACHO_A_PIN 15 // GPIO pin for tacho A feedback
#define EV3_MOTOR_TACHO_B_PIN 14 // GPIO pin for tacho B feedback

int main()
{
    try
    {
        // Output stream for logging
        std::shared_ptr<control_engineering_uni_a::OutputStream> outputStream = std::make_shared<control_engineering_uni_a::ConsoleOutputStream>();

        /**********************
         ******* Sensor *******
         **********************/

         std::cout << "Creating sensor..." << std::endl;

        // Create an I2C device instance for the LSM6DS3 IMU sensor
        std::unique_ptr<control_engineering_uni_a::I2CDevice> i2cDevice = std::make_unique<control_engineering_uni_a::I2CDevice>("/dev/i2c-1", 0x6A);

        // Create an LSM6DS3 IMU sensor using the builder pattern
        std::unique_ptr<control_engineering_uni_a::LSM6DS3> imuSensor =
            control_engineering_uni_a::LSM6DS3Builder::create()
            .withI2CDevice(std::move(i2cDevice))
            .withAccelerometerConfig(control_engineering_uni_a::LSM6DS3::AccelerometerOutputDataRate::HIGH_PERFORMANCE_1660Hz,
                                        control_engineering_uni_a::LSM6DS3::AccelerometerFullScale::FS_2G,
                                        control_engineering_uni_a::LSM6DS3::AccelerometerFilterBandwidth::BW_200Hz)
            .withAccelerometerUnit(control_engineering_uni_a::AccelerometerUnit::IN_M_PER_S2)
            .withGyroscopeConfig(control_engineering_uni_a::LSM6DS3::GyroscopeOutputDataRate::HIGH_PERFORMANCE_1660Hz,
                                    control_engineering_uni_a::LSM6DS3::GyroscopeFullScale::FS_500DPS)
            .withGyroscopeUnit(control_engineering_uni_a::GyroscopeUnit::IN_DEG_PER_S)
            .withTemperatureUnit(control_engineering_uni_a::TemperatureUnit::IN_CELSIUS)
            .calibrateOnTheFly()
            .build();

        // Create a filter for the IMU data
        std::unique_ptr<control_engineering_uni_a::ComplementaryFilter> imuFilter =
            std::make_unique<control_engineering_uni_a::ComplementaryFilter>(0.98);

        imuFilter->getOutputStream(outputStream); // Set the output stream for the filter

        imuSensor->addFilter(std::move(imuFilter)); // Add the filter to the IMU sensor
        imuSensor->setOutputStream(outputStream); // Set the output stream for the sensor

        std::cout << "Sensor created successfully." << std::endl;

        /**********************
         ****** Actuator ******
         **********************/

        std::cout << "Creating actuator..." << std::endl;

        // Initialize the GPIO system
        control_engineering_uni_a::initializeGpio();

        // Create a PWM device instance for the EV3 motor
        std::unique_ptr<control_engineering_uni_a::PwmDevice> pwmDevice =
            std::make_unique<control_engineering_uni_a::PwmDevice>(EV3_MOTOR_PWM_PIN, 1000); // 1 kHz PWM frequency

        // Create an EV3 motor using the builder pattern
        std::unique_ptr<control_engineering_uni_a::EV3Motor> motor =
            control_engineering_uni_a::EV3MotorBuilder::create(std::move(pwmDevice))
            .setForwardPin(EV3_MOTOR_FORWARD_PIN)
            .setBackwardPin(EV3_MOTOR_BACKWARD_PIN)
            .setTachoAPin(EV3_MOTOR_TACHO_A_PIN)
            .setTachoBPin(EV3_MOTOR_TACHO_B_PIN)
            .build();

        motor->setOutputStream(outputStream); // Set the output stream for the actuator

        std::cout << "Actuator created successfully." << std::endl;

        /**********************
         ****** Regulator *****
         **********************/

        std::cout << "Creating PID regulator..." << std::endl;

        // Create a PID regulator for the motor control
        std::unique_ptr<control_engineering_uni_a::PIDRegulator> pidRegulator =
            control_engineering_uni_a::PIDRegulatorBuilder::create()
            .setName("Lego Inverted Pendulum PID Regulator")
            .setSensor(std::move(imuSensor))
            .setActuator(std::move(motor))
            .setKp(38.0) // Proportional gain
            .setKi(0.0) // Integral gain
            .setKd(0.9) // Derivative gain
            .setSetpoint(control_engineering_uni_a::SensorData(0.0)) // Setpoint value
            .setOutputMin(control_engineering_uni_a::SensorData(-100.0)) // Minimum output value
            .setOutputMax(control_engineering_uni_a::SensorData(100.0)) // Maximum output value
            .setIntegralMin(control_engineering_uni_a::SensorData(-4.0)) // Minimum integral error
            .setIntegralMax(control_engineering_uni_a::SensorData(4.0)) // Maximum integral error
            .build();

        pidRegulator->setSetpointRampingFixrate(0.5); // Setpoint ramping fix rate
        pidRegulator->setSamplingRate(100); // Set the sampling rate to 100 Hz
        pidRegulator->setOutputStream(outputStream); // Set the output stream for logging

        std::cout << "PID regulator created successfully." << std::endl;

        // Start the PID regulator
        pidRegulator->startRegulation();

        // Wait for an input to stop the program
        std::cout << "Press Enter to stop the program..." << std::endl;
        std::cin.get();

        // Stop the PID regulator
        pidRegulator->stopRegulation();
    }
    catch (const std::exception &e)
    {
        // Handle exceptions
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}