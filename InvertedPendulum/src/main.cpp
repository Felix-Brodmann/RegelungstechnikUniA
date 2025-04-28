#include "../include/lsm6ds3.hpp"
#include "../include/ev3Motor.hpp"
#include "../include/complementaryFilter.hpp"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>
#include <string.h>


using namespace RegelungstechnikUniA;


// std::atomic == thread safe


// General constants
const int EV3_ENA_PIN = 18;
const int EV3_IN1_PIN = 15;
const int EV3_IN2_PIN = 14;
const int TACHO_A_PIN = 22;
const int TACHO_B_PIN = 27;


// Regulator constants
const int SAMPLE_RATE = 1000; // [Hz]
const int TIME_DELTA = (1 / SAMPLE_RATE) * 1000000; // [us]


// Generall global variables
std::atomic<bool> globalRunning = true;
bool globalDebugMode = false;
bool globalGuiEnabled = true;
float globalKp = -24.0f;
float globalKd = -0.8f;


// Regulator global variables
std::atomic<float> globalPhi = 0.0f; // [Deg]
std::atomic<float> globalPhiRef = 0.0f; // [Deg]
std::chrono::time_point<std::chrono::steady_clock> globalTimeOfTheLastIteration; // [us]


/**
 * @brief Regulation function
 * @details The regulation function calculates the motor speed based on the current angle, angular velocity and the desired angle.
 * @param t_phi The current angle of the pendulum in degrees
 * @param t_dPhi The current angular velocity of the pendulum in degrees per second
 * @param t_omega The current angular velocity of the motor in degrees per second
 * @return The motor speed in percent
 */
int Regulation(const float t_phi, const float t_dPhi, const float t_omega);


/**
 * @brief Get the position of the pendulum for the GUI
 * @details The function calculates the position of the pendulum based on the angle.
 * @param t_phi The angle of the pendulum in degrees
 * @return The position of the pendulum in pixels
 * @note The origin of the pendulum is in the middle of the screen
 */
sf::Vector2f getPendelPosition(const float t_phi);


/**
 * @brief Handle the GUI
 * @details The function creates a window and draws the pendulum and the motor.
 * @note The function runs in a separate thread
 */
void handleGUI();


int main(int argc, char *argv[])
{

    // Check if the program is run with console arguments
    if (argc == 1)
    {
        std::cout << "No console arguments provided. Switching to default values." << std::endl;
        std::cout << "Use --help for more information." << std::endl << std::endl;
    }


    // Go through the console arguments
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--help") == 0)
        {
            std::cout << std::endl << "Usage: " << argv[0] << " [options]" << std::endl << std::endl;
            std::cout << "Options:" << std::endl << std::endl;
            std::cout << "\t --help\t\tShow this help message" << std::endl;
            std::cout << "\t --debug\tEnable debug mode" << std::endl;
            std::cout << "\t --no-gui\tDisable the GUI" << std::endl;
            std::cout << "\t --kp <value>\tSet the proportional gain" << std::endl;
            std::cout << "\t --kd <value>\tSet the derivative gain" << std::endl << std::endl;
            return 0;
        }
        else if (strcmp(argv[i], "--debug") == 0)
        {
            std::cout << std::endl << "Debug mode enabled" << std::endl;
            globalDebugMode = true;
        }
        else if (strcmp(argv[i], "--no-gui") == 0)
        {
            std::cout << std::endl << "GUI disabled" << std::endl;
            globalGuiEnabled = false;
        }
        else if (strcmp(argv[i], "--kp") == 0)
        {
            globalKp = atof(argv[++i]);
            std::cout << std::endl << "Proportional gain set to " << globalKp << std::endl;
        }
        else if (strcmp(argv[i], "--kd") == 0)
        {
            globalKd = atof(argv[++i]);
            std::cout << std::endl << "Derivative gain set to " << globalKd << std::endl;
        }
        else
        {
            std::cerr << "Unknown argument: " << argv[i] << std::endl;
            std::cerr << "Use --help for more information." << std::endl;
            return 1;
        }
    }

    try
    {

        // Initialize the motor and the LSM6DS3 sensor
        if (globalDebugMode)
            std::cout << std::endl << "Initializing the EV3-motor and the LSM6DS3 sensor..." << std::endl;
        EV3Motor motor(EV3_ENA_PIN, EV3_IN1_PIN, EV3_IN2_PIN, 5000, TACHO_A_PIN, TACHO_B_PIN);
        LSM6DS3 lsm6ds3("/dev/i2c-1", 0x6A);
        if (!lsm6ds3.amITalkingToTheSensor())
        {
            throw std::runtime_error("LSM6DS3 sensor not found");
        }
        if (globalDebugMode)
            std::cout << "Initialization complete" << std::endl << std::endl;


        // Start the GUI thread if enabled
        std::thread guiThread;
        if (globalGuiEnabled)
        {
            if (globalDebugMode)
                std::cout << "Starting the GUI..." << std::endl;
            guiThread = std::thread(handleGUI);
            if (globalDebugMode)
                std::cout << "GUI started" << std::endl << std::endl;
        }


        // Configure the LSM6DS3 sensor
        if (globalDebugMode)
            std::cout << "Configuring the LSM6DS3 sensor..." << std::endl;
        lsm6ds3.configureAccelerometer( LSM6DS3::AccelerometerOutputDataRate::HIGH_PERFORMANCE_1660Hz,
                                        LSM6DS3::AccelerometerFullScale::FS_2G,
                                        LSM6DS3::AccelerometerFilterBandwidth::BW_400Hz);
        lsm6ds3.configureGyroscope( LSM6DS3::GyroscopeOutputDataRate::HIGH_PERFORMANCE_1660Hz,
                                    LSM6DS3::GyroscopeFullScale::FS_500DPS);
        if (globalDebugMode)
            std::cout << "Configuration complete" << std::endl;


        // Calibrate the gyroscope
        if (globalDebugMode)
            std::cout << std::endl << "Calibrating the gyroscope..." << std::endl;
        lsm6ds3.calibrateGyroscope(5);
        if (globalDebugMode)
            std::cout << "Calibration complete" << std::endl;


        // Set the precision of the output for debug mode
        if (globalDebugMode)
        {
            std::cout << std::fixed << std::setprecision(3) << std::showpos;
        }


        // Define the variables for the main loop
        std::vector<double> gyro;
        std::vector<double> accel;
        double unfilteredGyroscopeZ = 0.0;
        double unfilteredAccelerometerX = 0.0;
        double unfilteredAccelerometerY = 0.0;
        int motorSpeed = 0;
        float phi = 0;
        float dPhi = 0;

        // Initialize the complementary filter
        if (globalDebugMode)
            std::cout << "Initializing the complementary filter..." << std::endl;
        RegelungstechnikUniA::ComplementaryFilter filter(0.999);
        if (globalDebugMode)
            std::cout << "Complementary filter initialized" << std::endl;
        usleep(2500);


        // Let the pendulum find its current position based on the gravity, this taks about 5 seconds
        for (int i = 0; i < (SAMPLE_RATE * 5); i++)
        {


            // Read the values from the sensor
            gyro = lsm6ds3.readGyroscope(LSM6DS3::GyroscopeUnit::IN_DEG_PER_S);
            accel = lsm6ds3.readAccelerometer(LSM6DS3::AccelerometerUnit::IN_G);
            unfilteredGyroscopeZ = gyro[Z_AXIS];
            unfilteredAccelerometerX = accel[X_AXIS];
            unfilteredAccelerometerY = accel[Y_AXIS];


            // Apply the complementary filter to the unfiltered values
            filter.update(unfilteredGyroscopeZ, unfilteredAccelerometerX, unfilteredAccelerometerY);


            // Get the filtered phi value
            phi = filter.getAngle();


            // Get the unfiltered dPhi value
            dPhi = gyro[Z_AXIS];


            // Set the globalPhi value for the GUI
            globalPhi = phi;


            // Sleep based on the sample rate
            usleep(TIME_DELTA);
        }


        // Set the iteration time to the current time
        globalTimeOfTheLastIteration = std::chrono::steady_clock::now();


        // Define a debug counter, so that the output is not too fast (don't output every iteration)
        int debugCounter = 0;

        // Start the main loop
        while (globalRunning)
        {


            // Get the current motor speed
            double omega = motor.getOmega();


            // Read the values from the sensor
            gyro = lsm6ds3.readGyroscope(LSM6DS3::GyroscopeUnit::IN_DEG_PER_S);
            accel = lsm6ds3.readAccelerometer(LSM6DS3::AccelerometerUnit::IN_G);
            unfilteredGyroscopeZ = gyro[Z_AXIS];
            unfilteredAccelerometerX = accel[X_AXIS];
            unfilteredAccelerometerY = accel[Y_AXIS];


            // Apply the complementary filter to the unfiltered values
            filter.update(gyro[Z_AXIS], accel[X_AXIS], accel[Y_AXIS]);


            // Get the filtered phi value
            phi = filter.getAngle();


            // Get the unfiltered dPhi value
            dPhi = gyro[Z_AXIS];


            // Set the globalPhi value for the GUI
            globalPhi = phi;


            // Set the motor speed based on the regulation
            motorSpeed = Regulation(phi, dPhi, omega);
            motor.setSpeed(motorSpeed);


            // Print the values to the console
            if (debugCounter % 100 == 0 && globalDebugMode)
            {
                std::cout << std::endl << "┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━┓" << std::endl;
                std::cout << "┃ Unfiltered Gyroscope Z\t┃ " << std::setfill(' ') << std::setw(8) << unfilteredGyroscopeZ << " Deg/s ┃" << std::endl;
                std::cout << "┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━┫" << std::endl;
                std::cout << "┃ Unfiltered Accelerometer X\t┃ " << std::setfill(' ') << std::setw(8) << unfilteredAccelerometerX << " G     ┃" << std::endl;
                std::cout << "┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━┫" << std::endl;
                std::cout << "┃ Unfiltered Accelerometer Y\t┃ " << std::setfill(' ') << std::setw(8) << unfilteredAccelerometerY << " G     ┃" << std::endl;
                std::cout << "┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━┫" << std::endl;
                std::cout << "┃ Unfiltered Accelerometer Z\t┃ " << std::setfill(' ') << std::setw(8) << accel[Z_AXIS] << " G     ┃" << std::endl;
                std::cout << "┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━┫" << std::endl;
                std::cout << "┃ Phi\t\t\t\t┃ " << std::setfill(' ') << std::setw(8) << phi << " Deg   ┃" << std::endl;
                std::cout << "┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━┫" << std::endl;
                std::cout << "┃ dPhi\t\t\t\t┃ " << std::setfill(' ') << std::setw(8) << dPhi << " Deg/s ┃" << std::endl;
                std::cout << "┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━┫" << std::endl;
                std::cout << "┃ Omega\t\t\t\t┃ " << std::setfill(' ') << std::setw(8) << omega << " U     ┃" << std::endl;
                double doubleMotorSpeed = static_cast<double>(motorSpeed);
                std::cout << "┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━┫" << std::endl;
                std::cout << "┃ Motor Speed\t\t\t┃ " << std::setfill(' ') << std::setw(8) << doubleMotorSpeed << " %     ┃" << std::endl;
                std::cout << "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━━━┛" << std::endl;
                debugCounter = 0;
            }


            // Count the debug counter
            debugCounter++;


            // Sleep based on the sample rate
            usleep(TIME_DELTA);
        }


        // Stop the motor and join the GUI thread
        motor.setSpeed(0);
        guiThread.join();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}



/*************************
 ******* Regulator *******
 *************************/

int Regulation(const float t_phi, const float t_dPhi, const float t_omega)
{


    // Define the variables
    std::chrono::time_point<std::chrono::steady_clock> currentTime;
    std::chrono::duration<double> deltaTime;
    float error = 0.0f;
    int u = 0;


    // Set the reference angle based on the current angle
    if (t_phi < 1.0)
    {
        globalPhiRef = 3.0f;
    }
    else if (t_phi > 3.0)
    {
        globalPhiRef = 1.0f;
    }
    else
    {
        globalPhiRef = 2.0f;
    }


    // Safety-check (in case the pendulum falls down)
    if (t_phi > 20 || t_phi < -20)
    {
        return 0;
    }


    // Calculate the time difference from the last iteration
    currentTime = std::chrono::steady_clock::now();
    deltaTime = currentTime - globalTimeOfTheLastIteration;
    globalTimeOfTheLastIteration = currentTime;


    // Calculate the error
    error = globalPhiRef - t_phi;


    // Calculate the motor speed based on the PID regulator
    u = static_cast<int>(globalKp * error - globalKd * t_dPhi);


    // The motor operates in the range of -100 to -10 and 10 to 100
    if ((u >= 0 && u <= 10) || (u < 0 && u >= -10))
    {
        u = 0;
    }
    else if (u > 100)
    {
        u = 100;
    }
    else if (u < -100)
    {
        u = -100;
    }


    // Return the motor speed
    return u;
}



/*************************
 ********** GUI **********
 *************************/

// SFML Constants
const float LENGTH = 150.0f;
const float ORIGIN_X = 400.0f;
const float ORIGIN_Y = 300.0f;
const float CIRCLE_RADIUS = 10.0f;
const float RECTANGLE_WIDTH = 5.0f;
const int FRAME_RATE = 30;


sf::Vector2f getPendelPosition(const float t_phi)
{


    // Calculate the position of the pendulum on the screen
    float x = ORIGIN_X - LENGTH * sin(t_phi * M_PI / 180);
    float y = ORIGIN_Y - LENGTH * cos(t_phi * M_PI / 180);


    // Return the position of the pendulum
    return sf::Vector2f(x, y);
}


void handleGUI()
{


    // Create the window and set the frame rate
    sf::RenderWindow window(sf::VideoMode(800, 500), "Pendel");
    window.setFramerateLimit(FRAME_RATE);


    // The motor shape
    sf::CircleShape motorShape(CIRCLE_RADIUS);
    motorShape.setFillColor(sf::Color::Red);
    motorShape.setPosition(ORIGIN_X - CIRCLE_RADIUS, ORIGIN_Y - CIRCLE_RADIUS);
    motorShape.setRotation(180);


    // The actual arm shape
    sf::RectangleShape armShape(sf::Vector2f(RECTANGLE_WIDTH, LENGTH));
    armShape.setFillColor(sf::Color::Blue);
    armShape.setPosition(ORIGIN_X - RECTANGLE_WIDTH / 2, ORIGIN_Y);


    // The target arm position
    sf::RectangleShape targetArmPosition(sf::Vector2f(RECTANGLE_WIDTH, LENGTH));
    targetArmPosition.setFillColor(sf::Color::Green);
    targetArmPosition.setPosition(ORIGIN_X - RECTANGLE_WIDTH / 2, ORIGIN_Y);


    // The text to display the angle
    sf::Font font;
    if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSerif-Bold.ttf"))
    {
        std::cerr << "Failed to load font" << std::endl;
        return;
    }
    sf::Text angleText;
    angleText.setFont(font);
    angleText.setCharacterSize(20);
    angleText.setFillColor(sf::Color::White);
    angleText.setPosition(10, 10);
    angleText.setString("Angle: 0");


    // The GUI loop
    while (window.isOpen())
    {


        // Check if the window was closed
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                globalRunning = false;
                window.close();
            }
        }


        // Update the angle text
        angleText.setString("Angle: " + std::to_string(globalPhi));


        // Update the motor position
        sf::Vector2f pendelPosition = getPendelPosition(globalPhi);
        motorShape.setPosition(pendelPosition.x + RECTANGLE_WIDTH, pendelPosition.y + CIRCLE_RADIUS);


        // Update the arm position
        armShape.setPosition(ORIGIN_X - RECTANGLE_WIDTH / 2, ORIGIN_Y);
        armShape.setRotation(180 - globalPhi);


        // Update the target arm position
        targetArmPosition.setPosition(ORIGIN_X - RECTANGLE_WIDTH / 2, ORIGIN_Y);
        targetArmPosition.setRotation(180 - globalPhiRef);


        // Draw the window
        window.clear();
        window.draw(angleText);
        window.draw(targetArmPosition);
        window.draw(armShape);
        window.draw(motorShape);
        window.display();
    }
}