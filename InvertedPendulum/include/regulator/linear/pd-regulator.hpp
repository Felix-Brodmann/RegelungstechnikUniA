#ifndef REGULATOR_LINEAR_PD_REGULATOR_HPP
#define REGULATOR_LINEAR_PD_REGULATOR_HPP

#include <string>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <map>

#include "regulator/regulator.hpp"
#include "utility/types.hpp"

namespace control_engineering_uni_a
{
    
/**
 * @class PDRegulator
 * @brief Implements a Proportional-Derivative (PD) regulator for control systems.
 * @details The PDRegulator class derives from the Regulator base class and provides a control loop that calculates the control value based on proportional and derivative terms. It supports configurable proportional (Kp) and derivative (Kd) gains, and operates on sensor and actuator interfaces. The regulation loop runs in a separate thread, continuously reading sensor data, computing the control output, and updating the actuator.
 * @note The setpoint must have the same type as the sensor data provided by the sensor.
 * @author Felix Brodmann
 * @date 2025-05-21
 */
class PDRegulator : public Regulator
{
private:
    double m_kp = 1.0; // Proportional gain
    double m_kd = 0.0; // Derivative gain
    SensorData m_setpoint; // Setpoint value
    SensorData m_lastError; // Last error value
    std::chrono::steady_clock::time_point m_lastTime; // Last time the sensor was read
    bool m_firstRun = true; // Flag to check if it's the first run (for the derivative calculation)
    std::thread m_regulationThread; // Regulation thread
    std::map<std::string, bool> m_initilizationList; // Initialization list for the regulator

    /**
     * @brief Get the last error value.
     * @return The last error value.
     */
    SensorData getLastError() const { return m_lastError; }

    /**
     * @brief Get the last time the sensor was read.
     * @return The last time the sensor was read.
     */
    const std::chrono::steady_clock::time_point getLastTime() const { return m_lastTime; }

    /**
     * @brief Check if it's the first run (for the derivative calculation).
     * @return True if it's the first run, false otherwise.
     */
    const bool isFirstRun() const { return m_firstRun; }

    /**
     * @brief Set the last error value.
     * @param t_lastError The last error value to be set.
     */
    void setLastError(SensorData t_lastError)
    {
        m_lastError = t_lastError;
    }

    /**
     * @brief Set the last time the sensor was read.
     * @param t_lastTime The last time to be set.
     */
    void setLastTime(std::chrono::steady_clock::time_point t_lastTime)
    {
        m_lastTime = t_lastTime;
    }

    /**
     * @brief Set the first run flag.
     * @param t_firstRun The first run flag to be set.
     */
    void setFirstRun(bool t_firstRun)
    {
        m_firstRun = t_firstRun;
    }

    /**
     * @brief Set the regulation thread.
     * @param t_thread The thread to be set for regulation.
     */
    void setRegulationThread(std::thread t_thread)
    {
        m_regulationThread = std::move(t_thread);
    }

    /**
     * @brief Set the initialization status for a given variable name.
     * @param t_name The variable name for which the initialization status is to be set.
     * @param t_status The initialization status to be set (true for initialized, false for not initialized).
     */
    void setInitilizationStatus(const std::string& t_name, bool t_status)
    {
        m_initilizationList[t_name] = t_status;
    }

    /**
     * @brief Check if a variable is initialized.
     * @param t_name The variable name to check for initialization status.
     * @return True if the variable is initialized, false otherwise.
     */
    bool isInitilized(const std::string& t_name) const
    {
        auto it = m_initilizationList.find(t_name);
        if (it != m_initilizationList.end())
        {
            return it->second;
        }
        return false; // Default to false if not found
    }

    /**
     * @brief Reset the regulator to its initial state.
     * @details This method resets the last error, last time, and first run flag to their initial values.
     */
    void reset()
    {
        setLastError(SensorData{0}); // Reset last error to zero
        setLastTime(std::chrono::steady_clock::now()); // Reset last time to now
        setFirstRun(true); // Set first run flag to true
    }

    /**
     * @brief Regulation loop that runs in a separate thread.
     * @details This method continuously reads sensor data, computes the control value based on the proportional gain (Kp), derivative gain (Kd), and setpoint, and sends the control value to the actuator.
     * @throws std::runtime_error If the measurement type does not match the setpoint type.
     * @throws std::runtime_error If the data type is not supported for control calculation.
     */
    void regulationLoop()
    {
        // Regulation loop implementation
        while (isRegulating())
        {
            // Read the sensor data
            getSensor().readData();

            // Get the current sensor data
            SensorData measurement = getSensor().getSensorData();

            // Check if the measurement is of the same type as the setpoint
            if (measurement.index() != m_setpoint.index())
            {
                throw std::runtime_error("Measurement type does not match setpoint type");
            }

            // Calculate the control value based on the setpoint and sensor data
            // For now, we only support that the setpoint and the measurement are of the same type
            SensorData control = std::visit([this](auto&& val) -> SensorData
            {
                // Get the type of the setpoint/sensor data
                using T = std::decay_t<decltype(val)>;

                // Check if the type is supported
                if constexpr (std::is_same_v<T, int> || std::is_same_v<T, double> || std::is_same_v<T, Vector2D> || std::is_same_v<T, Vector3D>)
                {
                    // Calculate the error
                    auto error = std::get<T>(getSetpoint()) - val;

                    // Get the current time
                    auto now = std::chrono::steady_clock::now();

                    // Check if it's the first run (for the derivative calculation)
                    if (isFirstRun())
                    {
                        // Initialize the last error and time
                        setLastError(error);
                        setLastTime(now);
                        setFirstRun(false);
                        return error * getKp();
                    }

                    // Calculate the time difference since the last measurement
                    double dt = std::chrono::duration<double>(now - getLastTime()).count();
                    if (dt < 1e-6) dt = 1e-6; // mind. 1 µs zum Schutz

                    // Calculate the derivative error
                    auto derivativeError = (error - std::get<T>(getLastError())) / dt;

                    // Update the last error and time
                    setLastError(error);
                    setLastTime(now);

                    // Calculate the control value
                    return error * getKp() + derivativeError * getKd();
                }
                else if (std::is_same_v<T, IMUData>)
                {
                    throw std::runtime_error("IMUData type is not supported for control calculation, consider using a Filter for IMUData");
                }
                else
                {
                    throw std::runtime_error("Unsupported data type for control calculation");
                }
            }, measurement);

            // Set the control value to the actuator
            getActuator().setControlValue(control);

            // Sleep based on the sampling rate in us
            int sleepTime = 1'000'000 / getSamplingRate();
            std::this_thread::sleep_for(std::chrono::microseconds(sleepTime));
        }

        // Stop the actuator when regulation is stopped
        getActuator().stop();
    }

public:
    /**
     * @brief Complete constructor for the PDRegulator class.
     * @param t_regulatorName The name of the regulator.
     * @param t_sensor A unique pointer to the sensor to be used.
     * @param t_actuator A unique pointer to the actuator to be used.
     * @param t_kp The proportional gain (Kp) for the regulator.
     * @param t_kd The derivative gain (Kd) for the regulator.
     * @param t_setpoint The setpoint value for the regulator.
     */
    PDRegulator(std::string& t_regulatorName, std::unique_ptr<Sensor> t_sensor, std::unique_ptr<Actuator> t_actuator, double t_kp, double t_kd, SensorData t_setpoint)
        : Regulator(t_regulatorName, std::move(t_sensor), std::move(t_actuator))
    {
        setKp(t_kp);
        setKd(t_kd);
        setSetpoint(t_setpoint);
    }

    /**
     * @brief Partial constructor for the PDRegulator class.
     * @details This constructor initializes the PDRegulatoronly partially, allowing the user to set the gains and the setpoint later.
     * @param t_regulatorName The name of the regulator.
     * @param t_sensor A unique pointer to the sensor to be used.
     * @param t_actuator A unique pointer to the actuator to be used.
     */
    PDRegulator(std::string& t_regulatorName, std::unique_ptr<Sensor> t_sensor, std::unique_ptr<Actuator> t_actuator)
        : Regulator(t_regulatorName, std::move(t_sensor), std::move(t_actuator))
    {
        setInitilizationStatus("kp", false);
        setInitilizationStatus("kd", false);
        setInitilizationStatus("setpoint", false);
    }

    /**
     * @brief Destructor for the PDRegulator class.
     */
    ~PDRegulator()
    {
        stopRegulation();
    }

    /**
     * @brief Delete copy constructor and assignment operator to prevent copying.
     * @details The copy constructor is deleted to prevent copying of the PDRegulator instance, as it contains a thread that should not be copied.
     */
    PDRegulator(const PDRegulator&) = delete;

    /**
     * @brief Delete assignment operator to prevent copying.
     * @details This operator is deleted to prevent copying of the PDRegulator instance, as it contains a thread that should not be copied.
     */
    PDRegulator& operator=(const PDRegulator&) = delete;

    /**
     * @brief Set the proportional gain (Kp) for the regulator.
     * @param t_kp The proportional gain (Kp) to be set.
     */
    void setKp(double t_kp)
    {
        setInitilizationStatus("kp", true);
        m_kp = t_kp;
    }

    /**
     * @brief Get the proportional gain (Kp).
     * @return The proportional gain (Kp).
     */
    const double getKp() const
    {
        if (!isInitilized("kp"))
        {
            throw std::runtime_error("Kp is not initialized");
        }
        return m_kp;
    }

    /**
     * @brief Set the derivative gain (Kd) for the regulator.
     * @param t_kd The derivative gain (Kd) to be set.
     */
    void setKd(double t_kd)
    {
        setInitilizationStatus("kd", true);
        m_kd = t_kd;
    }

    /**
     * @brief Get the derivative gain (Kd).
     * @return The derivative gain (Kd).
     */
    double getKd() const
    {
        if (!isInitilized("kd"))
        {
            throw std::runtime_error("Kd is not initialized");
        }
        return m_kd;
    }

    /**
     * @brief Set the setpoint for the regulator.
     * @param t_setpoint The setpoint value to be set.
     */
    void setSetpoint(SensorData t_setpoint)
    {
        setInitilizationStatus("setpoint", true);
        m_setpoint = t_setpoint;
    }

    /**
     * @brief Get the proportional gain (Kp).
     * @return The proportional gain (Kp).
     */
    SensorData getSetpoint() const
    {
        if (!isInitilized("setpoint"))
        {
            throw std::runtime_error("Setpoint is not initialized");
        }
        return m_setpoint;
    }

    /**
     * @brief Start the regulation process.
     * @details This method starts the regulation loop in a separate thread.
     * @throws std::runtime_error if regulation is already in progress.
     */
    void startRegulation() override
    {
        if (isRegulating())
        {
            throw std::runtime_error("Regulation is already in progress");
        }
        setIsRegulating(true);
        
        // Start the regulation loop in a separate thread
        setRegulationThread(std::thread([this]() { regulationLoop(); }));
    }

    /**
     * @brief Stop the regulation process.
     * @details This method stops the regulation loop and sets the actuator to stop.
     */
    void stopRegulation() override
    {
        setIsRegulating(false);

        if (m_regulationThread.joinable())
        {
            m_regulationThread.join();

            // Reset the regulator state
            reset();
        }
    }
};

} // namespace control_engineering_uni_a

#endif // REGULATOR_LINEAR_PD_REGULATOR_HPP