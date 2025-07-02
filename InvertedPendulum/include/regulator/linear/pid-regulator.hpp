#ifndef REGULATOR_LINEAR_PID_REGULATOR_HPP
#define REGULATOR_LINEAR_PID_REGULATOR_HPP

#include <string>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <map>
#include <mutex>

#include "regulator/regulator.hpp"
#include "utility/types.hpp"

namespace control_engineering_uni_a
{

/**
 * @class PIDRegulator
 * @brief Implements a Proportional-Integral-Derivative (PID) regulator for control systems.
 * @details The PIDRegulator class derives from the Regulator base class and provides a control loop that calculates the control value based on proportional, integral, and derivative terms. It supports configurable proportional (Kp), integral (Ki), and derivative (Kd) gains, and operates on sensor and actuator interfaces. The regulation loop runs in a separate thread, continuously reading sensor data, computing the control output, and updating the actuator.
 * @note The setpoint must have the same type as the sensor data provided by the sensor.
 * @author Felix Brodmann
 * @date 2025-05-21
 */
class PIDRegulator : public Regulator
{
private:
    double m_kp = 1.0; // Proportional gain
    mutable std::mutex m_kpMutex; // Mutex for Kp to ensure thread safety
    double m_ki = 0.0; // Integral gain
    mutable std::mutex m_kiMutex; // Mutex for Ki to ensure thread safety
    double m_kd = 0.0; // Derivative gain
    mutable std::mutex m_kdMutex; // Mutex for Kd to ensure thread safety
    SensorData m_setpoint; // Setpoint value
    mutable std::mutex m_setpointMutex; // Mutex for setpoint to ensure thread safety
    double m_setpointRampingFixrate = 0.0; // Setpoint ramping fix rate
    mutable std::mutex m_setpointRampingFixrateMutex; // Mutex for setpoint
    SensorData m_outputMin; // Minimum output value
    mutable std::mutex m_outputMinMutex; // Mutex for outputMin to ensure thread safety
    SensorData m_outputMax; // Maximum output value
    mutable std::mutex m_outputMaxMutex; // Mutex for outputMax to ensure thread safety
    SensorData m_integralMin; // Minimum integral value (for anti-windup)
    mutable std::mutex m_integralMinMutex; // Mutex for integralMin to ensure thread safety
    SensorData m_integralMax; // Maximum integral value (for anti-windup)
    mutable std::mutex m_integralMaxMutex; // Mutex for integralMax to ensure thread safety

    SensorData m_lastError; // Last error value
    SensorData m_rampedSetpoint; // Ramped setpoint value
    SensorData m_integral; // Integral value
    std::chrono::steady_clock::time_point m_lastTime; // Last time the sensor was read
    bool m_firstRun = true; // Flag to check if it's the first run (for the derivative calculation)
    std::thread m_regulationThread; // Regulation thread
    std::map<std::string, bool> m_initializationList; // Initialization list for the regulator

    /**
     * @brief Get the last error value.
     * @return The last error value.
     */
    SensorData getLastError() const { return m_lastError; }

    /**
     * @brief Set the last error value.
     * @param t_lastError The last error value to be set.
     */
    void setLastError(SensorData t_lastError) { m_lastError = t_lastError; }

    /**
     * @brief Get the ramped setpoint value.
     * @return The ramped setpoint value.
     */
    SensorData getRampedSetpoint() const { return m_rampedSetpoint; }

    /**
     * @brief Set the ramped setpoint value.
     * @param t_rampedSetpoint The ramped setpoint value to be set.
     */
    void setRampedSetpoint(SensorData t_rampedSetpoint) { m_rampedSetpoint = t_rampedSetpoint; }

    /**
     * @brief Get the integral value.
     * @return The integral value.
     */
    SensorData getIntegral() const { return m_integral; }

    /**
     * @brief Set the integral value.
     * @param t_integral The integral value to be set.
     */
    void setIntegral(SensorData t_integral) { m_integral = t_integral; }

    /**
     * @brief Get the last time the sensor was read.
     * @return The last time the sensor was read.
     */
    std::chrono::steady_clock::time_point getLastTime() const { return m_lastTime; }

    /**
     * @brief Set the last time the sensor was read.
     * @param t_lastTime The last time to be set.
     */
    void setLastTime(std::chrono::steady_clock::time_point t_lastTime) { m_lastTime = t_lastTime; }

    /**
     * @brief Check if it's the first run (for the derivative calculation).
     * @return True if it's the first run, false otherwise.
     */
    bool isFirstRun() const { return m_firstRun; }

    /**
     * @brief Set the first run flag.
     * @param t_firstRun The first run flag to be set.
     */
    void setFirstRun(bool t_firstRun) { m_firstRun = t_firstRun; }

    /**
     * @brief Get the regulation thread.
     * @return The regulation thread.
     */
    std::thread& getRegulationThread() { return m_regulationThread; }

    /**
     * @brief Set the regulation thread.
     * @param t_thread The thread to be set for regulation.
     */
    void setRegulationThread(std::thread t_thread) { m_regulationThread = std::move(t_thread);}

    /**
     * @brief Check if a variable is initialized.
     * @param t_name The variable name to check for initialization status.
     * @return True if the variable is initialized, false otherwise.
     */
    bool isInitialized(const std::string& t_name) const
    {
        auto it = m_initializationList.find(t_name);
        if (it != m_initializationList.end())
        {
            return it->second;
        }
        return false; // Default to false if not found
    }

    /**
     * @brief Set the initialization status for a given variable name.
     * @param t_name The variable name for which the initialization status is to be set.
     * @param t_status The initialization status to be set (true for initialized, false for not initialized).
     */
    void setInitializationStatus(const std::string& t_name, bool t_status) { m_initializationList[t_name] = t_status; }

    /**
     * @brief Reset the regulator to its initial state.
     * @details This method resets the last error, integral, and last time to their initial values. It also sets the first run flag to true, indicating that the next regulation loop will be the first run. This method is useful for reinitializing the regulator without creating a new instance.
     */
    void reset()
    {
        setLastError(SensorData{0}); // Reset last error to zero
        setIntegral(SensorData{0}); // Reset integral to zero
        setLastTime(std::chrono::steady_clock::now()); // Reset last time to now
        setFirstRun(true); // Set first run flag to true
    }

    /**
     * @brief Regulation loop that runs in a separate thread.
     * @details This method implements the regulation loop that continuously reads sensor data, computes the control output based on the PID algorithm, and updates the actuator. It runs in a separate thread and stops when regulation is no longer active.
     * @throws std::runtime_error if the measurement type does not match the setpoint type.
     * @throws std::runtime_error if the data type is not supported for control calculation.
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
                    // Get the current time
                    auto now = std::chrono::steady_clock::now();

                    // Check if it's the first run (for the derivative and integral calculation)
                    if (isFirstRun())
                    {
                        // Initialize the last error and time and set the integral to zero
                        setLastError(std::get<T>(getSetpoint()) - val);
                        setRampedSetpoint(getSetpoint());
                        setIntegral(T{0});
                        setLastTime(now);
                        setFirstRun(false);
                        return std::clamp<T>(std::get<T>(getRampedSetpoint()) - val, 
                                             std::get<T>(getOutputMin()), 
                                             std::get<T>(getOutputMax()));
                    }

                    // Calculate the time difference since the last measurement
                    double dt = std::chrono::duration<double>(now - getLastTime()).count();
                    if (dt < 1e-6) dt = 1e-6; // at least 1 µs for protection

                    // If setpoint ramping is enabled, apply the ramping fix rate
                    if (getSetpointRampingFixrate() > 0.0)
                    {
                        if (val < getRampedSetpoint())
                        {
                            // Increase the ramped setpoint
                            setRampedSetpoint(std::get<T>(getRampedSetpoint()) + getSetpointRampingFixrate() * dt);
                        }
                        else if (val > getRampedSetpoint())
                        {
                            // Decrease the ramped setpoint
                            setRampedSetpoint(std::get<T>(getRampedSetpoint()) - getSetpointRampingFixrate() * dt);
                        }
                    }

                    // Calculate the error
                    auto error = std::get<T>(getRampedSetpoint()) - val;
                    getOutputStream().write(getRegulatorName(), "Calculated error: " + toString(error));

                    // Calculate the integral error
                    auto integralError = std::get<T>(getIntegral()) + error * dt;

                    // Apply anti-windup limits to the integral error
                    integralError = std::clamp<T>(integralError, std::get<T>(getIntegralMin()), std::get<T>(getIntegralMax()));
                    getOutputStream().write(getRegulatorName(), "Calculated integral error: " + toString(integralError));

                    // Calculate the derivative error
                    auto derivativeError = (error - std::get<T>(getLastError())) / dt;
                    getOutputStream().write(getRegulatorName(), "Calculated derivative error: " + toString(derivativeError));

                    // Update the last error, integral and time
                    setLastError(error);
                    setIntegral(integralError);
                    setLastTime(now);

                    // Calculate the control value
                    auto controlValue = std::clamp<T>(  error * getKp() + integralError * getKi() + derivativeError * getKd(),
                                                        std::get<T>(getOutputMin()), 
                                                        std::get<T>(getOutputMax()));
                    getOutputStream().write(getRegulatorName(), "Calculated control value: " + toString(controlValue));
                    return controlValue;
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
     * @brief Complete constructor for the PIDRegulator class.
     * @param t_regulatorName The name of the regulator.
     * @param t_sensor A unique pointer to the sensor to be used.
     * @param t_actuator A unique pointer to the actuator to be used.
     * @param t_kp The proportional gain (Kp) for the regulator.
     * @param t_ki The integral gain (Ki) for the regulator.
     * @param t_kd The derivative gain (Kd) for the regulator.
     * @param t_setpoint The setpoint value for the regulator.
     * @param t_outputMin The minimum output value for the regulator.
     * @param t_outputMax The maximum output value for the regulator.
     * @param t_integralMin The minimum integral value (for anti-windup).
     * @param t_integralMax The maximum integral value (for anti-windup).
     */
    PIDRegulator(std::string& t_regulatorName, std::unique_ptr<Sensor> t_sensor, std::unique_ptr<Actuator> t_actuator, double t_kp, double t_ki, double t_kd, SensorData t_setpoint, SensorData t_outputMin, SensorData t_outputMax, SensorData t_integralMin, SensorData t_integralMax)
        : Regulator(t_regulatorName, std::move(t_sensor), std::move(t_actuator))
    {
        setKp(t_kp);
        setKi(t_ki);
        setKd(t_kd);
        setSetpoint(t_setpoint);
        setOutputLimits(t_outputMin, t_outputMax);
        setIntegralLimits(t_integralMin, t_integralMax);
    }

    /**
     * @brief Partial constructor for the PIDRegulator class.
     * @details This constructor initializes the PIDRegulator only partially, allowing the user to set the gains, the setpoint and the integral limits later.
     * @param t_regulatorName The name of the regulator.
     * @param t_sensor A unique pointer to the sensor to be used.
     * @param t_actuator A unique pointer to the actuator to be used.
     */
    PIDRegulator(std::string& t_regulatorName, std::unique_ptr<Sensor> t_sensor, std::unique_ptr<Actuator> t_actuator)
        : Regulator(t_regulatorName, std::move(t_sensor), std::move(t_actuator))
    {
        setInitializationStatus("kp", false);
        setInitializationStatus("ki", false);
        setInitializationStatus("kd", false);
        setInitializationStatus("setpoint", false);
        setInitializationStatus("outputMin", false);
        setInitializationStatus("outputMax", false);
        setInitializationStatus("integralMin", false);
        setInitializationStatus("integralMax", false);
    }

    /**
     * @brief Destructor for the PIDRegulator class.
     */
    ~PIDRegulator()
    {
        stopRegulation();
    }

    /**
     * @brief Delete copy constructor and assignment operator to prevent copying.
     * @details The copy constructor is deleted to prevent copying of the PIDRegulator instance, as it contains a thread that should not be copied.
     */
    PIDRegulator(const PIDRegulator&) = delete;

    /**
     * @brief Delete assignment operator to prevent copying.
     * @details This operator is deleted to prevent copying of the PIDRegulator instance, as it contains a thread that should not be copied.
     */
    PIDRegulator& operator=(const PIDRegulator&) = delete;

    /**
     * @brief Get the proportional gain (Kp).
     * @return The proportional gain (Kp).
     */
    const double getKp() const
    {
        std::scoped_lock lock(m_kpMutex);
        if (!isInitialized("kp"))
        {
            throw std::runtime_error("Kp is not initialized");
        }
        return m_kp;
    }

    /**
     * @brief Set the proportional gain (Kp) for the regulator.
     * @param t_kp The proportional gain (Kp) to be set.
     */
    void setKp(double t_kp)
    {
        std::scoped_lock lock(m_kpMutex);
        setInitializationStatus("kp", true);
        m_kp = t_kp;
    }

    /**
     * @brief Get the integral gain (Ki).
     * @return The integral gain (Ki).
     */
    const double getKi() const
    {
        std::scoped_lock lock(m_kiMutex);
        if (!isInitialized("ki"))
        {
            throw std::runtime_error("Ki is not initialized");
        }
        return m_ki;
    }

    /**
     * @brief Set the integral gain (Ki) for the regulator.
     * @param t_ki The integral gain (Ki) to be set.
     */
    void setKi(double t_ki)
    {
        std::scoped_lock lock(m_kiMutex);
        setInitializationStatus("ki", true);
        m_ki = t_ki;
    }

    /**
     * @brief Get the derivative gain (Kd).
     * @return The derivative gain (Kd).
     */
    const double getKd() const
    {
        std::scoped_lock lock(m_kdMutex);
        if (!isInitialized("kd"))
        {
            throw std::runtime_error("Kd is not initialized");
        }
        return m_kd;
    }

    /**
     * @brief Set the derivative gain (Kd) for the regulator.
     * @param t_kd The derivative gain (Kd) to be set.
     */
    void setKd(double t_kd)
    {
        std::scoped_lock lock(m_kdMutex);
        setInitializationStatus("kd", true);
        m_kd = t_kd;
    }


    /**
     * @brief Get the setpoint value.
     * @return The setpoint value.
     */
    const SensorData getSetpoint() const
    {
        std::scoped_lock lock(m_setpointMutex);
        if (!isInitialized("setpoint"))
        {
            throw std::runtime_error("Setpoint is not initialized");
        }
        return m_setpoint;
    }

    /**
     * @brief Set the setpoint for the regulator.
     * @param t_setpoint The setpoint value to be set.
     * @throws std::invalid_argument if the setpoint type is not supported.
     */
    void setSetpoint(SensorData t_setpoint)
    {
        std::scoped_lock lock(m_setpointMutex);
        // Check if the setpoint type is valid
        if (!std::holds_alternative<int>(t_setpoint) &&
            !std::holds_alternative<double>(t_setpoint) &&
            !std::holds_alternative<Vector2D>(t_setpoint) &&
            !std::holds_alternative<Vector3D>(t_setpoint))
        {
            throw std::invalid_argument("Setpoint type is not supported, must be int, double, Vector2D or Vector3D");
        }

        setInitializationStatus("setpoint", true);
        m_setpoint = t_setpoint;
    }

    double getSetpointRampingFixrate()
    {
        std::scoped_lock lock(m_setpointRampingFixrateMutex);
        if (!isInitialized("setpointRampingFixrate"))
        {
            throw std::runtime_error("Setpoint ramping fix rate is not initialized");
        }
        return m_setpointRampingFixrate;
    }

    void setSetpointRampingFixrate(double t_setpointRampingFixrate)
    {
        std::scoped_lock lock(m_setpointRampingFixrateMutex);
        setInitializationStatus("setpointRampingFixrate", true);
        m_setpointRampingFixrate = t_setpointRampingFixrate;
    }

    /**
     * @brief Get the minimum output value.
     * @return The minimum output value.
     */
    const SensorData getOutputMin() const
    {
        std::scoped_lock lock(m_outputMinMutex);
        if (!isInitialized("outputMin"))
        {
            throw std::runtime_error("Output minimum is not initialized");
        }
        return m_outputMin;
    }

    /**
     * @brief Get the maximum output value.
     * @return The maximum output value.
     */
    const SensorData getOutputMax() const
    {
        std::scoped_lock lock(m_outputMaxMutex);
        if (!isInitialized("outputMax"))
        {
            throw std::runtime_error("Output maximum is not initialized");
        }
        return m_outputMax;
    }

    /**
     * @brief Set the output limits for the regulator.
     * @param t_outputMin The minimum output value.
     * @param t_outputMax The maximum output value.
     * @throws std::invalid_argument if the limits are not of the same type or if they are not valid.
     */
    void setOutputLimits(SensorData t_outputMin, SensorData t_outputMax)
    {
        std::scoped_lock lock(m_outputMinMutex, m_outputMaxMutex);
        // Check if the limits are of the same type
        if (t_outputMin.index() != t_outputMax.index())
        {
            throw std::invalid_argument("Limits must be of the same type");
        }

        // Check if the limits are invalid
        if (!std::visit([](const auto& minVal, const auto& maxVal) -> bool {
            using T = std::decay_t<decltype(minVal)>;
            using U = std::decay_t<decltype(maxVal)>;

            // Check if the types are the same
            if constexpr (!std::is_same_v<T, U>)
            {
                return false; // Types must be the same
            }
            else if constexpr ( std::is_same_v<T, int> || std::is_same_v<T, double> || std::is_same_v<T, Vector2D> || std::is_same_v<T, Vector3D>)
            {
                return minVal < maxVal;
            }
            else
            {
                return false;
            }
        }, t_outputMin, t_outputMax))
        {
            throw std::invalid_argument("Minimum cannot be greater than maximum and must be of a supported type");
        }

        setInitializationStatus("outputMin", true);
        setInitializationStatus("outputMax", true);
        m_outputMin = t_outputMin;
        m_outputMax = t_outputMax;
    }

    /**
     * @brief Get the minimum integral value (for anti-windup).
     * @return The minimum integral value.
     */
    const SensorData getIntegralMin() const
    {
        std::scoped_lock lock(m_integralMinMutex);
        if (!isInitialized("integralMin"))
        {
            throw std::runtime_error("Integral minimum is not initialized");
        }
        return m_integralMin;
    }

    /**
     * @brief Get the maximum integral value (for anti-windup).
     * @return The maximum integral value.
     */
    const SensorData getIntegralMax() const
    {
        std::scoped_lock lock(m_integralMaxMutex);
        if (!isInitialized("integralMax"))
        {
            throw std::runtime_error("Integral maximum is not initialized");
        }
        return m_integralMax;
    }
    
    /**
     * @brief Set the integral limits for anti-windup.
     * @param t_integralMin The minimum integral value.
     * @param t_integralMax The maximum integral value.
     * @throws std::invalid_argument if the integral limits are not of the same type or if they are not valid.
     */
    void setIntegralLimits(SensorData t_integralMin, SensorData t_integralMax)
    {
        std::scoped_lock lock(m_integralMinMutex, m_integralMaxMutex);
        // Check if the integral limits are of the same typ
        if (t_integralMin.index() != t_integralMax.index())
        {
            throw std::invalid_argument("Integral limits must be of the same type");
        }

        // Check if the integral limits are invalid
        if (!std::visit([](const auto& minVal, const auto& maxVal) -> bool {
            using T = std::decay_t<decltype(minVal)>;
            using U = std::decay_t<decltype(maxVal)>;
            // Check if the types are the same
            if constexpr (!std::is_same_v<T, U>)
            {
                return false; // Types must be the same
            }
            else if constexpr ( std::is_same_v<T, int> || std::is_same_v<T, double> || std::is_same_v<T, Vector2D> || std::is_same_v<T, Vector3D>)
            {
                return minVal < maxVal;
            }
            else
            {
                return false;
            }
        }, t_integralMin, t_integralMax))
        {
            throw std::invalid_argument("Integral minimum cannot be greater than integral maximum and must be of a supported type");
        }

        setInitializationStatus("integralMin", true);
        setInitializationStatus("integralMax", true);
        m_integralMin = t_integralMin;
        m_integralMax = t_integralMax;
    }

    /**
     * @brief Start the regulation process.
     * @details This method starts the regulation process by creating a separate thread that runs the regulation loop. It also sets the regulator to the regulating state.
     */
    void startRegulation() override
    {
        if (!isInitialized("kp"))
        {
            throw std::runtime_error("Kp is not initialized");
        }
        if (!isInitialized("ki"))
        {
            throw std::runtime_error("Ki is not initialized");
        }
        if (!isInitialized("kd"))
        {
            throw std::runtime_error("Kd is not initialized");
        }
        if (!isInitialized("setpoint"))
        {
            throw std::runtime_error("Setpoint is not initialized");
        }
        if (!isInitialized("outputMin"))
        {
            throw std::runtime_error("Output minimum is not initialized");
        }
        if (!isInitialized("outputMax"))
        {
            throw std::runtime_error("Output maximum is not initialized");
        }
        if (!isInitialized("integralMin"))
        {
            throw std::runtime_error("Integral minimum is not initialized");
        }
        if (!isInitialized("integralMax"))
        {
            throw std::runtime_error("Integral maximum is not initialized");
        }
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
     * @details This method stops the regulation process by setting the regulator to the non-regulating state and stopping the actuator. It also joins the regulation thread if it's running.
     */
    void stopRegulation() override
    {
        setIsRegulating(false);

        // Join the regulation thread if it's running
        if (getRegulationThread().joinable())
        {
            getRegulationThread().join();
        }

        // Reset the regulator state
        reset();
    }
};

} // namespace control_engineering_uni_a

#endif // REGULATOR_LINEAR_PID_REGULATOR_HPP