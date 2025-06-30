#ifndef REGULATOR_LINEAR_P_REGULATOR_HPP
#define REGULATOR_LINEAR_P_REGULATOR_HPP

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
 * @class PRegulator
 * @brief Implements a proportional (P) regulator for control systems.
 * @details The PRegulator class derives from the Regulator base class and provides a proportional control strategy. It supports various sensor data types using std::variant. The regulation loop runs in a separate thread and continuously reads sensor data, computes the control value based on the proportional gain (Kp) and setpoint, and sends the control value to the actuator.
 * @note The setpoint must have the same type as the sensor data provided by the sensor.
 * @author Felix Brodmann
 * @date 2025-05-21
 */
class PRegulator : public Regulator
{
private:
    double m_kp = 1.0; // Proportional gain
    mutable std::mutex m_kpMutex; // Mutex for Kp to ensure thread safety
    SensorData m_setpoint; // Setpoint value
    mutable std::mutex m_setpointMutex; // Mutex for setpoint to ensure thread safety
    SensorData m_outputMin; // Minimum output value
    mutable std::mutex m_outputMinMutex; // Mutex for outputMin to ensure thread safety
    SensorData m_outputMax; // Maximum output value
    mutable std::mutex m_outputMaxMutex; // Mutex for outputMax to ensure thread safety

    std::thread m_regulationThread; // Regulation thread
    std::map<std::string, bool> m_initializationList; // Initialization list for the regulator

    /**
     * @brief Get the regulation thread.
     * @return The regulation thread.
     */
    std::thread& getRegulationThread() { return m_regulationThread; }

    /**
     * @brief Set the regulation thread.
     * @param t_thread The thread to be set for regulation.
     */
    void setRegulationThread(std::thread t_thread)
    {
        // Join the previous thread if it is joinable
        if (m_regulationThread.joinable())
        {
            m_regulationThread.join();
        }
        m_regulationThread = std::move(t_thread);
    }

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
     * @brief Regulation loop that runs in a separate thread.
     * @details This method continuously reads sensor data, computes the control value based on the proportional gain (Kp) and setpoint, and sends the control value to the actuator.
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
                    getOutputStream().write(getRegulatorName(), "Calculated error: " + toString(error));
                    
                    // Calculate the control value
                    auto controlValue = std::clamp<T>(  error * getKp(),
                                                        std::get<T>(getOutputMin()),
                                                        std::get<T>(getOutputMax()));
                    getOutputStream().write(getRegulatorName(), "Calculated control value: " + toString(controlValue));
                    return controlValue;
                }
                else if constexpr (std::is_same_v<T, IMUData>)
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
     * @brief Complete constructor for the PRegulator class.
     * @param t_regulatorName The name of the regulator.
     * @param t_sensor A unique pointer to the sensor to be used.
     * @param t_actuator A unique pointer to the actuator to be used.
     * @param t_kp The proportional gain (Kp) for the regulator.
     * @param t_setpoint The setpoint value for the regulator.
     * @param t_outputMin The minimum output value for the regulator.
     * @param t_outputMax The maximum output value for the regulator.
     */
    PRegulator(const std::string& t_regulatorName, std::unique_ptr<Sensor> t_sensor, std::unique_ptr<Actuator> t_actuator, double t_kp, SensorData t_setpoint, SensorData t_outputMin, SensorData t_outputMax)
        : Regulator(t_regulatorName, std::move(t_sensor), std::move(t_actuator))
    {
        setKp(t_kp);
        setSetpoint(t_setpoint);
        setOutputLimits(t_outputMin, t_outputMax);
    }

    /**
     * @brief Partial constructor for the PRegulator class.
     * @param t_regulatorName The name of the regulator.
     * @param t_sensor A unique pointer to the sensor to be used.
     * @param t_actuator A unique pointer to the actuator to be used.
     */
    PRegulator(const std::string& t_regulatorName, std::unique_ptr<Sensor> t_sensor, std::unique_ptr<Actuator> t_actuator)
        : Regulator(t_regulatorName, std::move(t_sensor), std::move(t_actuator))
    {
        setInitializationStatus("kp", false);
        setInitializationStatus("setpoint", false);
        setInitializationStatus("outputMin", false);
        setInitializationStatus("outputMax", false);
    }

    /**
     * @brief Destructor for the PRegulator class.
     */
    ~PRegulator() noexcept
    {
        // Ensure that the regulation thread is stopped before destruction
        if (isRegulating())
        {
            stopRegulation();
        }
    }

    /**
     * @brief Delete copy constructor and assignment operator to prevent copying.
     * @details The copy constructor is deleted to prevent copying of the PRegulator instance, as it contains a thread that should not be copied.
     */
    PRegulator(const PRegulator&) = delete;

    /**
     * @brief Delete assignment operator to prevent copying.
     * @details This operator is deleted to prevent copying of the PRegulator instance, as it contains a thread that should not be copied.
     */
    PRegulator& operator=(const PRegulator&) = delete;

    /**
     * @brief Get the proportional gain (Kp).
     * @return The proportional gain (Kp).
     * @throws std::runtime_error If Kp is not initialized.
     */
    double getKp() const
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
     * @brief Set the setpoint value for the regulator.
     * @param t_setpoint The setpoint value to be set.
     * @throws std::invalid_argument If the setpoint type is not supported.
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
        std::scoped_lock lockMin(m_outputMinMutex, m_outputMaxMutex);
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
     * @brief Start the regulation process.
     * @details This method starts the regulation loop in a separate thread.
     * @throws std::runtime_error If regulation is already in progress.
     */
    void startRegulation() override
    {
        if (!isInitialized("kp"))
        {
            throw std::runtime_error("Kp is not initialized");
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

        if (getRegulationThread().joinable())
        {
            getRegulationThread().join();
        }
    }
};

} // namespace control_engineering_uni_a

#endif // REGULATOR_LINEAR_P_REGULATOR_HPP