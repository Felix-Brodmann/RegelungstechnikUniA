#ifndef REGULATOR_LINEAR_P_REGULATOR_HPP
#define REGULATOR_LINEAR_P_REGULATOR_HPP

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
    SensorData m_setpoint; // Setpoint value
    std::thread m_regulationThread; // Regulation thread
    std::map<std::string, bool> m_initilizationList; // Initialization list for the regulator

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
                    
                    // Calculate the control value
                    return error * getKp();
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
     */
    PRegulator(std::string& t_regulatorName, std::unique_ptr<Sensor> t_sensor, std::unique_ptr<Actuator> t_actuator, double t_kp, SensorData t_setpoint)
        : Regulator(t_regulatorName, std::move(t_sensor), std::move(t_actuator))
    {
        setKp(t_kp);
        setSetpoint(t_setpoint);
    }

    /**
     * @brief Partial constructor for the PRegulator class.
     * @param t_regulatorName The name of the regulator.
     * @param t_sensor A unique pointer to the sensor to be used.
     * @param t_actuator A unique pointer to the actuator to be used.
     */
    PRegulator(std::string& t_regulatorName, std::unique_ptr<Sensor> t_sensor, std::unique_ptr<Actuator> t_actuator)
        : Regulator(t_regulatorName, std::move(t_sensor), std::move(t_actuator))
    {
        setInitilizationStatus("kp", false);
        setInitilizationStatus("setpoint", false);
    }

    /**
     * @brief Destructor for the PRegulator class.
     */
    ~PRegulator()
    {
        stopRegulation();
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
     * @throws std::runtime_error If Kp is not initialized.
     */
    double getKp() const
    { 
        if (!isInitilized("kp"))
        {
            throw std::runtime_error("Kp is not initialized");
        }
        return m_kp;
    }

    /**
     * @brief Set the setpoint value for the regulator.
     * @param t_setpoint The setpoint value to be set.
     */
    void setSetpoint(SensorData t_setpoint)
    {
        setInitilizationStatus("setpoint", true);
        m_setpoint = t_setpoint;
    }

    /**
     * @brief Get the setpoint value.
     * @return The setpoint value.
     */
    const SensorData getSetpoint() const
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
     * @throws std::runtime_error If regulation is already in progress.
     */
    void startRegulation() override
    {
        if (!isInitilized("kp"))
        {
            throw std::runtime_error("Kp is not initialized");
        }
        if (!isInitilized("setpoint"))
        {
            throw std::runtime_error("Setpoint is not initialized");
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

        if (m_regulationThread.joinable())
        {
            m_regulationThread.join(); // Wait for the regulation thread to finish
        }
    }
};

} // namespace control_engineering_uni_a

#endif // REGULATOR_LINEAR_P_REGULATOR_HPP