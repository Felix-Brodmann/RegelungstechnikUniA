#ifndef UTILITY_FILTER_COMPLEMENTARYFILTER_HPP
#define UTILITY_FILTER_COMPLEMENTARYFILTER_HPP

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <variant>

#include "utility/types.hpp"
#include "utility/filter.hpp"

namespace control_engineering_uni_a
{

/**
 * @class ComplementaryFilter
 * @brief A filter that combines gyroscope and accelerometer from an IMU sensor to get the orientation angle of the Z-axis.
 * @details This filter uses a complementary filter approach to combine the high-frequency, high-noise gyroscope data with the low-frequency, low-noise accelerometer data to estimate the orientation angle around the Z-axis.
 * @author Felix Brodmann
 * @date 2025-05-26
 */
class ComplementaryFilter : public Filter
{
private:
    double m_alpha = 0.98; // Alpha value for the filter, must be between 0 and 1
    std::chrono::high_resolution_clock::time_point m_lastTime = std::chrono::high_resolution_clock::now(); // Last time the filter was updated

public:
    /**
     * @brief Constructs a ComplementaryFilter with a specified alpha value.
     * @param t_alpha The alpha value for the filter, must be between 0 and 1.
     * @throws std::invalid_argument If the alpha value is not in the range [0, 1].
     */
    ComplementaryFilter(const double t_alpha) : Filter("ComplementaryFilter")
    {
        if (t_alpha < 0 || t_alpha > 1)
        {
            throw std::invalid_argument("Alpha must be between 0 and 1");
        }
        m_alpha = t_alpha;
    }

    /**
     * @brief Default constructor for ComplementaryFilter.
     * @details Initializes the filter with a default alpha value of 0.98.
     */
    void update(const SensorData& data)
    {
        if (std::holds_alternative<IMUData>(data))
        {
            const IMUData& imuData = std::get<IMUData>(data);
            double gyroscopeZ = imuData.gyroscope.z;
            double accelerometerX = imuData.acceleration.x;
            double accelerometerY = imuData.acceleration.y;
            double latestAngle = 0.0;

            // Get the latest filtered angle, else use the initalized value
            if (std::holds_alternative<double>(getFilteredData()))
            {
                latestAngle = std::get<double>(getFilteredData());
            }

            std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsedTime = currentTime - m_lastTime;
            double dt = elapsedTime.count();
            m_lastTime = currentTime;

            double gyroAngleZ = latestAngle + gyroscopeZ * dt;
            double accelAngleZ = std::atan(accelerometerY /accelerometerX) * 180 / M_PI;
            double newAngle = m_alpha * gyroAngleZ + (1 - m_alpha) * accelAngleZ;
            setFilteredData(newAngle);
            setFilteredDataWithTimestamp({newAngle, std::chrono::system_clock::now()});
        }
        else
        {
            throw std::invalid_argument("Invalid data type for ComplementaryFilter. Expected IMUData.");
        }
    }
};

} // namespace control_engineering_uni_a

#endif // UTILITY_FILTER_COMPLEMENTARYFILTER_HPP