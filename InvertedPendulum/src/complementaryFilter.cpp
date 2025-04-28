#include "../include/complementaryFilter.hpp"

namespace RegelungstechnikUniA
{
    void ComplementaryFilter::update(const double t_gyroZ, const double t_accelX, const double t_accelY)
    {
        std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedTime = currentTime - m_lastTime;
        double dt = elapsedTime.count();
        m_lastTime = currentTime;

        double gyroAngleZ = m_angle + t_gyroZ * dt;
        double accelAngleZ = std::atan(t_accelY /t_accelX) * 180 / M_PI;
        m_angle = m_alpha * gyroAngleZ + (1 - m_alpha) * accelAngleZ;
    }
}