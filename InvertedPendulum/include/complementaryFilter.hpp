#ifndef COMPLEMENTARYFILTER_HPP
#define COMPLEMENTARYFILTER_HPP

#include <chrono>
#include <cmath>

namespace RegelungstechnikUniA
{
    class ComplementaryFilter
    {
    private:
        double m_angle = 0;
        double m_alpha = 0.98;
        std::chrono::high_resolution_clock::time_point m_lastTime = std::chrono::high_resolution_clock::now();

    public:
        ComplementaryFilter(const double t_alpha) : m_alpha(t_alpha) {}
        void update(const double t_gyroZ, const double t_accelX, const double t_accelY);
        double getAngle() const { return m_angle; }
    };
}

#endif