#ifndef UTILITY_UNITS_HPP
#define UTILITY_UNITS_HPP

namespace control_engineering_uni_a
{

enum class AccelerometerUnit
{
    IN_MG,
    IN_G,
    IN_M_PER_S2
};

enum class GyroscopeUnit
{
    IN_MDEG_PER_S,
    IN_DEG_PER_S,
    IN_RAD_PER_S
};

enum class TemperatureUnit
{
    IN_CELSIUS,
    IN_KELVIN,
    IN_FARENHEIT
};

} // namespace control_engineering_uni_a

#endif // UTILITY_UNITS_HPP