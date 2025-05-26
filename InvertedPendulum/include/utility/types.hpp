#ifndef UTILITY_TYPES_HPP
#define UTILITY_TYPES_HPP

#include <string>
#include <variant>
#include <chrono>

namespace control_engineering_uni_a
{

/**
 * @struct Vector2D
 * @brief A structure representing a 2D vector.
 * @details This structure provides basic arithmetic operations for 2D vectors, including addition, subtraction, scalar multiplication, and division.
 */
struct Vector2D
{
    double x = 0.0;
    double y = 0.0;

    Vector2D operator+(const Vector2D& other) const
    {
        return Vector2D{x + other.x, y + other.y};
    }

    Vector2D operator-(const Vector2D& other) const
    {
        return Vector2D{x - other.x, y - other.y};
    }

    Vector2D operator*(double scalar) const
    {
        return Vector2D{x * scalar, y * scalar};
    }

    Vector2D operator/(double scalar) const
    {
        if (scalar == 0.0)
        {
            throw std::runtime_error("Division by zero");
        }
        return Vector2D{x / scalar, y / scalar};
    }

    Vector2D& operator+=(const Vector2D& other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2D& operator-=(const Vector2D& other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2D& operator*=(double scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vector2D& operator/=(double scalar)
    {
        if (scalar == 0.0)
        {
            throw std::runtime_error("Division by zero");
        }
        x /= scalar;
        y /= scalar;
        return *this;
    }

    bool operator<(const Vector2D& other) const
    {
        return x < other.x && y < other.y;
    }

    bool operator>(const Vector2D& other) const
    {
        return x > other.x && y > other.y;
    }
};

inline Vector2D clamp(const Vector2D& value, const Vector2D& min, const Vector2D& max)
{
    return Vector2D{
        std::clamp(value.x, min.x, max.x),
        std::clamp(value.y, min.y, max.y)
    };
}

/**
 * @struct Vector3D
 * @brief A structure representing a 3D vector.
 * @details This structure provides basic arithmetic operations for 3D vectors, including addition, subtraction, scalar multiplication, and division.
 */
struct Vector3D
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vector3D operator+(const Vector3D& other) const
    {
        return Vector3D{x + other.x, y + other.y, z + other.z};
    }

    Vector3D operator-(const Vector3D& other) const
    {
        return Vector3D{x - other.x, y - other.y, z - other.z};
    }

    Vector3D operator*(double scalar) const
    {
        return Vector3D{x * scalar, y * scalar, z * scalar};
    }

    Vector3D operator/(double scalar) const
    {
        if (scalar == 0.0)
        {
            throw std::runtime_error("Division by zero");
        }
        return Vector3D{x / scalar, y / scalar, z / scalar};
    }

    Vector3D& operator+=(const Vector3D& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3D& operator-=(const Vector3D& other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3D& operator*=(double scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vector3D& operator/=(double scalar)
    {
        if (scalar == 0.0)
        {
            throw std::runtime_error("Division by zero");
        }
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    bool operator<(const Vector3D& other) const
    {
        return x < other.x && y < other.y && z < other.z;
    }

    bool operator>(const Vector3D& other) const
    {
        return x > other.x && y > other.y && z > other.z;
    }
};

inline Vector3D clamp(const Vector3D& value, const Vector3D& min, const Vector3D& max)
{
    return Vector3D{
        std::clamp(value.x, min.x, max.x),
        std::clamp(value.y, min.y, max.y),
        std::clamp(value.z, min.z, max.z)
    };
}

/**
 * @struct IMUData
 * @brief A structure representing IMU data.
 * @details This structure contains acceleration, gyroscope, and temperature data.
 */
struct IMUData
{
    Vector3D acceleration;
    Vector3D gyroscope;
    double temperature = 0.0;
};

using SensorData = std::variant<int, double, Vector2D, Vector3D, IMUData>;
using Timestamp = std::chrono::system_clock::time_point;

/**
 * @struct SensorDataWithTimestamp
 * @brief A structure representing sensor data with a timestamp.
 * @details This structure contains the sensor data and the time it was acquired.
 */
struct SensorDataWithTimestamp
{
    SensorData data;
    Timestamp timestamp;
};

enum class InputType
{
    INT,
    DOUBLE,
    VECTOR2D,
    VECTOR3D,
    IMU_DATA,
};

enum class OutputType
{
    INT,
    DOUBLE,
    VECTOR2D,
    VECTOR3D
};

} // namespace control_engineering_uni_a

#endif // UTILITY_TYPES_HPP