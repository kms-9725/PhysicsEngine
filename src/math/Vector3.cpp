#include "Vector3.h"
#include <cmath>
#include <cassert>


namespace PE {

    Vector3::Vector3() : x(0), y(0), z(0) {}
    Vector3::Vector3(Real x, Real y, Real z) : x(x), y(y), z(z) {}

    Real Vector3::Length() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    Real Vector3::LengthSquared() const
    {
        return x * x + y * y + z * z;
    }

    void Vector3::NormalizeUnsafe()
    {
        Real lengthSquared = LengthSquared();

        assert(lengthSquared > NORMALIZE_EPSILON_SQUARED);

        Real inverseLength = Real{ 1 } / std::sqrt(lengthSquared);

        x *= inverseLength;
        y *= inverseLength;
        z *= inverseLength;
    }

    void Vector3::NormalizeSafe()
    {
        if (LengthSquared() <= NORMALIZE_EPSILON_SQUARED)
        {
            x = 0;
            y = 0;
            z = 0;
            return;
        }

        NormalizeUnsafe();
    }

    Vector3 Vector3::GetNormalizedUnsafe() const
    {
        Vector3 result = *this;
        result.NormalizeUnsafe();
        return result;
    }

    Vector3 Vector3::GetNormalizedSafe() const
    {
        Vector3 result = *this;
        result.NormalizeSafe();
        return result;
    }

    bool Vector3::IsNearlyZero() const
    {
        return LengthSquared() <= NORMALIZE_EPSILON_SQUARED;
    }

    Real Vector3::Dot(const Vector3& other) const
    {
        return x * other.x
            + y * other.y
            + z * other.z;
    }

    Vector3 Vector3::Cross(const Vector3& other) const
    {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    Vector3 Vector3::operator+(const Vector3& other) const
    {
        return Vector3(
            x + other.x,
            y + other.y,
            z + other.z
        );
    }

    Vector3 Vector3::operator-(const Vector3& other) const
    {
        return Vector3(
            x - other.x,
            y - other.y,
            z - other.z
        );
    }

    Vector3 Vector3::operator-() const
    {
        return Vector3(
            -x,
            -y,
            -z
        );
    }
    Vector3 Vector3::operator*(Real scalar) const
    {
        return Vector3(
            x * scalar,
            y * scalar,
            z * scalar
        );
    }

    Vector3 operator*(Real scalar, const Vector3& v)
    {
        return v * scalar;
    }

    Vector3 Vector3::operator/(Real scalar) const
    {
        assert(scalar != Real{ 0 });

        Real inverseScalar = Real{ 1 } / scalar;

        return Vector3(
            x * inverseScalar,
            y * inverseScalar,
            z * inverseScalar
        );
    }

    Vector3& Vector3::operator+=(const Vector3& other)
    {
        x += other.x;
        y += other.y;
        z += other.z;

        return *this;
    }


    Vector3& Vector3::operator-=(const Vector3& other)
    {
        x -= other.x;
        y -= other.y;
        z -= other.z;

        return *this;
    }


    Vector3& Vector3::operator*=(Real scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;

        return *this;
    }


    Vector3& Vector3::operator/=(Real scalar)
    {
        assert(scalar != Real{ 0 });

        Real inverseScalar = Real{ 1 } / scalar;

        x *= inverseScalar;
        y *= inverseScalar;
        z *= inverseScalar;

        return *this;
    }
}