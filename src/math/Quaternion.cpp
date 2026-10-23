#include "Quaternion.h"

#include <cassert>
#include <cmath>

namespace PE {

    Quaternion::Quaternion()
        : w(1), x(0), y(0), z(0)
    {
    }


    Quaternion::Quaternion(
        Real w,
        Real x,
        Real y,
        Real z
    )
        : w(w), x(x), y(y), z(z)
    {
    }


    Quaternion Quaternion::Identity()
    {
        return Quaternion(1, 0, 0, 0);
    }


    Real Quaternion::LengthSquared() const
    {
        return w * w
            + x * x
            + y * y
            + z * z;
    }


    Real Quaternion::Length() const
    {
        return std::sqrt(LengthSquared());
    }


    bool Quaternion::IsNearlyZero() const
    {
        return LengthSquared() <= NORMALIZE_EPSILON_SQUARED;
    }


    void Quaternion::NormalizeUnsafe()
    {
        Real lengthSquared = LengthSquared();

        assert(lengthSquared > NORMALIZE_EPSILON_SQUARED);

        Real inverseLength =
            Real{ 1 } / std::sqrt(lengthSquared);

        w *= inverseLength;
        x *= inverseLength;
        y *= inverseLength;
        z *= inverseLength;
    }


    void Quaternion::NormalizeSafe()
    {
        if (LengthSquared() <= NORMALIZE_EPSILON_SQUARED)
        {
            w = 1;
            x = 0;
            y = 0;
            z = 0;

            return;
        }

        NormalizeUnsafe();
    }


    Quaternion Quaternion::GetNormalizedUnsafe() const
    {
        Quaternion result = *this;

        result.NormalizeUnsafe();

        return result;
    }


    Quaternion Quaternion::GetNormalizedSafe() const
    {
        Quaternion result = *this;

        result.NormalizeSafe();

        return result;
    }


    Quaternion Quaternion::Conjugated() const
    {
        return Quaternion(
            w,
            -x,
            -y,
            -z
        );
    }


    Quaternion Quaternion::operator*(
        const Quaternion& other
        ) const
    {
        return Quaternion(
            w * other.w
            - x * other.x
            - y * other.y
            - z * other.z,

            w * other.x
            + x * other.w
            + y * other.z
            - z * other.y,

            w * other.y
            - x * other.z
            + y * other.w
            + z * other.x,

            w * other.z
            + x * other.y
            - y * other.x
            + z * other.w
        );
    }


    Quaternion& Quaternion::operator*=(
        const Quaternion& other
        )
    {
        *this = *this * other;

        return *this;
    }

    Quaternion Quaternion::operator*(Real scalar) const
    {
        return Quaternion(
            w * scalar,
            x * scalar,
            y * scalar,
            z * scalar
        );
    }

    Quaternion operator*(Real scalar, const Quaternion& q)
    {
        return q * scalar;
    }

    Quaternion Quaternion::operator+(const Quaternion& other) const
    {
        return Quaternion(
            w + other.w,
            x + other.x,
            y + other.y,
            z + other.z
        );
    }

    Matrix3 Quaternion::ToMatrix3() const  // 쿼터니언을 3x3 회전 행렬로 변환 (쿼터니언은 단위 쿼터니언이어야 함)
    {                                      // 이 행렬은 로컬->월드변환 행렬임.
                                           // 즉, 강체의 로컬좌표계에서 강체의 특정 위치벡터에 이 행렬을 적용하면, 그 벡터를 월드좌표계로 변환할 수 있음.
        return Matrix3(
            //0행
            1 - 2 * (y * y + z * z),
            2 * (x * y - w * z),
            2 * (x * z + w * y),
            //1행
            2 * (x * y + w * z),
            1 - 2 * (x * x + z * z),
            2 * (y * z - w * x),
            //2행
            2 * (x * z - w * y),
            2 * (y * z + w * x),
            1 - 2 * (x * x + y * y)
        );
    }
}