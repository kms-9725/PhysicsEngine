#pragma once
#include "Precision.h"

namespace PE {

    class Vector3
    {
    public:
        Real x;
        Real y;
        Real z;

        Vector3();
        Vector3(Real x, Real y, Real z);

        Real Length() const;
        Real LengthSquared() const;
        void NormalizeUnsafe();        //벡터를 정규화. 내부에서 길이가 0에 가까운지 검사하지 않음. 이 함수를 호출하기 전에 길이가 0에 가까운지 검사해야 함.
        void NormalizeSafe();          //벡터를 정규화. 내부에서 길이가 0에 가까운지 검사, 길이가 0에 가까운 경우 영벡터로 설정.
        Vector3 GetNormalizedUnsafe() const; //벡터를 정규화한 새로운 벡터를 반환. 내부에서 길이가 0에 가까운지 검사하지 않음. 이 함수를 호출하기 전에 길이가 0에 가까운지 검사해야 함.
        Vector3 GetNormalizedSafe() const; //벡터를 정규화한 새로운 벡터를 반환. 내부에서 길이가 0에 가까운지 검사, 길이가 0에 가까운 경우 영벡터를 반환.
        bool IsNearlyZero() const;
        Real Dot(const Vector3& other) const;
        Vector3 Cross(const Vector3& other) const;
        Vector3 operator+(const Vector3& other) const;
        Vector3 operator-(const Vector3& other) const;
        Vector3 operator-() const;
        Vector3 operator*(Real scalar) const;
        Vector3 operator/(Real scalar) const;
        Vector3& operator+=(const Vector3& other);
        Vector3& operator-=(const Vector3& other);
        Vector3& operator*=(Real scalar);
        Vector3& operator/=(Real scalar);

    private:
        static constexpr Real NORMALIZE_EPSILON = PE::Real(1e-6); // 벡터 정규화할 때 길이가 0에 가까운지 검사할 때 사용하는 작은 값.이 값의 제곱보다 길이가 작으면 0에 가까운 것으로 간주.
        static constexpr Real NORMALIZE_EPSILON_SQUARED =
            NORMALIZE_EPSILON * NORMALIZE_EPSILON;
    };

    Vector3 operator*(Real scalar, const Vector3& v);
}