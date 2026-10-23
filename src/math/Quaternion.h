#pragma once

#include "Precision.h"
#include "Matrix3.h"


namespace PE {

    class Quaternion
    {
    public:
        Real w;
        Real x;
        Real y;
        Real z;

        Quaternion();

        Quaternion(
            Real w,
            Real x,
            Real y,
            Real z
        );

        static Quaternion Identity();

        Real Length() const;
        Real LengthSquared() const;

        bool IsNearlyZero() const;

        void NormalizeSafe();
        void NormalizeUnsafe();

        Quaternion GetNormalizedSafe() const;
        Quaternion GetNormalizedUnsafe() const;

        Quaternion Conjugated() const;

        Quaternion operator*(const Quaternion& other) const;
        Quaternion& operator*=(const Quaternion& other);
        Quaternion operator*(Real scalar) const;
        Quaternion operator+(const Quaternion& other) const;


        Matrix3 ToMatrix3() const;



    private:
        static constexpr Real NORMALIZE_EPSILON = PE::Real(1e-6);
        static constexpr Real NORMALIZE_EPSILON_SQUARED =
            NORMALIZE_EPSILON * NORMALIZE_EPSILON;
    };

    Quaternion operator*(Real scalar, const Quaternion& q);
}