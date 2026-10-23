#pragma once

#include "Precision.h"
#include "Vector3.h"

namespace PE {

    class Matrix3
    {
    public:
        Real m00, m01, m02;
        Real m10, m11, m12;
        Real m20, m21, m22;

        Matrix3();

        Matrix3(
            Real m00, Real m01, Real m02,
            Real m10, Real m11, Real m12,
            Real m20, Real m21, Real m22
        );

        static Matrix3 Identity();

        Matrix3 Transposed() const;

        Vector3 operator*(const Vector3& v) const;
        Matrix3 operator*(const Matrix3& other) const;
    };
}