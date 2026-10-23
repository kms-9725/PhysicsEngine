#include "Matrix3.h"

namespace PE {

    Matrix3::Matrix3()
        : m00(0), m01(0), m02(0),
        m10(0), m11(0), m12(0),
        m20(0), m21(0), m22(0)
    {
    }


    Matrix3::Matrix3(
        Real m00, Real m01, Real m02,
        Real m10, Real m11, Real m12,
        Real m20, Real m21, Real m22
    )
        : m00(m00), m01(m01), m02(m02),
        m10(m10), m11(m11), m12(m12),
        m20(m20), m21(m21), m22(m22)
    {
    }


    Matrix3 Matrix3::Identity()
    {
        return Matrix3(
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        );
    }


    Matrix3 Matrix3::Transposed() const
    {
        return Matrix3(
            m00, m10, m20,
            m01, m11, m21,
            m02, m12, m22
        );
    }


    Vector3 Matrix3::operator*(const Vector3& v) const
    {
        return Vector3(
            m00 * v.x + m01 * v.y + m02 * v.z,
            m10 * v.x + m11 * v.y + m12 * v.z,
            m20 * v.x + m21 * v.y + m22 * v.z
        );
    }

    Matrix3 Matrix3::operator*(const Matrix3& other) const
    {
        return Matrix3(
            // 0행
            m00 * other.m00 + m01 * other.m10 + m02 * other.m20,
            m00 * other.m01 + m01 * other.m11 + m02 * other.m21,
            m00 * other.m02 + m01 * other.m12 + m02 * other.m22,

            // 1행
            m10 * other.m00 + m11 * other.m10 + m12 * other.m20,
            m10 * other.m01 + m11 * other.m11 + m12 * other.m21,
            m10 * other.m02 + m11 * other.m12 + m12 * other.m22,

            // 2행
            m20 * other.m00 + m21 * other.m10 + m22 * other.m20,
            m20 * other.m01 + m21 * other.m11 + m22 * other.m21,
            m20 * other.m02 + m21 * other.m12 + m22 * other.m22
        );
    }
}