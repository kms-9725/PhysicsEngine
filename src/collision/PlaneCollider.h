#pragma once

#include "../math/Precision.h"
#include "../math/Vector3.h"


namespace PE {

    class PlaneCollider
    {
    public:
        PlaneCollider(
            const Vector3& normal,
            Real offset,
            Real restitution
        );

        // 항상 단위벡터로 저장
        Vector3 normal;

        // plane equation:
        //
        // normal · x = offset
        Real offset;

        Real restitution;
    };
}