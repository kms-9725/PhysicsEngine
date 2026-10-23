#pragma once

#include <array>

#include "../math/Precision.h"
#include "../math/Vector3.h"
#include "../physics/RigidBody.h"

namespace PE {

    class BoxCollider
    {
    public:
        BoxCollider(
            RigidBody* body,
            const Vector3& halfSize,
            Real restitution
        );

        RigidBody* body;

        // 박스 중심에서 각 면까지의 거리.
        //
        // 전체 크기가 (width, height, depth)라면
        //
        // halfSize =
        // (width / 2, height / 2, depth / 2)
        Vector3 halfSize;

        Real restitution;

        // 박스의 질량과 관성텐서를 RigidBody에 설정
        void InitMassProperties(Real mass);

        void GetWorldVertices(
            std::array<Vector3, 8>& vertices
        ) const;

    };
}