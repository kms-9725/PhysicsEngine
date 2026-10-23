#include "BoxCollider.h"

#include <cassert>

namespace PE{

    BoxCollider::BoxCollider(
        RigidBody* body,
        const Vector3& halfSize,
        Real restitution
    )
        : body(body),
        halfSize(halfSize),
        restitution(restitution)
    {
        assert(body != nullptr);

        assert(halfSize.x > Real{ 0 });
        assert(halfSize.y > Real{ 0 });
        assert(halfSize.z > Real{ 0 });
    }


    void BoxCollider::InitMassProperties(Real mass)
    {
        assert(body != nullptr);
        assert(mass > Real{ 0 });


        Real hx = halfSize.x;
        Real hy = halfSize.y;
        Real hz = halfSize.z;

        // 직육면체의 질량중심 기준 관성모멘트.
        //
        // 전체 길이는
        // width  = 2hx
        // height = 2hy
        // depth  = 2hz
        //
        // 따라서
        //
        // Ixx = m/3 (hy² + hz²)
        // Iyy = m/3 (hx² + hz²)
        // Izz = m/3 (hx² + hy²)

        Real scaledMass = mass / Real{ 3 };


        Real Ixx =
            scaledMass
            * (hy * hy + hz * hz);

        Real Iyy =
            scaledMass
            * (hx * hx + hz * hz);

        Real Izz =
            scaledMass
            * (hx * hx + hy * hy);


        Matrix3 inertiaTensor(
            Ixx, 0, 0,
            0, Iyy, 0,
            0, 0, Izz
        );


        // 대각행렬이므로 inverse도 각 대각성분의 역수.
        Matrix3 inverseInertiaTensor(
            Real{ 1 } / Ixx, 0, 0,
            0, Real{ 1 } / Iyy, 0,
            0, 0, Real{ 1 } / Izz
        );
        // BoxCollider는 계산만 하고,
        // 실제 RigidBody 상태 변경은 RigidBody에게 맡긴다.
        body->InitMassProperties(
            mass,
            inertiaTensor,
            inverseInertiaTensor
        );
    }

    void BoxCollider::GetWorldVertices( //박스의 8개 꼭짓점의 월드 좌표를 줌. 
        //외부에서 std::array<Vector3, 8> 를 만들어서 &로 주면 거기에 담아줌.
        std::array<Vector3, 8>& vertices
    ) const
    {
        assert(body != nullptr);

        Matrix3 rotation =
            body->orientation.ToMatrix3();


        Vector3 localVertices[8] =
        {
            Vector3(-halfSize.x, -halfSize.y, -halfSize.z),
            Vector3(halfSize.x, -halfSize.y, -halfSize.z),
            Vector3(-halfSize.x,  halfSize.y, -halfSize.z),
            Vector3(halfSize.x,  halfSize.y, -halfSize.z),

            Vector3(-halfSize.x, -halfSize.y,  halfSize.z),
            Vector3(halfSize.x, -halfSize.y,  halfSize.z),
            Vector3(-halfSize.x,  halfSize.y,  halfSize.z),
            Vector3(halfSize.x,  halfSize.y,  halfSize.z)
        };


        for (int i = 0; i < 8; ++i)
        {
            vertices[i] =
                body->position
                + rotation * localVertices[i];
        }
    }
}