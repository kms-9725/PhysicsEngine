#include "SphereCollider.h"

#include <cassert>


namespace PE {

    SphereCollider::SphereCollider(
        RigidBody* body,
        Real radius,
        Real restitution
    )
        : body(body),
        radius(radius),
        restitution(restitution)
    {
        assert(body != nullptr);
        assert(radius > Real{ 0 });
    }


    void SphereCollider::InitMassProperties(Real mass)
    {
        assert(body != nullptr);
        assert(mass > Real{ 0 });

        // 균일한 구의 관성모멘트
        // I = (2/5)mr²

        Real inertia =
            (Real{ 2 } / Real{ 5 })
            * mass * radius * radius;


        Matrix3 inertiaTensor(
            inertia, 0, 0,
            0, inertia, 0,
            0, 0, inertia
        );


        Real inverseInertia =
            Real{ 1 } / inertia;

        Matrix3 inverseInertiaTensor(
            inverseInertia, 0, 0,
            0, inverseInertia, 0,
            0, 0, inverseInertia
        );


        body->InitMassProperties(
            mass,
            inertiaTensor,
            inverseInertiaTensor
        );
    }
}