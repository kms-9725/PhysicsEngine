#pragma once

#include "../math/Precision.h"
#include "../physics/RigidBody.h"


namespace PE {

    class SphereCollider
    {
    public:
        SphereCollider(
            RigidBody* body,
            Real radius,
            Real restitution
        );

        RigidBody* body;

        Real radius;

        Real restitution;

        void InitMassProperties(Real mass);
    };
}