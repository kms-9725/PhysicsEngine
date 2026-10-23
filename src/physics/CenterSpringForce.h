#pragma once

// Conservative, zero-rest-length spring attached to a fixed world-space point.
// No damping, gravity, contact, or position projection is used in SpringTest.
// F = -k (position - equilibrium).
// This generator acts at the centre of mass and therefore adds no torque.
#include "ForceRegistry.h"

namespace PE
{
    class CenterSpringForce final : public ForceGenerator
    {
    public:
        Vector3 equilibrium{0, 0, 0};
        Real stiffness = Real{10};   // N / m; must be nonnegative

        void Apply(RigidBody& body, Real /*dt*/) override
        {
            if (!body.HasFiniteMass() || stiffness <= Real{0}) return;
            body.AddLinearForce((equilibrium - body.position) * stiffness);
        }

        Real PotentialEnergy(const RigidBody& body) const
        {
            const Vector3 displacement = body.position - equilibrium;
            return Real{0.5f} * stiffness * displacement.LengthSquared();
        }
    };
}
