#pragma once

#include <array>
#include <cstddef>

#include "../math/Precision.h"
#include "../math/Vector3.h"
#include "../physics/RigidBody.h"

namespace PE {

// One collision pair in one physics step.
// This is a TRANSIENT manifold: contact matching and cached impulses
// between frames will be added with the new PGS solver.
struct ManifoldPoint
{
    Vector3 worldPoint;  // Midpoint of the two contact surface points.
    Vector3 localPointA; // Contact point on surface A, in A's local space.
    Vector3 localPointB; // Contact point on surface B, in B's local space.
    Real penetration = Real{0};
};

struct ContactManifold
{
    static constexpr std::size_t MaxPoints = 4;

    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    Vector3 normal; // WORLD space; bodyB -> bodyA, as in PE::Contact.
    Real restitution = Real{0};
    std::array<ManifoldPoint, MaxPoints> points{};
    std::size_t pointCount = 0;
};

} // namespace PE
