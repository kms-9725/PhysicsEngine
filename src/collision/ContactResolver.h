#pragma once

#include <cstddef>
#include <vector>
#include "Contact.h"

namespace PE {

// An instance belongs to ONE physics world. Keep it across physics frames,
// call ClearCache() whenever bodies/colliders are destroyed or the scene resets.
// Contact::normal must point from B to A. bodyB == nullptr means static environment.
class ContactResolver
{
public:
    struct Statistics
    {
        std::size_t contactCount = 0;
        std::size_t warmStartedCount = 0;
        Real totalNormalImpulse = Real{0};
    };

    void ResolveContacts(
        std::vector<Contact>& contacts,
        unsigned positionIterations,
        unsigned velocityIterations,
        Real dt,
        Real positionEpsilon = Real{0.005f},
        Real velocityEpsilon = Real{0.01f});

    // Required on scene reset or object deletion: cached pointers are not owning.
    void ClearCache();
    const Statistics& GetStatistics() const { return stats_; }

private:
    struct CachedContact
    {
        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr;
        Vector3 localA;
        Vector3 localB;
        Vector3 normal;
        Real normalImpulse = Real{0};
        Vector3 tangentImpulse; // World-space tangent impulse, independent of basis rotation
    };
    std::vector<CachedContact> cache_;
    Statistics stats_;
};

} // namespace PE
