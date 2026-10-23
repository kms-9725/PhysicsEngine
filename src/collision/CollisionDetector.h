#pragma once

#include <vector>

#include "BoxCollider.h"
#include "PlaneCollider.h"
#include "SphereCollider.h"
#include "Contact.h"
#include "ContactManifold.h"

namespace PE {

    class CollisionDetector
    {
    public:
        static void DetectSphereSphere(
            const SphereCollider& sphereA,
            const SphereCollider& sphereB,
            std::vector<Contact>& contacts
        );
        static void DetectSpherePlane(
            const SphereCollider& sphere,
            const PlaneCollider& plane,
            std::vector<Contact>& contacts
        );
        static void DetectBoxPlane(
            const BoxCollider& box,
            const PlaneCollider& plane,
            std::vector<Contact>& contacts
        );
        static void DetectBoxSphere(
            const BoxCollider& box,
            const SphereCollider& sphere,
            std::vector<Contact>& contacts
        );
        // New geometry API: build up to 4 contact points for one box pair.
        // Returns false if the boxes are not touching.
        static bool BuildBoxBoxManifold(
            const BoxCollider& boxA,
            const BoxCollider& boxB,
            ContactManifold& manifold
        );

        static void DetectBoxBox(
            const BoxCollider& boxA,
            const BoxCollider& boxB,
            std::vector<Contact>& contacts
        );

    };
}