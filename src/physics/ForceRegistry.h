#pragma once

// Header-only sandbox force pipeline. No modifications to the engine RigidBody are necessary.
// Generator lifetime is owned by the world; registrations are non-owning.
#include <algorithm>
#include <cmath>
#include <vector>
#include "RigidBody.h"

namespace PE
{
    class ForceGenerator
    {
    public:
        virtual ~ForceGenerator() = default;
        virtual void Apply(RigidBody& body, Real dt) = 0;
    };

    class GravityForce final : public ForceGenerator
    {
    public:
        Vector3 acceleration{0, Real{-9.8f}, 0};
        void Apply(RigidBody& body, Real) override
        {
            if (body.HasFiniteMass())
                body.AddLinearForce(acceleration * body.GetMass());
        }
    };

    class LinearDragForce final : public ForceGenerator
    {
    public:
        Real coefficient{0};
        void Apply(RigidBody& body, Real) override
        {
            if (body.HasFiniteMass() && coefficient > Real{0})
                body.AddLinearForce(-body.linearVelocity * coefficient);
        }
    };

    class PointSpringForce final : public ForceGenerator
    {
    public:
        Vector3 localAnchor{};
        Vector3 target{};
        Real stiffness{95};
        Real damping{19};
        Real maxAcceleration{450};

        void Apply(RigidBody& body, Real) override
        {
            if (!body.HasFiniteMass()) return;
            const Vector3 r = body.orientation.ToMatrix3() * localAnchor;
            const Vector3 anchor = body.position + r;
            const Vector3 pointVelocity = body.linearVelocity + body.angularVelocity.Cross(r);
            Vector3 acceleration = (target - anchor) * stiffness - pointVelocity * damping;
            const Real squared = acceleration.LengthSquared();
            if (squared > maxAcceleration * maxAcceleration)
                acceleration *= maxAcceleration / static_cast<Real>(std::sqrt(squared));
            body.AddForceAtPoint(acceleration * body.GetMass(), anchor);
        }
    };

    class ForceRegistry
    {
    public:
        struct Entry { RigidBody* body; ForceGenerator* generator; };
        void Add(RigidBody& body, ForceGenerator& generator)
        {
            for (const auto& e : entries_)
                if (e.body == &body && e.generator == &generator) return;
            entries_.push_back({&body, &generator});
        }
        void RemoveGenerator(ForceGenerator& generator)
        {
            entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                [&](const Entry& e) {return e.generator == &generator;}), entries_.end());
        }
        void Clear() { entries_.clear(); }
        void ApplyAll(Real dt)
        {
            for (const Entry& e : entries_)
                if (e.body && e.generator) e.generator->Apply(*e.body, dt);
        }
        std::size_t Size() const { return entries_.size(); }
    private:
        std::vector<Entry> entries_;
    };
}
