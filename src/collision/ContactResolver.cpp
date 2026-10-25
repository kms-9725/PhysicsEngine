#include "ContactResolver.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <functional>

namespace PE {
namespace {

constexpr Real kMinEffectiveMass = Real{1e-8f};
constexpr Real kAnchorMatchDistance2 = Real{0.10f * 0.10f};
constexpr Real kNormalMatchCos = Real{0.95f};
constexpr Real kRestitutionSpeedThreshold = Real{1.0f};
constexpr Real kWarmStartMultiplier = Real{1.0f};
constexpr Real kPositionCorrectionFactor = Real{0.35f};
constexpr Real kMaxPositionCorrection = Real{0.08f};

// Contact cache is grouped by rigid-body pair. A large pyramid can have
// thousands of contacts; a linear scan of the entire cache PER contact
// would otherwise make Warm Starting O(total_contacts^2).
struct BodyPair
{
    const RigidBody* a = nullptr;
    const RigidBody* b = nullptr;
    bool operator==(const BodyPair& other) const noexcept
    {
        return a == other.a && b == other.b;
    }
};
struct BodyPairHash
{
    std::size_t operator()(const BodyPair& p) const noexcept
    {
        const std::size_t h1 = std::hash<const RigidBody*>{}(p.a);
        const std::size_t h2 = std::hash<const RigidBody*>{}(p.b);
        return h1 ^ (h2 + std::size_t{0x9e3779b9} + (h1 << 6) + (h1 >> 2));
    }
};

bool Movable(const RigidBody* body)
{
    return body != nullptr && body->HasFiniteMass();
}

Vector3 LocalAnchor(const RigidBody* body, const Vector3& point)
{
    return body == nullptr ? Vector3() :
        body->orientation.ToMatrix3().Transposed() * (point - body->position);
}

Vector3 WorldAnchor(const RigidBody* body, const Vector3& local)
{
    return body == nullptr ? Vector3() :
        body->position + body->orientation.ToMatrix3() * local;
}

void MakeTangents(const Vector3& n, Vector3& t1, Vector3& t2)
{
    // Stable orthonormal contact basis. n is already normalized.
    if (std::abs(n.x) > std::abs(n.z))
        t1 = Vector3(-n.y, n.x, Real{0}).GetNormalizedUnsafe();
    else
        t1 = Vector3(Real{0}, -n.z, n.y).GetNormalizedUnsafe();
    t2 = n.Cross(t1);
}

Real EffectiveMassDenominator(const RigidBody* a, const RigidBody* b,
                              const Vector3& ra, const Vector3& rb,
                              const Vector3& direction)
{
    Real k = Real{0};
    if (Movable(a))
    {
        const Vector3 cross = ra.Cross(direction);
        k += a->inverseMass + cross.Dot(a->inverseInertiaTensorWorld * cross);
    }
    if (Movable(b))
    {
        const Vector3 cross = rb.Cross(direction);
        k += b->inverseMass + cross.Dot(b->inverseInertiaTensorWorld * cross);
    }
    return k;
}

Real InverseEffectiveMass(const RigidBody* a, const RigidBody* b,
                          const Vector3& ra, const Vector3& rb,
                          const Vector3& direction)
{
    const Real k = EffectiveMassDenominator(a, b, ra, rb, direction);
    return k > kMinEffectiveMass ? Real{1} / k : Real{0};
}

Vector3 PointVelocity(const RigidBody* b, const Vector3& r)
{
    return b ? b->linearVelocity + b->angularVelocity.Cross(r) : Vector3();
}

void ApplyVelocityImpulse(RigidBody* a, RigidBody* b,
                          const Vector3& ra, const Vector3& rb,
                          const Vector3& impulse)
{
    if (Movable(a))
    {
        a->linearVelocity += impulse * a->inverseMass;
        a->angularMomentum += ra.Cross(impulse);
        a->SyncAngularVelocityFromMomentum();
    }
    if (Movable(b))
    {
        b->linearVelocity -= impulse * b->inverseMass;
        b->angularMomentum -= rb.Cross(impulse);
        b->SyncAngularVelocityFromMomentum();
    }
}

struct SolverPoint
{
    Contact* contact = nullptr;
    Vector3 localA, localB;
    Vector3 n, t1, t2;
    Vector3 rA, rB;
    Real invMassN = Real{0};
    Real invMassT1 = Real{0};
    Real invMassT2 = Real{0};
    Real bounceSpeed = Real{0};
    Real normalImpulse = Real{0};
    Real tangentImpulse1 = Real{0};
    Real tangentImpulse2 = Real{0};
};

// Separate position-level projection (not velocity-level Baumgarte bias):
// correct penetrations without directly adding physical velocity impulses.
void CorrectPositions(std::vector<SolverPoint>& points, unsigned iterations,
                      Real penetrationSlop)
{
    for (unsigned pass = 0; pass < iterations; ++pass)
    {
        bool changed = false;
        for (SolverPoint& sp : points)
        {
            Contact& c = *sp.contact;
            RigidBody* a = c.bodyA;
            RigidBody* b = c.bodyB;
            if (!Movable(a) && !Movable(b)) continue;

            // Contact.point was the common initial anchor at detection time.
            // Delta anchors measure separation gained since collision detection.
            const Vector3 wa = WorldAnchor(a, sp.localA);
            const Vector3 wb = b ? WorldAnchor(b, sp.localB) : c.point;
            const Real remainingPenetration = c.penetration - (wa - wb).Dot(sp.n);
            if (remainingPenetration <= penetrationSlop) continue;

            const Vector3 ra = wa - a->position;
            const Vector3 rb = b ? wb - b->position : Vector3();
            const Real invK = InverseEffectiveMass(a, b, ra, rb, sp.n);
            if (invK == Real{0}) continue;

            const Real separation = std::min(
                kPositionCorrectionFactor * (remainingPenetration - penetrationSlop),
                kMaxPositionCorrection);
            const Vector3 impulse = sp.n * (separation * invK);
            if (Movable(a))
            {
                a->position += impulse * a->inverseMass;
                a->RotateByRotationVector(a->inverseInertiaTensorWorld * ra.Cross(impulse));
            }
            if (Movable(b))
            {
                b->position -= impulse * b->inverseMass;
                b->RotateByRotationVector(b->inverseInertiaTensorWorld * rb.Cross(-impulse));
            }
            changed = true;
        }
        if (!changed) break;
    }

    // Orientation changed without applying physical torque. Keep world angular
    // momentum as the authoritative state, as in the user's RigidBody design.
    for (const SolverPoint& sp : points)
    {
        if (Movable(sp.contact->bodyA)) sp.contact->bodyA->SyncAngularVelocityFromMomentum();
        if (Movable(sp.contact->bodyB)) sp.contact->bodyB->SyncAngularVelocityFromMomentum();
    }
}

} // namespace

void ContactResolver::ClearCache()
{
    cache_.clear();
    stats_ = Statistics{};
}

void ContactResolver::ResolveContacts(std::vector<Contact>& contacts,
                                      unsigned positionIterations,
                                      unsigned velocityIterations,
                                      Real dt, Real positionEpsilon,
                                      Real velocityEpsilon)
{
    assert(dt > Real{0});
    assert(positionEpsilon >= Real{0});
    assert(velocityEpsilon >= Real{0});
    (void)dt; // Current fixed-step implementation uses separate positional correction.
    (void)velocityEpsilon; // Fixed-count PGS passes; no old maximum-error selection.

    stats_ = Statistics{};
    stats_.contactCount = contacts.size();
    if (contacts.empty())
    {
        cache_.clear();
        return;
    }

    std::vector<SolverPoint> points;
    points.reserve(contacts.size());
    std::vector<bool> used(cache_.size(), false);

    // Build once per frame; typical pair has <=4 cached contact points.
    std::unordered_map<BodyPair, std::vector<std::size_t>, BodyPairHash> byPair;
    byPair.reserve(cache_.size());
    for (std::size_t i = 0; i < cache_.size(); ++i)
        byPair[{cache_[i].bodyA, cache_[i].bodyB}].push_back(i);

    for (Contact& c : contacts)
    {
        if (!c.bodyA || (!Movable(c.bodyA) && !Movable(c.bodyB))) continue;
        const Real n2 = c.normal.LengthSquared();
        if (n2 < Real{0.5f}) continue;

        SolverPoint sp;
        sp.contact = &c;
        sp.n = c.normal / static_cast<Real>(std::sqrt(n2));
        sp.localA = LocalAnchor(c.bodyA, c.point);
        sp.localB = LocalAnchor(c.bodyB, c.point);
        sp.rA = c.point - c.bodyA->position;
        sp.rB = c.bodyB ? c.point - c.bodyB->position : Vector3();
        MakeTangents(sp.n, sp.t1, sp.t2);
        sp.invMassN = InverseEffectiveMass(c.bodyA, c.bodyB, sp.rA, sp.rB, sp.n);
        sp.invMassT1 = InverseEffectiveMass(c.bodyA, c.bodyB, sp.rA, sp.rB, sp.t1);
        sp.invMassT2 = InverseEffectiveMass(c.bodyA, c.bodyB, sp.rA, sp.rB, sp.t2);
        if (sp.invMassN == Real{0}) continue;

        // Match persistent impulses by body pair, normal and LOCAL anchors;
        // don't assume the contact points have the same array index next frame.
        int best = -1;
        Real bestError = kAnchorMatchDistance2 * Real{2};
        const auto candidates = byPair.find({c.bodyA, c.bodyB});
        if (candidates != byPair.end())
        {
            for (const std::size_t i : candidates->second)
            {
                if (used[i]) continue;
                const CachedContact& old = cache_[i];
                if (old.normal.Dot(sp.n) < kNormalMatchCos) continue;
                const Real ea = (old.localA - sp.localA).LengthSquared();
                const Real eb = c.bodyB ? (old.localB - sp.localB).LengthSquared() : Real{0};
                if (ea > kAnchorMatchDistance2 || eb > kAnchorMatchDistance2) continue;
                if (ea + eb < bestError)
                {
                    bestError = ea + eb;
                    best = static_cast<int>(i);
                }
            }
        }
        if (best >= 0)
        {
            const std::size_t i = static_cast<std::size_t>(best);
            used[i] = true;
            const CachedContact& old = cache_[i];
            sp.normalImpulse = std::max(Real{0}, old.normalImpulse) * kWarmStartMultiplier;
            // Project the old tangent impulse onto the current tangent plane.
            const Vector3 tangential = old.tangentImpulse - sp.n * old.tangentImpulse.Dot(sp.n);
            sp.tangentImpulse1 = tangential.Dot(sp.t1) * kWarmStartMultiplier;
            sp.tangentImpulse2 = tangential.Dot(sp.t2) * kWarmStartMultiplier;
            ++stats_.warmStartedCount;
        }

        const Vector3 relative = PointVelocity(c.bodyA, sp.rA)
                               - PointVelocity(c.bodyB, sp.rB);
        const Real vn = relative.Dot(sp.n);
        // Restitution is a NEW-impact effect, not a perpetual resting bounce.
        if (best < 0 && vn < -kRestitutionSpeedThreshold)
            sp.bounceSpeed = std::clamp(c.restitution, Real{0}, Real{1}) * (-vn);
        points.push_back(sp);
    }

    // Warm starting applies cached impulses to both linear and angular velocity.
    for (SolverPoint& sp : points)
    {
        const Real friction = std::max(Real{0}, sp.contact->friction);
        const Real limit = friction * sp.normalImpulse;
        const Real tangent2 = sp.tangentImpulse1 * sp.tangentImpulse1
                            + sp.tangentImpulse2 * sp.tangentImpulse2;
        if (tangent2 > limit * limit && tangent2 > Real{0})
        {
            const Real scale = limit / static_cast<Real>(std::sqrt(tangent2));
            sp.tangentImpulse1 *= scale;
            sp.tangentImpulse2 *= scale;
        }
        const Vector3 impulse = sp.n * sp.normalImpulse
                              + sp.t1 * sp.tangentImpulse1
                              + sp.t2 * sp.tangentImpulse2;
        ApplyVelocityImpulse(sp.contact->bodyA, sp.contact->bodyB,
                             sp.rA, sp.rB, impulse);
    }

    // Sequential impulses: one contact is applied immediately before the next.
    // Non-negative accumulated normal impulses enforce the unilateral contact.
    for (unsigned pass = 0; pass < velocityIterations; ++pass)
    {
        for (SolverPoint& sp : points)
        {
            Contact& c = *sp.contact;
            const Vector3 rel = PointVelocity(c.bodyA, sp.rA) - PointVelocity(c.bodyB, sp.rB);
            const Real vn = rel.Dot(sp.n);
            const Real oldN = sp.normalImpulse;
            sp.normalImpulse = std::max(
                Real{0}, oldN + sp.invMassN * (sp.bounceSpeed - vn));
            const Real dn = sp.normalImpulse - oldN;
            if (dn != Real{0})
                ApplyVelocityImpulse(c.bodyA, c.bodyB, sp.rA, sp.rB, sp.n * dn);

            // Two tangent directions approximate isotropic Coulomb friction.
            const Vector3 tangentialVelocity = PointVelocity(c.bodyA, sp.rA)
                                             - PointVelocity(c.bodyB, sp.rB);
            const Real oldT1 = sp.tangentImpulse1;
            const Real oldT2 = sp.tangentImpulse2;
            Real newT1 = oldT1 - sp.invMassT1 * tangentialVelocity.Dot(sp.t1);
            Real newT2 = oldT2 - sp.invMassT2 * tangentialVelocity.Dot(sp.t2);
            const Real frictionLimit = std::max(Real{0}, c.friction) * sp.normalImpulse;
            const Real jt2 = newT1 * newT1 + newT2 * newT2;
            if (jt2 > frictionLimit * frictionLimit && jt2 > Real{0})
            {
                const Real scale = frictionLimit / static_cast<Real>(std::sqrt(jt2));
                newT1 *= scale;
                newT2 *= scale;
            }
            sp.tangentImpulse1 = newT1;
            sp.tangentImpulse2 = newT2;
            const Vector3 dT = sp.t1 * (newT1 - oldT1) + sp.t2 * (newT2 - oldT2);
            ApplyVelocityImpulse(c.bodyA, c.bodyB, sp.rA, sp.rB, dT);
        }
    }

    // Save the impulses BEFORE positional correction; these are real velocity impulses.
    std::vector<CachedContact> next;
    next.reserve(points.size());
    for (const SolverPoint& sp : points)
    {
        CachedContact cached;
        cached.bodyA = sp.contact->bodyA;
        cached.bodyB = sp.contact->bodyB;
        cached.localA = sp.localA;
        cached.localB = sp.localB;
        cached.normal = sp.n;
        cached.normalImpulse = sp.normalImpulse;
        cached.tangentImpulse = sp.t1 * sp.tangentImpulse1 + sp.t2 * sp.tangentImpulse2;
        next.push_back(cached);
        stats_.totalNormalImpulse += sp.normalImpulse;
    }
    cache_.swap(next);

    // Positional correction is separate from the PGS velocity impulses.
    CorrectPositions(points, positionIterations, positionEpsilon);
}

} // namespace PE
