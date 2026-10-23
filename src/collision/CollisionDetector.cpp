#include "CollisionDetector.h"
#include <utility>
#include <initializer_list>
#include <cassert>
#include <array>
#include <cmath>
#include <algorithm>
#include <limits>

namespace PE {

    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    void CollisionDetector::DetectSphereSphere(
        const SphereCollider& sphereA,
        const SphereCollider& sphereB,
        std::vector<Contact>& contacts
    )
    {
        assert(sphereA.body != nullptr);
        assert(sphereB.body != nullptr);

        // 같은 강체에 속한 두 collider는 자기 자신과 충돌하지 않음
        if (sphereA.body == sphereB.body)
        {
            return;
        }


        Vector3 centerA =
            sphereA.body->position;

        Vector3 centerB =
            sphereB.body->position;


        // B의 중심에서 A의 중심으로 향하는 벡터
        Vector3 midline =
            centerA - centerB;

        Real distanceSquared =
            midline.LengthSquared();

        Real radiusSum =
            sphereA.radius + sphereB.radius;


        // 구가 서로 떨어져 있으면 contact 생성하지 않음
        if (distanceSquared > radiusSum * radiusSum)
        {
            return;
        }


        Real distance =
            std::sqrt(distanceSquared);


        // Contact normal: bodyB -> bodyA
        Vector3 normal;

        if (distance > Real{ 0 })
        {
            normal = midline / distance;
        }
        else
        {
            // 두 구의 중심이 정확히 일치하면
            // 중심 간 벡터로는 법선을 결정할 수 없음.
            // 임시로 +X 방향 선택.
            normal = Vector3(1, 0, 0);
        }


        // 각 구의 표면에서 상대 구를 향하는 점
        Vector3 pointA =
            centerA - normal * sphereA.radius;

        Vector3 pointB =
            centerB + normal * sphereB.radius;


        // 두 표면점의 중간을 contact point로 사용
        Vector3 contactPoint =
            (pointA + pointB) * Real{ 0.5 };


        Contact contact;

        contact.bodyA = sphereA.body;
        contact.bodyB = sphereB.body;

        contact.setRestitution(sphereA.restitution, sphereB.restitution);

        contact.point = contactPoint;
        contact.normal = normal;

        contact.penetration =
            radiusSum - distance;


        contacts.push_back(contact);
    }

    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    void CollisionDetector::DetectSpherePlane(
        const SphereCollider& sphere,
        const PlaneCollider& plane,
        std::vector<Contact>& contacts
    )
    {
        assert(sphere.body != nullptr);

        // 구의 중심 (world space)
        Vector3 center =
            sphere.body->position;


        // 구의 중심에서 평면까지의 signed distance에서
        // 구의 반지름을 뺀 값.
        //
        // ballDistance > 0 : 충돌하지 않음
        // ballDistance = 0 : 정확히 접촉
        // ballDistance < 0 : 침투

        Real ballDistance =
            plane.normal.Dot(center)
            - sphere.radius
            - plane.offset;


        if (ballDistance > Real{ 0 })
        {
            return;
        }


        Contact contact;

        contact.bodyA = sphere.body;
        contact.bodyB = nullptr;

        contact.setRestitution(sphere.restitution, plane.restitution);

        // 평면의 법선 방향으로 구를 밀어냄.
        contact.normal = plane.normal;

        // 침투 깊이
        contact.penetration = -ballDistance;

        // 구의 중심을 평면에 정사영한 점.
        contact.point =
            center
            - plane.normal * (ballDistance + sphere.radius);


        contacts.push_back(contact);
    }
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    void CollisionDetector::DetectBoxPlane(
        const BoxCollider& box,
        const PlaneCollider& plane,
        std::vector<Contact>& contacts
    )
    {
        std::array<Vector3, 8> vertices;

        box.GetWorldVertices(vertices);


        for (const Vector3& vertex : vertices)
        {
            // Plane equation:
            //
            // n · x = offset
            //
            // signedDistance > 0 : normal 방향 쪽
            // signedDistance = 0 : plane 위
            // signedDistance < 0 : plane 안쪽으로 침투

            Real signedDistance =
                plane.normal.Dot(vertex)
                - plane.offset;


            if (signedDistance <= Real{ 0 })
            {
                Real penetration =
                    -signedDistance;


                // 침투한 vertex를 plane 위로 정사영한 점.
                //
                // projectedPoint
                // = vertex - signedDistance * normal
                //
                // signedDistance가 음수이므로
                // normal 방향으로 이동하게 된다.
                Vector3 contactPoint =
                    vertex
                    - plane.normal * signedDistance;


                Contact contact;

                contact.bodyA =
                    box.body;

                contact.bodyB =
                    nullptr;

                contact.setRestitution(box.restitution, plane.restitution);

                contact.point =
                    contactPoint;

                contact.normal =
                    plane.normal;

                contact.penetration =
                    penetration;


                contacts.push_back(contact);
            }
        }
    }
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    void CollisionDetector::DetectBoxSphere(
        const BoxCollider& box,
        const SphereCollider& sphere,
        std::vector<Contact>& contacts
    )
    {
        assert(box.body != nullptr);
        assert(sphere.body != nullptr);

        if (box.body == sphere.body)
        {
            return;
        }

        // 1. 구 중심을 박스의 local space로 변환
        Vector3 center = sphere.body->position;

        Matrix3 rotation =
            box.body->orientation.ToMatrix3();

        Vector3 relCenter =
            rotation.Transposed()
            * (center - box.body->position);


        // 2. 빠른 충돌 배제 검사
        if (
            std::abs(relCenter.x) > box.halfSize.x + sphere.radius ||
            std::abs(relCenter.y) > box.halfSize.y + sphere.radius ||
            std::abs(relCenter.z) > box.halfSize.z + sphere.radius
            )
        {
            return;
        }


        // 3. 구 중심이 박스 내부 또는 표면에 있는지 검사
        bool centerInside =
            std::abs(relCenter.x) <= box.halfSize.x &&
            std::abs(relCenter.y) <= box.halfSize.y &&
            std::abs(relCenter.z) <= box.halfSize.z;


        Contact contact;

        contact.bodyA = box.body;
        contact.bodyB = sphere.body;
        contact.setRestitution(box.restitution, sphere.restitution);

        if (!centerInside)
        {
            // 4-1. 구 중심이 박스 외부에 있는 경우

            // 박스에서 구 중심에 가장 가까운 점
            Vector3 closestPoint(
                std::clamp(
                    relCenter.x,
                    -box.halfSize.x,
                    box.halfSize.x
                ),
                std::clamp(
                    relCenter.y,
                    -box.halfSize.y,
                    box.halfSize.y
                ),
                std::clamp(
                    relCenter.z,
                    -box.halfSize.z,
                    box.halfSize.z
                )
            );

            Vector3 delta = closestPoint - relCenter;

            Real distanceSquared = delta.LengthSquared();
            Real radiusSquared = sphere.radius * sphere.radius;

            if (distanceSquared > radiusSquared)
            {
                return;
            }

            Real distance = std::sqrt(distanceSquared);

            contact.point =
                box.body->position + rotation * closestPoint;

            // Sphere(B) -> Box(A)
            contact.normal =
                rotation * (delta / distance);

            contact.penetration =
                sphere.radius - distance;
        }
        else
        {
            // 4-2. 구 중심이 박스 내부 또는 표면에 있는 경우

            // 각 축에서 가장 가까운 면까지의 거리
            Real dx =
                box.halfSize.x - std::abs(relCenter.x);

            Real dy =
                box.halfSize.y - std::abs(relCenter.y);

            Real dz =
                box.halfSize.z - std::abs(relCenter.z);

            Real minDistance;
            Vector3 localNormal;


            // 가장 가까운 면 선택
            if (dx <= dy && dx <= dz)
            {
                minDistance = dx;

                localNormal = Vector3(
                    relCenter.x >= Real{ 0 } ? Real{ -1 } : Real{ 1 },
                    0, 0
                );
            }
            else if (dy <= dz)
            {
                minDistance = dy;

                localNormal = Vector3(
                    0,
                    relCenter.y >= Real{ 0 } ? Real{ -1 } : Real{ 1 },
                    0
                );
            }
            else
            {
                minDistance = dz;

                localNormal = Vector3(
                    0, 0,
                    relCenter.z >= Real{ 0 } ? Real{ -1 } : Real{ 1 }
                );
            }


            // 선택한 박스 면 위의 접촉점
            Vector3 closestPoint =
                relCenter - localNormal * minDistance;

            contact.point =
                box.body->position + rotation * closestPoint;

            // Sphere(B) -> Box(A)
            contact.normal =
                rotation * localNormal;

            contact.penetration =
                sphere.radius + minDistance;
        }


        contacts.push_back(contact);
    }
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

namespace {

constexpr Real kAxisParallelSq = Real{1e-8};
constexpr Real kContactTolerance = Real{1e-5};

Real Extent(const Vector3& v, int axis)
{
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

Real Clamp01(Real v)
{
    return std::max(Real{0}, std::min(Real{1}, v));
}

struct OrientedBox
{
    const BoxCollider& collider;
    Matrix3 rotation;
    Vector3 axes[3];

    explicit OrientedBox(const BoxCollider& b)
        : collider(b), rotation(b.body->orientation.ToMatrix3()), axes{
            Vector3(rotation.m00, rotation.m10, rotation.m20),
            Vector3(rotation.m01, rotation.m11, rotation.m21),
            Vector3(rotation.m02, rotation.m12, rotation.m22) }
    {
    }

    Real ProjectRadius(const Vector3& axis) const
    {
        return collider.halfSize.x * std::abs(axes[0].Dot(axis))
             + collider.halfSize.y * std::abs(axes[1].Dot(axis))
             + collider.halfSize.z * std::abs(axes[2].Dot(axis));
    }

    Vector3 WorldToLocal(const Vector3& p) const
    {
        return rotation.Transposed() * (p - collider.body->position);
    }
};

struct CandidatePoint
{
    Vector3 worldPoint;
    Vector3 pointA;
    Vector3 pointB;
    Real penetration;
};

// Sutherland-Hodgman: keep points where dot(n, p-origin) <= limit.
void ClipAgainstPlane(
    const std::vector<Vector3>& input,
    std::vector<Vector3>& output,
    const Vector3& n,
    const Vector3& origin,
    Real limit)
{
    output.clear();
    if (input.empty()) return;

    Vector3 prev = input.back();
    Real prevDistance = n.Dot(prev - origin) - limit;
    bool prevInside = prevDistance <= kContactTolerance;

    for (const Vector3& current : input)
    {
        const Real currentDistance = n.Dot(current - origin) - limit;
        const bool currentInside = currentDistance <= kContactTolerance;

        if (prevInside != currentInside)
        {
            const Real denominator = prevDistance - currentDistance;
            if (std::abs(denominator) > Real{1e-12})
            {
                const Real t = Clamp01(prevDistance / denominator);
                output.push_back(prev + (current - prev) * t);
            }
        }

        if (currentInside) output.push_back(current);
        prev = current;
        prevDistance = currentDistance;
        prevInside = currentInside;
    }
}

// Segment-segment closest points, including clamping to the finite edges.
void ClosestPointsOnSegments(
    const Vector3& p1, const Vector3& q1,
    const Vector3& p2, const Vector3& q2,
    Vector3& outA, Vector3& outB)
{
    const Vector3 d1 = q1 - p1;
    const Vector3 d2 = q2 - p2;
    const Vector3 r = p1 - p2;
    const Real a = d1.Dot(d1);
    const Real e = d2.Dot(d2);
    const Real b = d1.Dot(d2);
    const Real c = d1.Dot(r);
    const Real f = d2.Dot(r);
    const Real denominator = a * e - b * b;

    Real s = Real{0};
    if (denominator > Real{1e-12})
        s = Clamp01((b * f - c * e) / denominator);

    Real t = (b * s + f) / e;
    if (t < Real{0})
    {
        t = Real{0};
        s = Clamp01(-c / a);
    }
    else if (t > Real{1})
    {
        t = Real{1};
        s = Clamp01((b - c) / a);
    }

    outA = p1 + d1 * s;
    outB = p2 + d2 * t;
}

// Preserve the deepest contact while distributing the remaining 3 across
// the patch. This is a small, deterministic geometric reduction heuristic.
void ReduceCandidates(
    const std::vector<CandidatePoint>& candidates,
    std::vector<CandidatePoint>& reduced,
    Real duplicateDistanceSquared)
{
    std::vector<CandidatePoint> unique;
    unique.reserve(candidates.size());
    for (const CandidatePoint& candidate : candidates)
    {
        bool merged = false;
        for (CandidatePoint& existing : unique)
        {
            if ((existing.worldPoint - candidate.worldPoint).LengthSquared()
                <= duplicateDistanceSquared)
            {
                if (candidate.penetration > existing.penetration)
                    existing = candidate;
                merged = true;
                break;
            }
        }
        if (!merged) unique.push_back(candidate);
    }

    reduced.clear();
    if (unique.empty()) return;
    if (unique.size() <= ContactManifold::MaxPoints)
    {
        reduced = std::move(unique);
        return;
    }

    bool used[16] = {};
    // Clipping a quad against four planes produces at most 8 vertices.
    assert(unique.size() <= 16);
    std::size_t deepest = 0;
    for (std::size_t i = 1; i < unique.size(); ++i)
        if (unique[i].penetration > unique[deepest].penetration) deepest = i;

    reduced.push_back(unique[deepest]);
    used[deepest] = true;

    // Farthest point, farthest from the resulting line, then farthest from
    // the selected points (maximizing the minimum separation).
    while (reduced.size() < ContactManifold::MaxPoints)
    {
        Real bestScore = Real{-1};
        std::size_t best = unique.size();
        for (std::size_t i = 0; i < unique.size(); ++i)
        {
            if (used[i]) continue;
            const Vector3 p = unique[i].worldPoint;
            Real score = Real{0};
            if (reduced.size() == 1)
            {
                score = (p - reduced[0].worldPoint).LengthSquared();
            }
            else if (reduced.size() == 2)
            {
                const Vector3 d = reduced[1].worldPoint - reduced[0].worldPoint;
                const Vector3 v = p - reduced[0].worldPoint;
                const Real d2 = d.LengthSquared();
                score = d2 > Real{1e-12}
                    ? v.Cross(d).LengthSquared() / d2
                    : v.LengthSquared();
            }
            else
            {
                score = std::numeric_limits<Real>::max();
                for (const CandidatePoint& selected : reduced)
                    score = std::min(score, (p - selected.worldPoint).LengthSquared());
            }

            if (score > bestScore)
            {
                bestScore = score;
                best = i;
            }
        }
        if (best == unique.size()) break;
        reduced.push_back(unique[best]);
        used[best] = true;
    }
}

void AddCandidateToManifold(
    const CandidatePoint& c,
    const OrientedBox& a,
    const OrientedBox& b,
    ContactManifold& manifold)
{
    if (manifold.pointCount >= ContactManifold::MaxPoints) return;
    ManifoldPoint& out = manifold.points[manifold.pointCount++];
    out.worldPoint = c.worldPoint;
    out.localPointA = a.WorldToLocal(c.pointA);
    out.localPointB = b.WorldToLocal(c.pointB);
    out.penetration = std::max(c.penetration, Real{0});
}

} // namespace

bool CollisionDetector::BuildBoxBoxManifold(
    const BoxCollider& boxA,
    const BoxCollider& boxB,
    ContactManifold& manifold)
{
    manifold = ContactManifold{};
    assert(boxA.body != nullptr);
    assert(boxB.body != nullptr);
    if (boxA.body == nullptr || boxB.body == nullptr || boxA.body == boxB.body)
        return false;

    const OrientedBox a(boxA);
    const OrientedBox b(boxB);
    const Vector3 centerDelta = boxB.body->position - boxA.body->position;

    Real bestFace = std::numeric_limits<Real>::max();
    int faceIndex = -1;  // A axes: 0-2; B axes: 3-5.
    Real bestEdge = std::numeric_limits<Real>::max();
    int edgeIndex = -1;  // (i * 3 + j), with axes A[i] and B[j].

    auto testAxis = [&](const Vector3& unitAxis, int index, bool edge) -> bool
    {
        const Real overlap = a.ProjectRadius(unitAxis) + b.ProjectRadius(unitAxis)
            - std::abs(centerDelta.Dot(unitAxis));
        if (overlap < -kContactTolerance) return false;
        if (edge)
        {
            if (overlap < bestEdge) { bestEdge = overlap; edgeIndex = index; }
        }
        else
        {
            if (overlap < bestFace) { bestFace = overlap; faceIndex = index; }
        }
        return true;
    };

    // All 15 SAT separating axes must be tested, even if the face axes overlap.
    for (int i = 0; i < 3; ++i)
        if (!testAxis(a.axes[i], i, false)) return false;
    for (int i = 0; i < 3; ++i)
        if (!testAxis(b.axes[i], 3 + i, false)) return false;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Vector3 axis = a.axes[i].Cross(b.axes[j]);
            const Real len2 = axis.LengthSquared();
            if (len2 <= kAxisParallelSq) continue;
            axis /= std::sqrt(len2);
            if (!testAxis(axis, 3 * i + j, true)) return false;
        }
    }

    manifold.bodyA = boxA.body;
    manifold.bodyB = boxB.body;
    manifold.restitution = (boxA.restitution + boxB.restitution) * Real{0.5};

    // Prefer a face manifold unless an edge axis is clearly better.
    // This avoids face/edge jitter from near-equal SAT overlap values.
    const bool useEdge = edgeIndex >= 0 &&
        bestEdge < bestFace * Real{0.95} - Real{1e-4};

    if (useEdge)
    {
        const int i = edgeIndex / 3;
        const int j = edgeIndex % 3;
        Vector3 normal = a.axes[i].Cross(b.axes[j]);
        normal.NormalizeUnsafe();
        if (normal.Dot(centerDelta) > Real{0}) normal = -normal;
        manifold.normal = normal; // B -> A

        // Choose the two actual edges facing the opposing shape.
        Vector3 edgeCenterA = boxA.body->position;
        Vector3 edgeCenterB = boxB.body->position;
        for (int k = 0; k < 3; ++k)
        {
            if (k != i)
            {
                const Real signA = a.axes[k].Dot(-normal) >= Real{0} ? Real{1} : Real{-1};
                edgeCenterA += a.axes[k] * (signA * Extent(boxA.halfSize, k));
            }
            if (k != j)
            {
                const Real signB = b.axes[k].Dot(normal) >= Real{0} ? Real{1} : Real{-1};
                edgeCenterB += b.axes[k] * (signB * Extent(boxB.halfSize, k));
            }
        }

        const Vector3 da = a.axes[i] * Extent(boxA.halfSize, i);
        const Vector3 db = b.axes[j] * Extent(boxB.halfSize, j);
        Vector3 pA, pB;
        ClosestPointsOnSegments(edgeCenterA - da, edgeCenterA + da,
                                edgeCenterB - db, edgeCenterB + db, pA, pB);
        const CandidatePoint candidate{
            (pA + pB) * Real{0.5}, pA, pB, std::max(bestEdge, Real{0})
        };
        AddCandidateToManifold(candidate, a, b, manifold);
        return true;
    }

    // Face contact: choose reference face, then clip the incident face quad
    // against the four side planes of the reference face.
    const bool referenceIsA = faceIndex < 3;
    const OrientedBox& ref = referenceIsA ? a : b;
    const OrientedBox& inc = referenceIsA ? b : a;
    const int refAxis = referenceIsA ? faceIndex : faceIndex - 3;
    const Vector3 towardIncident = referenceIsA ? centerDelta : -centerDelta;
    const Real refSign = ref.axes[refAxis].Dot(towardIncident) >= Real{0}
        ? Real{1} : Real{-1};
    const Vector3 refNormal = ref.axes[refAxis] * refSign; // Ref -> Incident
    const Vector3 refFaceCenter = ref.collider.body->position
        + refNormal * Extent(ref.collider.halfSize, refAxis);
    manifold.normal = referenceIsA ? -refNormal : refNormal; // B -> A

    int incidentAxis = 0;
    Real maxDot = std::abs(inc.axes[0].Dot(refNormal));
    for (int k = 1; k < 3; ++k)
    {
        const Real d = std::abs(inc.axes[k].Dot(refNormal));
        if (d > maxDot) { maxDot = d; incidentAxis = k; }
    }
    const Real incSign = inc.axes[incidentAxis].Dot(refNormal) > Real{0}
        ? Real{-1} : Real{1};
    const Vector3 incFaceCenter = inc.collider.body->position
        + inc.axes[incidentAxis] * (incSign * Extent(inc.collider.halfSize, incidentAxis));
    const int u = (incidentAxis + 1) % 3;
    const int v = (incidentAxis + 2) % 3;
    const Vector3 du = inc.axes[u] * Extent(inc.collider.halfSize, u);
    const Vector3 dv = inc.axes[v] * Extent(inc.collider.halfSize, v);

    std::vector<Vector3> polygon = {
        incFaceCenter - du - dv,
        incFaceCenter + du - dv,
        incFaceCenter + du + dv,
        incFaceCenter - du + dv
    };
    std::vector<Vector3> output;
    output.reserve(16);
    const int ru = (refAxis + 1) % 3;
    const int rv = (refAxis + 2) % 3;
    for (int axis : {ru, rv})
    {
        const Real extent = Extent(ref.collider.halfSize, axis);
        ClipAgainstPlane(polygon, output, ref.axes[axis], refFaceCenter, extent);
        polygon.swap(output);
        ClipAgainstPlane(polygon, output, -ref.axes[axis], refFaceCenter, extent);
        polygon.swap(output);
    }

    std::vector<CandidatePoint> candidates;
    candidates.reserve(polygon.size());
    for (const Vector3& incidentPoint : polygon)
    {
        const Real separation = refNormal.Dot(incidentPoint - refFaceCenter);
        if (separation > kContactTolerance) continue;
        const Vector3 referencePoint = incidentPoint - refNormal * separation;
        const Vector3 pointA = referenceIsA ? referencePoint : incidentPoint;
        const Vector3 pointB = referenceIsA ? incidentPoint : referencePoint;
        candidates.push_back(CandidatePoint{
            (referencePoint + incidentPoint) * Real{0.5},
            pointA, pointB,
            std::max(-separation, Real{0})
        });
    }

    if (candidates.empty())
    {
        // SAT says the shapes touch; at near-degenerate numerical configurations
        // clipping may produce no retained points. Do NOT fabricate a point.
        manifold = ContactManifold{};
        return false;
    }

    const Real minExtent = std::min({
        boxA.halfSize.x, boxA.halfSize.y, boxA.halfSize.z,
        boxB.halfSize.x, boxB.halfSize.y, boxB.halfSize.z });
    const Real mergeTol = std::max(Real{1e-5}, minExtent * Real{1e-4});
    std::vector<CandidatePoint> reduced;
    ReduceCandidates(candidates, reduced, mergeTol * mergeTol);
    for (const CandidatePoint& point : reduced)
        AddCandidateToManifold(point, a, b, manifold);

    return manifold.pointCount != 0;
}

void CollisionDetector::DetectBoxBox(
    const BoxCollider& boxA,
    const BoxCollider& boxB,
    std::vector<Contact>& contacts)
{
    ContactManifold manifold;
    if (!BuildBoxBoxManifold(boxA, boxB, manifold)) return;

    // Legacy adapter. Keep the existing PE::ContactResolver and DX11 GUI
    // unmodified. The new PGS solver will consume manifolds directly later.
    for (std::size_t i = 0; i < manifold.pointCount; ++i)
    {
        const ManifoldPoint& mp = manifold.points[i];
        Contact contact;
        contact.bodyA = manifold.bodyA;
        contact.bodyB = manifold.bodyB;
        contact.normal = manifold.normal;
        contact.point = mp.worldPoint;
        contact.penetration = mp.penetration;
        contact.setRestitution(boxA.restitution, boxB.restitution);
        contacts.push_back(contact);
    }
}

} // namespace PE
