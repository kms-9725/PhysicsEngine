#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

#include "../physics/ForceRegistry.h"
#include "../physics/CenterSpringForce.h"
#include "../collision/BoxCollider.h"
#include "../collision/SphereCollider.h"
#include "../collision/PlaneCollider.h"
#include "../collision/CollisionDetector.h"
#include "../collision/ContactResolver.h"
// Step 3 PersistentManifoldCache is OPTIONAL. The PGS-only project works too.
#if defined(__has_include)
#  if __has_include("../collision/PersistentManifoldCache.h")
#    include "../collision/PersistentManifoldCache.h"
#    define PE_TEST_LAB_HAS_PERSISTENT 1
#  endif
#endif
#ifndef PE_TEST_LAB_HAS_PERSISTENT
namespace PE
{
    class PersistentManifoldCache
    {
    public:
        struct Statistics
        {
            std::size_t activeManifolds = 0;
            std::size_t reusedPoints = 0;
            std::size_t addedPoints = 0;
            std::size_t detectedPoints = 0;
            std::size_t reducedPoints = 0;
        };
        void Clear() {}
        void Update(std::vector<Contact>&) {}
        const Statistics& GetStatistics() const { return statistics_; }
    private:
        Statistics statistics_{};
    };
}
#endif

namespace Demo
{
    enum Scenario : int
    {
        Gallery = 0, BoxFloor, SphereFloor, SphereSphere, BoxSphere, BoxBox,
        FourStack, Mixed, FreeRotation, TallStack, Pyramid, SpringTest,
        ScenarioCount
    };

    struct EnergyReading
    {
        double time = 0;
        double energy = 0;
        double driftPercent = 0;
        double angularMomentumDriftPercent = 0;
    };

    struct SandboxScene
    {
        static constexpr std::size_t MaxBodies = 1000;
        std::deque<PE::RigidBody> bodies;  // Stable addresses while appending.
        std::vector<PE::BoxCollider> boxes;
        std::vector<PE::SphereCollider> spheres;
        std::vector<PE::Contact> contacts;
        std::vector<PE::Contact> debugContacts;
        PE::ContactResolver solver;
        PE::PersistentManifoldCache manifolds;

        // World-level force routing. Generators and registered bodies have stable lifetime.
        PE::GravityForce gravityGenerator;
        PE::PointSpringForce grabGenerator;
        PE::ForceRegistry forces;
        PE::RigidBody* grabbedBody = nullptr;
        // Independent force generator for the undamped mass-spring diagnostic.
        PE::CenterSpringForce springGenerator;
        PE::RigidBody* springBody = nullptr;
        PE::Real springStiffness = PE::Real{10};
        PE::Real springInitialDisplacement = PE::Real{2.5f};

        PE::Real friction = PE::Real{0.65f};
        PE::PlaneCollider ground{PE::Vector3(0, 1, 0), PE::Real{0}, PE::Real{0.15f}};
        PE::RigidBody groundVisual;
        PE::Vector3 groundHalfSize{24.0f, 0.12f, 12.0f};
        PE::Real restitution = PE::Real{0.15f};
        PE::Real gravity = PE::Real{9.8f};
        PE::LinearIntegrationMethod linearMethod = PE::LinearIntegrationMethod::SemiImplicitEuler;
        PE::AngularIntegrationMethod angularMethod = PE::AngularIntegrationMethod::StrangSplitting;
        // Optional stress-test performance mode: reduced solver iterations.
        // Use full quality to evaluate stacking stability quantitatively.
        bool fastSolver = false;
        int preset = Gallery;
        int objectCount = 12;
        double simulationTime = 0;
        int physicsSteps = 0;
        std::size_t broadPhasePairs = 0;
        std::size_t narrowPhasePairs = 0;
        double referenceEnergy = 0;
        double referenceAngularMomentum = 0;
        double currentEnergy = 0;
        double energyErrorPercent = 0;
        double angularMomentumErrorPercent = 0;
        double maxAbsoluteEnergyErrorPercent = 0;
        std::deque<EnergyReading> energyHistory;

        SandboxScene()
        {
            boxes.reserve(MaxBodies);
            spheres.reserve(MaxBodies);
            contacts.reserve(1600);
            debugContacts.reserve(1600);
            groundVisual.position = PE::Vector3(0, PE::Real{-0.12f}, 0);
        }

        bool MaySpawn() const { return bodies.size() < MaxBodies; }
        bool HasGround() const { return preset != FreeRotation && preset != SpringTest; }
        bool IsDiagnostic() const { return preset == FreeRotation || preset == SpringTest; }
        bool IsFreeRotation() const { return preset == FreeRotation; }
        bool IsSpringTest() const { return preset == SpringTest; }
        double SpringDisplacement() const
        {
            return springBody ? static_cast<double>(springBody->position.x - springGenerator.equilibrium.x) : 0.0;
        }
        double SpringKineticEnergy() const
        {
            if (!springBody) return 0.0;
            return 0.5 * static_cast<double>(springBody->GetMass()) *
                static_cast<double>(springBody->linearVelocity.LengthSquared());
        }
        double SpringPotentialEnergy() const
        {
            return springBody ? static_cast<double>(springGenerator.PotentialEnergy(*springBody)) : 0.0;
        }
        double SpringAnalyticPositionError() const
        {
            if (!springBody || springGenerator.stiffness <= PE::Real{0}) return 0.0;
            const double m = static_cast<double>(springBody->GetMass());
            const double omega = std::sqrt(static_cast<double>(springGenerator.stiffness) / m);
            const double analytic = static_cast<double>(springInitialDisplacement) * std::cos(omega * simulationTime);
            return SpringDisplacement() - analytic;
        }

        PE::RigidBody* AddBox(const PE::Vector3& position,
                             const PE::Vector3& halfSize,
                             const PE::Vector3& velocity = PE::Vector3(),
                             const PE::Vector3& tilt = PE::Vector3(),
                             PE::Real mass = PE::Real{1})
        {
            if (!MaySpawn()) return nullptr;
            bodies.emplace_back();
            PE::RigidBody& body = bodies.back();
            body.position = position;
            boxes.emplace_back(&body, halfSize, restitution);
            boxes.back().InitMassProperties(mass);
            if (!tilt.IsNearlyZero()) body.RotateByRotationVector(tilt);
            body.linearVelocity = velocity;
            forces.Add(body, gravityGenerator);
            return &body;
        }

        PE::RigidBody* AddSphere(const PE::Vector3& position, PE::Real radius,
                                const PE::Vector3& velocity = PE::Vector3(),
                                PE::Real mass = PE::Real{1})
        {
            if (!MaySpawn()) return nullptr;
            bodies.emplace_back();
            PE::RigidBody& body = bodies.back();
            body.position = position;
            spheres.emplace_back(&body, radius, restitution);
            spheres.back().InitMassProperties(mass);
            body.linearVelocity = velocity;
            forces.Add(body, gravityGenerator);
            return &body;
        }

        void SetGravity(PE::Real g)
        {
            gravity = g;
            gravityGenerator.acceleration = PE::Vector3(0, (preset == FreeRotation || preset == SpringTest) ? PE::Real{0} : -gravity, 0);
        }
        void SetRestitution(PE::Real value)
        {
            restitution = value;
            ground.restitution = value;
            for (auto& box : boxes) box.restitution = value;
            for (auto& sphere : spheres) sphere.restitution = value;
        }
        void EndGrab()
        {
            forces.RemoveGenerator(grabGenerator);
            grabbedBody = nullptr;
        }
        void StartGrab(PE::RigidBody& body, const PE::Vector3& localAnchor,
                       const PE::Vector3& target)
        {
            EndGrab();
            if (IsDiagnostic()) return;
            grabGenerator.localAnchor = localAnchor;
            grabGenerator.target = target;
            grabbedBody = &body;
            forces.Add(body, grabGenerator);
        }
        void MoveGrab(const PE::Vector3& target) { grabGenerator.target = target; }

        void Reset(int nextPreset, PE::Real initialSpeed, PE::Real height, int count)
        {
            // Drop every non-owning pointer before deleting bodies.
            EndGrab();
            forces.Clear();
            solver.ClearCache();
            manifolds.Clear();
            springBody = nullptr;
            boxes.clear();
            spheres.clear();
            bodies.clear();
            contacts.clear();
            debugContacts.clear();
            preset = nextPreset;
            objectCount = std::clamp(count, 4, 40);
            simulationTime = 0.0;
            physicsSteps = 0;
            broadPhasePairs = 0;
            narrowPhasePairs = 0;
            energyHistory.clear();
            referenceEnergy = referenceAngularMomentum = 0;
            currentEnergy = energyErrorPercent = angularMomentumErrorPercent = 0;
            maxAbsoluteEnergyErrorPercent = 0;
            SetGravity(gravity);
            const PE::Real h = height;
            const PE::Real v = initialSpeed;
            switch (preset)
            {
            case Gallery:
                AddBox(PE::Vector3(-5.1f, h, 0), PE::Vector3(.5f, .5f, .5f),
                       PE::Vector3(), PE::Vector3(.12f, 0, .2f));
                AddSphere(PE::Vector3(-2.55f, h, 0), .5f);
                AddSphere(PE::Vector3(0, .52f, -1.6f), .5f, PE::Vector3(0, 0, v));
                AddSphere(PE::Vector3(0, .52f, 1.6f), .5f, PE::Vector3(0, 0, -v));
                AddBox(PE::Vector3(2.55f, .52f, 0), PE::Vector3(.52f, .52f, .52f));
                AddSphere(PE::Vector3(2.55f, h, 0), .5f);
                AddBox(PE::Vector3(5.1f, .52f, 0), PE::Vector3(.52f, .52f, .52f));
                AddBox(PE::Vector3(5.1f, h, 0), PE::Vector3(.48f, .48f, .48f),
                       PE::Vector3(), PE::Vector3(.1f, 0, .07f));
                break;
            case BoxFloor:
                AddBox(PE::Vector3(0,h,0), PE::Vector3(.7f,.5f,.55f),
                       PE::Vector3(), PE::Vector3(.2f,0,.3f)); break;
            case SphereFloor:
                AddSphere(PE::Vector3(0,h,0), .6f); break;
            case SphereSphere:
                AddSphere(PE::Vector3(-1.8f,.62f,0), .6f, PE::Vector3(v,0,0));
                AddSphere(PE::Vector3(1.8f,.62f,0), .6f, PE::Vector3(-v,0,0)); break;
            case BoxSphere:
                AddBox(PE::Vector3(0,.52f,0), PE::Vector3(.52f,.52f,.52f));
                AddSphere(PE::Vector3(0,h,0), .55f, PE::Vector3(v*.15f,0,0)); break;
            case BoxBox:
                AddBox(PE::Vector3(0,.52f,0), PE::Vector3(.52f,.52f,.52f));
                AddBox(PE::Vector3(0,h,0), PE::Vector3(.5f,.5f,.5f),
                       PE::Vector3(v*.1f,0,0), PE::Vector3(0,0,.2f)); break;
            case FourStack:
                for (int i=0;i<4;++i)
                    AddBox(PE::Vector3(0,.52f+1.04f*i,0),PE::Vector3(.5f,.5f,.5f));
                break;
            case Mixed:
                for (int i=0;i<5;++i)
                {
                    const PE::Real x=-2.6f+1.3f*i;
                    if ((i%2)==0)
                        AddBox(PE::Vector3(x,h+1.4f*i,0),PE::Vector3(.5f,.5f,.5f),
                               PE::Vector3((2-i)*.3f,0,0),PE::Vector3(0,.1f,.13f));
                    else AddSphere(PE::Vector3(x,h+1.4f*i,0),.55f,
                                   PE::Vector3((2-i)*.3f,0,0));
                } break;
            case FreeRotation:
            {
                // Principal inertias for these non-cubic dimensions: Iy is intermediate.
                PE::RigidBody* body = AddBox(PE::Vector3(0,1.5f,0),
                    PE::Vector3(.4f,.7f,1.1f));
                if (body) body->SetAngularVelocity(PE::Vector3(.08f, 5.5f, .11f));
                break;
            }
            case TallStack:
                for (int i=0;i<objectCount;++i)
                    AddBox(PE::Vector3(0,.505f+1.005f*i,0),
                           PE::Vector3(.5f,.5f,.5f));
                break;
            case Pyramid:
            {
                // One-box-thick pyramid in X/Y; each row is half a brick offset.
                // The bottom row has `objectCount` bricks, top row has one.
                const PE::Real halfX=.24f, halfY=.24f, halfZ=.18f;
                const PE::Real pitch=PE::Real{.486f};
                for(int row=0; row<objectCount; ++row)
                {
                    const int columns=objectCount-row;
                    const PE::Real y=halfY + row*pitch;
                    for (int col=0; col<columns; ++col)
                    {
                        const PE::Real x=(col-PE::Real{.5f}*(columns-1))*pitch;
                        AddBox(PE::Vector3(x,y,0),PE::Vector3(halfX,halfY,halfZ),
                               PE::Vector3(),PE::Vector3(),PE::Real{.25f});
                    }
                }
                break;
            }
            case SpringTest:
            {
                // A one-dimensional ideal harmonic oscillator:
                //    m = 1 kg,  F_x = -k*x,  x(0) = 2.5 m, v(0) = 0.
                // No damping, gravity, collision response, or joint projection.
                springGenerator.equilibrium = PE::Vector3(0, PE::Real{2.2f}, 0);
                springGenerator.stiffness = springStiffness;
                springBody = AddSphere(
                    springGenerator.equilibrium + PE::Vector3(springInitialDisplacement, 0, 0),
                    PE::Real{.32f}, PE::Vector3(), PE::Real{1});
                if (springBody)
                    forces.Add(*springBody, springGenerator);
                break;
            }
            default: break;
            }
            CaptureReferenceEnergy();
        }

        void CaptureReferenceEnergy()
        {
            energyHistory.clear();
            maxAbsoluteEnergyErrorPercent=0;
            referenceEnergy=ComputeEnergy();
            if (IsFreeRotation() && !bodies.empty())
                referenceAngularMomentum=bodies.front().angularMomentum.Length();
            else referenceAngularMomentum=0;
            RecordEnergy();
        }
        double ComputeEnergy() const
        {
            if (!IsDiagnostic() || bodies.empty()) return 0;
            const PE::RigidBody& b = bodies.front();
            const double mass=static_cast<double>(b.GetMass());
            const double kinetic=.5*mass*static_cast<double>(b.linearVelocity.LengthSquared());
            const double rotational=.5*static_cast<double>(b.angularVelocity.Dot(b.angularMomentum));
            if (IsFreeRotation()) return kinetic+rotational;
            if (IsSpringTest())
                return kinetic + rotational + SpringPotentialEnergy();
            return 0;
        }
        void RecordEnergy()
        {
            if (!IsDiagnostic()) return;
            currentEnergy=ComputeEnergy();
            if (std::abs(referenceEnergy)>1e-12)
                energyErrorPercent=100.0*(currentEnergy-referenceEnergy)/std::abs(referenceEnergy);
            else energyErrorPercent=0;
            if (IsFreeRotation() && referenceAngularMomentum>1e-12)
                angularMomentumErrorPercent=100.0*(bodies.front().angularMomentum.Length()-
                    referenceAngularMomentum)/referenceAngularMomentum;
            else angularMomentumErrorPercent=0;
            if (std::isfinite(energyErrorPercent))
                maxAbsoluteEnergyErrorPercent=std::max(maxAbsoluteEnergyErrorPercent,
                    std::abs(energyErrorPercent));
            if (physicsSteps % 3 == 0 || energyHistory.empty())
            {
                energyHistory.push_back({simulationTime,currentEnergy,
                                         energyErrorPercent,angularMomentumErrorPercent});
                if(energyHistory.size()>650) energyHistory.pop_front();
            }
        }

        // Integrator selection belongs to the physics RigidBody, not to the sandbox.
        void IntegrateBody(PE::RigidBody& b, PE::Real dt)
        {
            b.Integrate(dt, linearMethod, angularMethod);
        }

        struct AABB
        {
            PE::Vector3 lo,hi;
        };
        struct Proxy
        {
            AABB aabb{};
            std::size_t index=0;
            bool sphere=false;
        };
        static AABB Bounds(const PE::BoxCollider& box)
        {
            const auto& p=box.body->position;
            const PE::Matrix3 r=box.body->orientation.ToMatrix3();
            const auto& h=box.halfSize;
            const PE::Vector3 worldHalf(
                std::abs(r.m00)*h.x+std::abs(r.m01)*h.y+std::abs(r.m02)*h.z,
                std::abs(r.m10)*h.x+std::abs(r.m11)*h.y+std::abs(r.m12)*h.z,
                std::abs(r.m20)*h.x+std::abs(r.m21)*h.y+std::abs(r.m22)*h.z);
            return {p-worldHalf,p+worldHalf};
        }
        static AABB Bounds(const PE::SphereCollider& sphere)
        {
            const PE::Vector3 h(sphere.radius,sphere.radius,sphere.radius);
            return {sphere.body->position-h,sphere.body->position+h};
        }
        void DetectContacts()
        {
            contacts.clear();
            broadPhasePairs=0;
            narrowPhasePairs=0;
            if (!HasGround()) return;
            // Ground is infinite, so only y-bottom plausibly close to y=0 is tested.
            constexpr PE::Real margin=PE::Real{.015f};
            std::vector<Proxy> proxies;
            proxies.reserve(boxes.size()+spheres.size());
            for(std::size_t i=0;i<boxes.size();++i)
            {
                const AABB bounds=Bounds(boxes[i]);
                if(bounds.lo.y<=margin)
                    PE::CollisionDetector::DetectBoxPlane(boxes[i],ground,contacts);
                proxies.push_back({bounds,i,false});
            }
            for(std::size_t i=0;i<spheres.size();++i)
            {
                const AABB bounds=Bounds(spheres[i]);
                if(bounds.lo.y<=margin)
                    PE::CollisionDetector::DetectSpherePlane(spheres[i],ground,contacts);
                proxies.push_back({bounds,i,true});
            }
            // Conservative sweep-and-prune broad phase: no naive all-pairs SAT for 820 bricks.
            std::sort(proxies.begin(),proxies.end(),[](const Proxy& a,const Proxy& b){
                return a.aabb.lo.x<b.aabb.lo.x;
            });
            for(std::size_t i=0;i<proxies.size();++i)
            {
                const Proxy& a=proxies[i];
                for(std::size_t j=i+1;j<proxies.size() &&
                    proxies[j].aabb.lo.x <= a.aabb.hi.x;++j)
                {
                    const Proxy& b=proxies[j];
                    ++broadPhasePairs;
                    if(a.aabb.lo.y>b.aabb.hi.y || b.aabb.lo.y>a.aabb.hi.y ||
                       a.aabb.lo.z>b.aabb.hi.z || b.aabb.lo.z>a.aabb.hi.z) continue;
                    ++narrowPhasePairs;
                    if(!a.sphere && !b.sphere)
                        PE::CollisionDetector::DetectBoxBox(boxes[a.index],boxes[b.index],contacts);
                    else if(a.sphere && b.sphere)
                        PE::CollisionDetector::DetectSphereSphere(spheres[a.index],spheres[b.index],contacts);
                    else if(a.sphere)
                        PE::CollisionDetector::DetectBoxSphere(boxes[b.index],spheres[a.index],contacts);
                    else
                        PE::CollisionDetector::DetectBoxSphere(boxes[a.index],spheres[b.index],contacts);
                }
            }
        }
        void Step(PE::Real dt, PE::Real g)
        {
            SetGravity(g);
            forces.ApplyAll(dt);
            for(PE::RigidBody& body:bodies) IntegrateBody(body,dt);
            // No positional constraint for the spring oscillator.
            DetectContacts();
            for(PE::Contact& c:contacts) c.friction=friction;
            if(HasGround())
            {
                manifolds.Update(contacts);
                debugContacts=contacts;
                solver.ResolveContacts(contacts,
                    fastSolver ? 4u : 12u,
                    fastSolver ? 8u : 24u,dt);
            }
            else debugContacts.clear();
            simulationTime+=dt;
            ++physicsSteps;
            RecordEnergy();
        }
    };
}
