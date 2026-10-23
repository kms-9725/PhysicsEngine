#include "RigidBody.h"

#include <cassert>
#include <cmath>

namespace PE {

    namespace {

        // NextOmega(implicit)의 고정점 반복 횟수
        constexpr int kNextOmegaIterations = 10;

        // 외적 행렬 [v]× : [v]× a = v × a
        Matrix3 Skew(const Vector3& v)
        {
            return Matrix3(
                Real{ 0 }, -v.z, v.y,
                v.z, Real{ 0 }, -v.x,
                -v.y, v.x, Real{ 0 }
            );
        }

        Matrix3 Add(const Matrix3& a, const Matrix3& b)
        {
            return Matrix3(
                a.m00 + b.m00, a.m01 + b.m01, a.m02 + b.m02,
                a.m10 + b.m10, a.m11 + b.m11, a.m12 + b.m12,
                a.m20 + b.m20, a.m21 + b.m21, a.m22 + b.m22
            );
        }

        Matrix3 Subtract(const Matrix3& a, const Matrix3& b)
        {
            return Matrix3(
                a.m00 - b.m00, a.m01 - b.m01, a.m02 - b.m02,
                a.m10 - b.m10, a.m11 - b.m11, a.m12 - b.m12,
                a.m20 - b.m20, a.m21 - b.m21, a.m22 - b.m22
            );
        }

        Matrix3 Scale(const Matrix3& m, Real s)
        {
            return Matrix3(
                m.m00 * s, m.m01 * s, m.m02 * s,
                m.m10 * s, m.m11 * s, m.m12 * s,
                m.m20 * s, m.m21 * s, m.m22 * s
            );
        }

        Real Determinant(
            Real a, Real b, Real c,
            Real d, Real e, Real f,
            Real g, Real h, Real i)
        {
            return a * (e * i - f * h)
                - b * (d * i - f * g)
                + c * (d * h - e * g);
        }

        // J x = rhs 를 크래머 공식으로 푼다. 특이행렬이면 영벡터 반환
        Vector3 Solve3x3(const Matrix3& J, const Vector3& rhs)
        {
            const Real det = Determinant(
                J.m00, J.m01, J.m02,
                J.m10, J.m11, J.m12,
                J.m20, J.m21, J.m22);

            if (std::abs(det) < Real{ 1e-12f })
            {
                return Vector3();
            }

            const Real inverseDet = Real{ 1 } / det;

            return Vector3(
                Determinant(rhs.x, J.m01, J.m02,
                            rhs.y, J.m11, J.m12,
                            rhs.z, J.m21, J.m22) * inverseDet,
                Determinant(J.m00, rhs.x, J.m02,
                            J.m10, rhs.y, J.m12,
                            J.m20, rhs.z, J.m22) * inverseDet,
                Determinant(J.m00, J.m01, rhs.x,
                            J.m10, J.m11, rhs.y,
                            J.m20, J.m21, rhs.z) * inverseDet
            );
        }
    }


    RigidBody::RigidBody()
        : position(0, 0, 0),
        orientation(),
        linearVelocity(0, 0, 0),
        angularVelocity(0, 0, 0),
        angularMomentum(0, 0, 0),
        inverseMass(1),
        inertiaTensorBody(Matrix3::Identity()),
        inverseInertiaTensorBody(Matrix3::Identity()),
        inertiaTensorWorld(Matrix3::Identity()),
        inverseInertiaTensorWorld(Matrix3::Identity()),
        linearForceAccumulator(0, 0, 0),
        torqueAccumulator(0, 0, 0),
        lastFrameAcceleration(0, 0, 0)
    {
    }


    void RigidBody::AddLinearForce(const Vector3& force)
    {
        linearForceAccumulator += force;
    }


    void RigidBody::AddTorque(const Vector3& torque)
    {
        torqueAccumulator += torque;
    }


    void RigidBody::AddForceAtPoint(   //강체의 한 점에 작용한 힘을 linearForceAccumulator와 torqueAccumulator에 반영하는 함수. worldPoint는 힘이 작용한 점의 위치를 나타냄.
        const Vector3& force,
        const Vector3& worldPoint
    )
    {
        linearForceAccumulator += force;

        // 질량중심에서 힘이 작용한 점까지의 벡터
        // world space
        Vector3 r = worldPoint - position;

        // τ = r × F
        torqueAccumulator += r.Cross(force);
    }


    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // 적분
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    void RigidBody::Integrate(
        Real dt,
        LinearIntegrationMethod linearMethod,
        AngularIntegrationMethod angularMethod
    )
    {
        assert(dt > Real{ 0 });

        if (!HasFiniteMass())
        {
            lastFrameAcceleration = Vector3();
            ClearAccumulators();
            return;
        }

        // =====================================================
        // 1. 선형 운동
        // =====================================================

        IntegrateLinear(dt, linearMethod);


        // =====================================================
        // 2. 회전 운동
        // =====================================================

        switch (angularMethod)
        {
        case AngularIntegrationMethod::PreviousOmega:
            IntegrateAngularPreviousOmega(dt);
            break;
        case AngularIntegrationMethod::NextOmega:
            IntegrateAngularNextOmega(dt);
            break;
        case AngularIntegrationMethod::MidpointOmega:
            IntegrateAngularMidpointOmega(dt);
            break;
        case AngularIntegrationMethod::StrangSplitting:
            IntegrateAngularStrangSplitting(dt);
            break;
        case AngularIntegrationMethod::EulerEquation:
            IntegrateAngularEulerEquation(dt);
            break;
        case AngularIntegrationMethod::JoltStyle:
            IntegrateAngularJoltStyle(dt);
            break;
        case AngularIntegrationMethod::BulletStyle:
            IntegrateAngularBulletStyle(dt);
            break;
        case AngularIntegrationMethod::GyroscopicOff:
            IntegrateAngularGyroscopicOff(dt);
            break;
        }

        ClearAccumulators();
    }


    void RigidBody::IntegrateLinear(Real dt, LinearIntegrationMethod method)
    {
        Vector3 linearAcceleration =
            linearForceAccumulator * inverseMass;

        lastFrameAcceleration = linearAcceleration;

        if (method == LinearIntegrationMethod::ExplicitEuler)
        {
            // x(n+1) = x(n) + v(n) dt
            position += linearVelocity * dt;

            // v(n+1) = v(n) + a(n) dt
            linearVelocity += linearAcceleration * dt;
        }
        else
        {
            // v(n+1) = v(n) + a(n) dt
            linearVelocity += linearAcceleration * dt;

            // x(n+1) = x(n) + v(n+1) dt
            position += linearVelocity * dt;
        }
    }


    Vector3 RigidBody::ComputeAngularVelocityFromMomentum() const
    {
        return inverseInertiaTensorWorld * angularMomentum;
    }


    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // 회전: 각운동량 L 기반
    //
    // 공통: 1) L += τ dt   2) 자세 적분   3) ω = I_world^-1 L
    // 토크가 없으면 L은 바뀌지 않으므로 정확히 보존된다.
    // 방식별 차이는 "2) 자세 적분에서 어느 시점의 ω를 쓰느냐" 뿐이다.
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // 스텝 시작 자세의 ω로 한 스텝 회전 (1차 explicit)
    //
    // 한 스텝 동안 ω가 일정하다고 가정한다. 실제로는 회전하면서 I_world가 바뀌어 ω도 바뀐다.
    // body 좌표에서 L_b는 축 ω_n 둘레로 돌기 때문에 ω_n·L_b가 보존되고,
    // 이 평면은 에너지 타원체의 접평면이므로 매 스텝 E가 ½ΔLᵀI⁻¹ΔL 만큼 증가한다.
    // (토크가 없으면 기존 IntegrateOrientationExplicit과 같은 계산)
    void RigidBody::IntegrateAngularPreviousOmega(Real dt)
    {
        angularMomentum += torqueAccumulator * dt;

        // ω(n) = I_world(n)^-1 L(n+1)
        const Vector3 startOmega =
            ComputeAngularVelocityFromMomentum();

        RotateByRotationVector(startOmega * dt);

        SyncAngularVelocityFromMomentum();
    }

    // 스텝 끝 자세의 ω로 한 스텝 회전 (implicit, 고정점 반복)
    //
    // ω(n+1) = I_world(q(n+1))^-1 L,  q(n+1) = Δq(ω(n+1) dt) q(n)
    // 미지수 ω(n+1)이 양변에 있으므로 반복해서 푼다.
    // 같은 접평면 논리를 거꾸로 적용하면 E(n) = E(n+1) + ½ΔLᵀI⁻¹ΔL 이므로 매 스텝 E가 감소한다.
    void RigidBody::IntegrateAngularNextOmega(Real dt)
    {
        angularMomentum += torqueAccumulator * dt;

        const Quaternion start = orientation;

        // 초기 추정값: 시작 자세의 ω
        Vector3 omega =
            ComputeAngularVelocityFromMomentum();

        for (int iteration = 0; iteration < kNextOmegaIterations; ++iteration)
        {
            // 시작 자세로 되돌린 뒤 추정한 ω로 회전
            orientation = start;
            UpdateOrientationDependentData();
            RotateByRotationVector(omega * dt);

            // 그 끝 자세에서의 ω로 추정값 갱신
            omega = ComputeAngularVelocityFromMomentum();
        }

        orientation = start;
        UpdateOrientationDependentData();
        RotateByRotationVector(omega * dt);

        SyncAngularVelocityFromMomentum();
    }

    // 반 스텝 예측한 중간 자세의 ω로 한 스텝 회전 (explicit midpoint, RK2)
    //
    // 2차 정확도라 오차가 훨씬 작지만, symplectic이 아니라서 오래 돌리면 에너지 오차가 쌓인다.
    void RigidBody::IntegrateAngularMidpointOmega(Real dt)
    {
        angularMomentum += torqueAccumulator * dt;

        const Quaternion start = orientation;

        // 1) 시작 ω로 반 스텝 회전해서 중간 자세 예측
        const Vector3 startOmega =
            ComputeAngularVelocityFromMomentum();

        RotateByRotationVector(startOmega * (dt * Real{ 0.5 }));

        // 2) 중간 자세에서의 ω
        const Vector3 midOmega =
            ComputeAngularVelocityFromMomentum();

        // 3) 시작 자세로 돌아가 중간 ω로 한 스텝 회전
        //    (midOmega가 0이면 RotateByRotationVector가 바로 반환하므로 I_world를 먼저 복구)
        orientation = start;
        UpdateOrientationDependentData();
        RotateByRotationVector(midOmega * dt);

        SyncAngularVelocityFromMomentum();
    }

    // 주축별 정확한 회전을 대칭으로 합성 (Strang splitting)
    void RigidBody::IntegrateAngularStrangSplitting(Real dt)
    {
        angularMomentum += torqueAccumulator * dt;

        IntegrateOrientation(dt);

        SyncAngularVelocityFromMomentum();
    }


    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // 회전: 각속도 ω 기반
    //
    // 공통: 1) ω 갱신   2) 새 ω로 자세 적분 (semi-implicit)   3) L = I_world ω
    // 접촉 해결기가 L에 충격량을 더하고 ω를 다시 계산하므로, 끝에서 L을 ω에 맞춰 둔다.
    //
    // 참고: 고정축 ω로 회전하면 ω 방향은 그대로이므로
    //       회전 후에도 world ω와 body ω_b = Rᵀω 가 모두 변하지 않는다.
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // 오일러 운동방정식 (body 좌표)
    //
    //   I_b α_b = τ_b - ω_b × (I_b ω_b)
    //   ω_b(n+1) = ω_b(n) + α_b dt
    //
    // ω × Iω (자이로 항)는 ω 자신에 의존하므로, ω를 먼저 갱신해도 이 부분은 explicit Euler다.
    //   E(n+1) = E(n) + ½ΔωᵀIΔω  (ω·(ω×Iω) = 0 이므로 1차 항이 사라짐)
    //   |L(n+1)|² = |L(n)|² + |ΔL|²  (ΔL ⟂ L)
    // 그래서 에너지와 |L|이 모두 증가한다.
    void RigidBody::IntegrateAngularEulerEquation(Real dt)
    {
        const Matrix3 rotation = orientation.ToMatrix3();
        const Matrix3 rotationTranspose = rotation.Transposed();

        // world -> body
        const Vector3 bodyOmega = rotationTranspose * angularVelocity;
        const Vector3 bodyTorque = rotationTranspose * torqueAccumulator;
        const Vector3 bodyMomentum = inertiaTensorBody * bodyOmega;

        // α_b = I_b^-1 (τ_b - ω_b × I_b ω_b)
        const Vector3 bodyAngularAcceleration =
            inverseInertiaTensorBody
            * (bodyTorque - bodyOmega.Cross(bodyMomentum));

        // ω(n+1) = ω(n) + α dt
        const Vector3 newBodyOmega =
            bodyOmega + bodyAngularAcceleration * dt;

        // body -> world
        angularVelocity = rotation * newBodyOmega;

        // 새 ω로 자세 적분
        RotateByRotationVector(angularVelocity * dt);

        SyncAngularMomentumFromVelocity();
    }

    // Jolt Physics 방식 (MotionProperties::ApplyGyroscopicForceInternal 참고)
    //
    //   L_b' = L_b - dt ω_b × L_b        (explicit Euler, |L|이 커짐)
    //   L_b' ← L_b' · |L_b| / |L_b'|     (|L|을 원래 크기로 되돌림)
    //
    // |L|의 크기만 맞출 뿐 방향과 에너지는 정확히 보존하지 않는 간단한 보정.
    void RigidBody::IntegrateAngularJoltStyle(Real dt)
    {
        // 외부 토크: ω += I_world^-1 τ dt
        angularVelocity +=
            (inverseInertiaTensorWorld * torqueAccumulator) * dt;

        const Matrix3 rotation = orientation.ToMatrix3();
        const Matrix3 rotationTranspose = rotation.Transposed();

        const Vector3 bodyOmega = rotationTranspose * angularVelocity;
        const Vector3 bodyMomentum = inertiaTensorBody * bodyOmega;

        // 자이로 항 explicit Euler 한 스텝
        Vector3 newBodyMomentum =
            bodyMomentum - bodyOmega.Cross(bodyMomentum) * dt;

        // |L| 재정규화
        const Real newLengthSquared = newBodyMomentum.LengthSquared();

        if (newLengthSquared > Real{ 0 })
        {
            newBodyMomentum *= std::sqrt(
                bodyMomentum.LengthSquared() / newLengthSquared);
        }

        angularVelocity =
            rotation * (inverseInertiaTensorBody * newBodyMomentum);

        RotateByRotationVector(angularVelocity * dt);

        SyncAngularMomentumFromVelocity();
    }

    // Bullet Physics 방식 (btRigidBody::computeGyroscopicImpulseImplicit_Body 참고)
    //
    // body 좌표 implicit Euler:
    //   f(ω') = I_b(ω' - ω) + dt ω' × I_b ω' = 0
    //
    // ω' = ω 에서 시작해 뉴턴법 1회:
    //   f(ω) = dt ω × I_b ω
    //   J    = I_b + dt([ω]× I_b - [I_b ω]×)      (d(ω×Iω)/dω = [ω]×I - [Iω]×)
    //   ω'   = ω - J^-1 f
    //
    // implicit이라 에너지가 줄어드는 쪽으로만 틀린다. 대신 큰 ω에서도 터지지 않는다.
    void RigidBody::IntegrateAngularBulletStyle(Real dt)
    {
        // 외부 토크: ω += I_world^-1 τ dt
        angularVelocity +=
            (inverseInertiaTensorWorld * torqueAccumulator) * dt;

        const Matrix3 rotation = orientation.ToMatrix3();
        const Matrix3 rotationTranspose = rotation.Transposed();

        const Vector3 bodyOmega = rotationTranspose * angularVelocity;
        const Vector3 bodyMomentum = inertiaTensorBody * bodyOmega;

        // 잔차
        const Vector3 residual =
            bodyOmega.Cross(bodyMomentum) * dt;

        // 야코비안
        const Matrix3 jacobian = Add(
            inertiaTensorBody,
            Scale(
                Subtract(Skew(bodyOmega) * inertiaTensorBody, Skew(bodyMomentum)),
                dt));

        // 뉴턴법 1회
        const Vector3 newBodyOmega =
            bodyOmega - Solve3x3(jacobian, residual);

        angularVelocity = rotation * newBodyOmega;

        RotateByRotationVector(angularVelocity * dt);

        SyncAngularMomentumFromVelocity();
    }

    // 자이로 항 무시
    //
    //   I dω/dt = τ   (ω × Iω 항을 버림)
    //
    // 토크가 없으면 world ω가 일정하다. 에너지는 안정적이지만 L은 보존되지 않고,
    // 테니스 라켓 정리 같은 뒤집힘이 일어나지 않는다.
    void RigidBody::IntegrateAngularGyroscopicOff(Real dt)
    {
        angularVelocity +=
            (inverseInertiaTensorWorld * torqueAccumulator) * dt;

        RotateByRotationVector(angularVelocity * dt);

        SyncAngularMomentumFromVelocity();
    }


    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // 기존 함수 (테스트 호환용)
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    void RigidBody::IntegrateOrientationExplicit(Real dt)
    {
        Vector3 rotationVector =
            angularVelocity * dt;       // 축각벡터 = 각속도 * 시간. 이 수식은 각속도가 일정하다면 무조건 성립. 작은 각만 성립하는 게 아니라 큰 각도에서도 성립.
        RotateByRotationVector(rotationVector);
        // 문제점: 실제로는 한 스텝 동안 ω가 일정하지 않다.
        // (L은 일정해도 자세가 바뀌면 I_world가 바뀌므로 ω = I_world^-1 L 이 변함)
        // 그래서 회전 에너지 E = 1/2 L·ω 가 매 스텝 조금씩 증가하고,
        // 결국 강체가 관성모멘트가 가장 작은 축으로 끌려간다. (tests/IntegratorTest.cpp 참고)
    }

    /*
        자유 강체의 회전 에너지(해밀토니안)를 body-space 각운동량 L_b로 쓰면

            H = Lx^2/(2Ix) + Ly^2/(2Iy) + Lz^2/(2Iz)

        H를 세 항으로 나누면, 각 항만 있을 때의 운동은 정확히 풀린다:
        "body의 i번째 주축을 중심으로 각속도 Li/Ii 로 회전" (이때 Li는 변하지 않음)

        그래서 세 회전을 대칭으로 이어 붙인다. (Strang splitting, 2차 정확도)

            x축 dt/2 -> y축 dt/2 -> z축 dt -> y축 dt/2 -> x축 dt/2

        각 단계가 정확한 회전이므로
          - world-space L은 정확히 보존된다 (회전만 할 뿐 L을 건드리지 않음)
          - 방법이 symplectic이라 에너지 오차가 누적되지 않고 작은 범위에서 진동한다
    */
    void RigidBody::RotateAboutPrincipalAxis(int axis, Real h)
    {
        // world-space L을 body-space로 변환: L_b = R^T L
        const Matrix3 rotation =
            orientation.ToMatrix3();

        const Vector3 bodyMomentum =
            rotation.Transposed() * angularMomentum;

        Real Li = Real{ 0 };
        Real Ii = Real{ 0 };

        switch (axis)
        {
        case 0: Li = bodyMomentum.x; Ii = inertiaTensorBody.m00; break;
        case 1: Li = bodyMomentum.y; Ii = inertiaTensorBody.m11; break;
        default: Li = bodyMomentum.z; Ii = inertiaTensorBody.m22; break;
        }

        assert(Ii > Real{ 0 });

        // 이 축에 대한 회전각 = (Li / Ii) * h
        const Real halfAngle =
            Real{ 0.5 } * (Li / Ii) * h;

        const Real s = std::sin(halfAngle);

        const Quaternion deltaRotation(
            std::cos(halfAngle),
            axis == 0 ? s : Real{ 0 },
            axis == 1 ? s : Real{ 0 },
            axis == 2 ? s : Real{ 0 }
        );

        // body-space 축 기준 회전이므로 오른쪽에서 곱한다.
        // (RotateByRotationVector는 world-space 회전이라 왼쪽에서 곱함)
        orientation =
            orientation * deltaRotation;

        UpdateOrientationDependentData();
    }

    void RigidBody::IntegrateOrientation(Real dt)
    {
        // 주축 좌표계 가정: body 관성텐서의 비대각 성분이 0이어야 함
        assert(inertiaTensorBody.m01 == Real{ 0 } && inertiaTensorBody.m02 == Real{ 0 } &&
               inertiaTensorBody.m10 == Real{ 0 } && inertiaTensorBody.m12 == Real{ 0 } &&
               inertiaTensorBody.m20 == Real{ 0 } && inertiaTensorBody.m21 == Real{ 0 });

        if (angularMomentum.IsNearlyZero())
        {
            return;
        }

        const Real half = Real{ 0.5 } * dt;

        RotateAboutPrincipalAxis(0, half);
        RotateAboutPrincipalAxis(1, half);
        RotateAboutPrincipalAxis(2, dt);
        RotateAboutPrincipalAxis(1, half);
        RotateAboutPrincipalAxis(0, half);
    }


    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // 기타
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    void RigidBody::UpdateOrientationDependentData()
    {
        orientation.NormalizeUnsafe();

        Matrix3 rotation =
            orientation.ToMatrix3();

        Matrix3 rotationTranspose =
            rotation.Transposed();


        // I_world = R I_body R^T
        inertiaTensorWorld =
            rotation
            * inertiaTensorBody
            * rotationTranspose;


        // I_world^-1 = R I_body^-1 R^T
        inverseInertiaTensorWorld =
            rotation
            * inverseInertiaTensorBody
            * rotationTranspose;
    }

    void RigidBody::SyncAngularVelocityFromMomentum()
    {
        angularVelocity =
            inverseInertiaTensorWorld * angularMomentum;
    }

    void RigidBody::SyncAngularMomentumFromVelocity()
    {
        angularMomentum =
            inertiaTensorWorld * angularVelocity;
    }

    void RigidBody::SetAngularVelocity(
        const Vector3& newAngularVelocity
    )
    {
        angularVelocity =
            newAngularVelocity;

        SyncAngularMomentumFromVelocity();
    }

    void RigidBody::ClearAccumulators()
    {
        linearForceAccumulator =
            Vector3(0, 0, 0);

        torqueAccumulator =
            Vector3(0, 0, 0);
    }

    void RigidBody::SetMass(Real mass)
    {
        assert(mass > Real{ 0 });

        inverseMass =
            Real{ 1 } / mass;
    }


    Real RigidBody::GetMass() const
    {
        assert(inverseMass > Real{ 0 });

        return Real{ 1 } / inverseMass;
    }


    bool RigidBody::HasFiniteMass() const
    {
        return inverseMass > Real{ 0 };
    }

    void RigidBody::InitMassProperties(
        Real mass,
        const Matrix3& inertiaTensor,
        const Matrix3& inverseInertiaTensor
    )
    {
        assert(mass > Real{ 0 });

        // 새 강체의 초기화 용도이므로
        // 아직 회전 운동을 시작하기 전이라고 가정.
        assert(angularVelocity.IsNearlyZero());
        assert(angularMomentum.IsNearlyZero());

        inverseMass =
            Real{ 1 } / mass;

        inertiaTensorBody =
            inertiaTensor;

        inverseInertiaTensorBody =
            inverseInertiaTensor;

        // 현재 orientation에 맞는
        // world-space inertia tensor 생성
        UpdateOrientationDependentData();
    }

    void RigidBody::RotateByRotationVector(
        const Vector3& rotationVector  // rotationVector는 월드 좌표계에서의 축-각 회전벡터. 길이가 회전각(rad), 방향이 회전축.
    )
    {
        if (rotationVector.IsNearlyZero())
        {
            return;
        }

        const Real angle =
            rotationVector.Length();

        const Real halfAngle =
            Real{ 0.5 } * angle;

        const Real vectorScale =
            std::sin(halfAngle) / angle;

        const Quaternion deltaRotation(
            std::cos(halfAngle),
            rotationVector.x * vectorScale,
            rotationVector.y * vectorScale,
            rotationVector.z * vectorScale
        );

        // 월드 좌표계의 회전벡터이므로 왼쪽에서 곱한다.
        orientation =
            deltaRotation * orientation;

        // orientation이 바뀌었으므로 orientation을 정규화하고, world-space inertia tensor를 갱신한다.
        UpdateOrientationDependentData();
    }
}
