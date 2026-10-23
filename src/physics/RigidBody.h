#pragma once

#include "../math/Precision.h"
#include "../math/Vector3.h"
#include "../math/Matrix3.h"
#include "../math/Quaternion.h"

namespace PE {

    // -------------------------
    // 병진 운동 적분 방식
    // -------------------------
    enum class LinearIntegrationMethod
    {
        ExplicitEuler,      // x(n+1) = x(n) + v(n) dt,   v(n+1) = v(n) + a dt
        SemiImplicitEuler   // v(n+1) = v(n) + a dt,      x(n+1) = x(n) + v(n+1) dt
    };

    // -------------------------
    // 회전 운동 적분 방식
    // -------------------------
    //
    // [각운동량 L 기반] 상태 변수 = world-space L.  ω는 매 스텝 ω = I_world^-1 L 로 계산
    //   공통 순서: 1) L += τ dt   2) 자세 적분(방식별)   3) ω = I_world^-1 L
    //   토크가 없으면 L은 정확히 보존된다.
    //
    // [각속도 ω 기반] 상태 변수 = ω.  L은 매 스텝 L = I_world ω 로 맞춰줌
    //   공통 순서: 1) ω 갱신(방식별)   2) 새 ω로 자세 적분   3) L = I_world ω
    //
    enum class AngularIntegrationMethod
    {
        // ----- 각운동량 L 기반 -----
        PreviousOmega,      // 스텝 시작 자세의 ω로 한 스텝 회전 (explicit). 에너지 증가
        NextOmega,          // 스텝 끝 자세의 ω로 한 스텝 회전 (implicit, 고정점 반복). 에너지 감소
        MidpointOmega,      // 반 스텝 예측한 중간 자세의 ω로 한 스텝 회전 (RK2). 느리게 누적
        StrangSplitting,    // 주축별 정확한 회전을 x-y-z-y-x로 대칭 합성. 에너지 오차 누적 없음

        // ----- 각속도 ω 기반 -----
        EulerEquation,      // I dω/dt = τ - ω × Iω 를 semi-implicit Euler로. 에너지와 |L| 모두 증가
        JoltStyle,          // 위 explicit 스텝 후 |L|을 원래 크기로 재정규화
        BulletStyle,        // implicit Euler를 뉴턴법 1회로 근사. 에너지 감소(안정)
        GyroscopicOff       // 자이로 항(ω × Iω) 무시. 토크 없으면 ω 일정, 뒤집힘 없음
    };


    class RigidBody
    {
    public:
        RigidBody();

        Vector3 position;
        Quaternion orientation;
        Vector3 linearVelocity;
        Vector3 angularVelocity;  //world space
        Vector3 angularMomentum;  //world space
        Real inverseMass;

        Matrix3 inertiaTensorBody;
        Matrix3 inverseInertiaTensorBody;
        Matrix3 inertiaTensorWorld;
        Matrix3 inverseInertiaTensorWorld;

        Vector3 linearForceAccumulator;
        Vector3 torqueAccumulator;

        Vector3 lastFrameAcceleration;


        // -------------------------
        // 힘 / 토크
        // -------------------------

        void AddLinearForce(const Vector3& force);

        void AddTorque(const Vector3& torque);

        void AddForceAtPoint(
            const Vector3& force,
            const Vector3& worldPoint
        );


        // 기본값: 병진 semi-implicit Euler, 회전 분할 회전 (기존 동작과 동일)
        void Integrate(
            Real dt,
            LinearIntegrationMethod linearMethod =
                LinearIntegrationMethod::SemiImplicitEuler,
            AngularIntegrationMethod angularMethod =
                AngularIntegrationMethod::StrangSplitting
        );


        // -------------------------
        // 상태 동기화. L=Iω 공식에 따라 angularVelocity와 angularMomentum을 서로 동기화
        // -------------------------

        // L을 기준으로 ω를 계산
        // ω = I_world^-1 L
        void SyncAngularVelocityFromMomentum();

        // ω를 기준으로 L을 계산
        // L = I_world ω
        void SyncAngularMomentumFromVelocity();


        // angularVelocity를 외부에서 직접 설정할 때 사용
        void SetAngularVelocity(const Vector3& newAngularVelocity);


        // -------------------------
        // 기타
        // -------------------------

        void ClearAccumulators();

        // orientation으로부터 world-space inertia tensor 갱신
        void UpdateOrientationDependentData();

        void SetMass(Real mass);
        Real GetMass() const;

        bool HasFiniteMass() const;

        void InitMassProperties(
            Real mass,
            const Matrix3& inertiaTensor,
            const Matrix3& inverseInertiaTensor
        );
        void RotateByRotationVector(
            const Vector3& rotationVector
        );

        // [기존 함수, 테스트 호환용]
        // 현재 angularVelocity로 한 스텝 회전 (1차 explicit)
        void IntegrateOrientationExplicit(Real dt);

        // [기존 함수, 테스트 호환용]
        // 현재 angularMomentum 기준 분할 회전 (StrangSplitting과 같은 계산)
        void IntegrateOrientation(Real dt);

    private:
        // ----- 병진 -----
        void IntegrateLinear(Real dt, LinearIntegrationMethod method);

        // ----- 회전: 각운동량 L 기반 -----
        void IntegrateAngularPreviousOmega(Real dt);
        void IntegrateAngularNextOmega(Real dt);
        void IntegrateAngularMidpointOmega(Real dt);
        void IntegrateAngularStrangSplitting(Real dt);

        // ----- 회전: 각속도 ω 기반 -----
        void IntegrateAngularEulerEquation(Real dt);
        void IntegrateAngularJoltStyle(Real dt);
        void IntegrateAngularBulletStyle(Real dt);
        void IntegrateAngularGyroscopicOff(Real dt);

        // 현재 orientation에서의 world-space ω = I_world^-1 L (멤버는 바꾸지 않음)
        Vector3 ComputeAngularVelocityFromMomentum() const;

        // body-space 주축 axis(0=x,1=y,2=z)를 중심으로
        // 시간 h 동안 자유 회전시킨다 (분할 회전용)
        void RotateAboutPrincipalAxis(int axis, Real h);
    };
}
