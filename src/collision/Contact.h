#pragma once

#include "../math/Precision.h"
#include "../math/Vector3.h"
#include "../physics/RigidBody.h"


namespace PE {

    struct Contact
    {
        RigidBody* bodyA;

        // 정적 환경(Plane 등)이면 nullptr
        RigidBody* bodyB;

        Real restitution = Real{ 0 };
        Real friction = Real{ 0 };

        // world-space contact point
        Vector3 point;

        // world-space contact normal
        // bodyB -> bodyA 방향
        Vector3 normal;

        // 침투 깊이. 양수=침투 중, 0=접촉 중, 음수=떨어져있음
        Real penetration;

        // Contact Space -> World Space 변환 행렬
        // 열벡터: [normal, tangent1, tangent2] 
        // normal, tangent1, tangent2는 월드 좌표계 벡터
        Matrix3 contactToWorld;

        // 접촉점 - 각 강체의 질량중심 (World Space)
        Vector3 relativeContactPosition[2];

        // 접촉 좌표계 상대속도
        // x = normal, y/z = tangent
        Vector3 contactVelocity;

        // 접촉 해결에 필요한 법선 방향 속도 변화량
        Real desiredDeltaVelocity = Real{ 0 };



        void CalculateContactBasis();
        void CalculateContactVelocity(Real dt);
        void CalculateDesiredDeltaVelocity(Real dt);

        // 접촉 해결에 필요한 내부 데이터 준비
        void CalculateInternals();

        //두 collider의 반발계수로 contact의 반발계수 설정
        //현재 사용 수식: 두 collider 반발계수의 평균값
        void setRestitution(Real restitutionA, Real restitutionB);
    };

}