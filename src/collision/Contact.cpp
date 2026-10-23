
#include "Contact.h"

#include <cassert>
#include <cmath>


namespace PE {

    void Contact::CalculateContactBasis() //normal 벡터를 기준으로 접촉 좌표계의 기저벡터를 계산한다.(정규직교 좌표계)
    {
        // contact.normal은 단위벡터라고 가정한다.
        assert(
            std::abs(normal.LengthSquared() - Real{ 1 })
            < Real{ 0.001 }
        );

        //normal에 수직인 기저벡터를 구한다.
        //normal과 평행하지않은 임의의 어떤 벡터든 normal과 외적하면 normal에 수직인 벡터가 나온다.
        //tangent1은 x축 기저 (1,0,0) 아니면 y축 기저 (0,1,0)중 normal과 더 수직인(normal과 외적한 벡터의 크기가 더 큰) 기저벡터를 선택한다.
        Vector3 tangent1;

        if (std::abs(normal.x) > std::abs(normal.y))
        {   //y축 기저 (0,1,0)선택
            //inverseLength는 정규화를 위한 값
            const Real inverseLength =
                Real{ 1 } / std::sqrt(
                    normal.x * normal.x +
                    normal.z * normal.z
                );

            //외적을 계산하게 하지 않고 외적 결과를 바로 쓴다.
            tangent1 = Vector3(
                normal.z * inverseLength,
                Real{ 0 },
                -normal.x * inverseLength
            );
        }
        else
        {   //x축 기저 (1,0,0)선택
            const Real inverseLength =
                Real{ 1 } / std::sqrt(
                    normal.y * normal.y +
                    normal.z * normal.z
                );

            tangent1 = Vector3(
                Real{ 0 },
                -normal.z * inverseLength,
                normal.y * inverseLength
            );
        }

        // normal과 tangent1 모두에 수직인 두 번째 접선
        const Vector3 tangent2 =
            normal.Cross(tangent1);

        // 세 기저벡터를 열벡터로 저장한다.
        contactToWorld = Matrix3(
            normal.x, tangent1.x, tangent2.x,
            normal.y, tangent1.y, tangent2.y,
            normal.z, tangent1.z, tangent2.z
        );
    }

    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


    void Contact::CalculateContactVelocity(Real dt)
    {
        assert(bodyA != nullptr);
        assert(dt > Real{ 0 });

        // A의 접촉점 속도 (World Space)
        const Vector3 velocityA =
            bodyA->linearVelocity
            + bodyA->angularVelocity.Cross(
                relativeContactPosition[0]
            );

        // B가 정적 환경이면 접촉점 속도를 0으로 처리
        const Vector3 velocityB =
            bodyB != nullptr
            ? bodyB->linearVelocity
            + bodyB->angularVelocity.Cross(
                relativeContactPosition[1]
            )
            : Vector3();

        const Vector3 relativeVelocity =
            velocityA - velocityB;

        // 월드 좌표계 -> 접촉 좌표계
        const Matrix3 worldToContact =
            contactToWorld.Transposed();

        contactVelocity =
            worldToContact * relativeVelocity;

        CalculateDesiredDeltaVelocity(dt);
    }

    //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    void Contact::CalculateDesiredDeltaVelocity(Real dt)
    {
        assert(bodyA != nullptr);
        assert(dt > Real{ 0 });

        constexpr Real velocityLimit = Real{ 0.25 };

        Vector3 relativeAcceleration =
            bodyA->lastFrameAcceleration;

        if (bodyB != nullptr)
        {
            relativeAcceleration -=
                bodyB->lastFrameAcceleration;
        }

        const Real velocityFromAcceleration =
            relativeAcceleration.Dot(normal) * dt;

        Real effectiveRestitution = restitution;

        // 저속 충돌에서는 반발을 억제한다.
        if (std::abs(contactVelocity.x) < velocityLimit)
        {
            effectiveRestitution = Real{ 0 };
        }

        desiredDeltaVelocity =
            -contactVelocity.x
            - effectiveRestitution
            * (contactVelocity.x - velocityFromAcceleration);
    }


    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    void Contact::CalculateInternals()
    {
        assert(bodyA != nullptr);

        // 접촉 좌표계 생성
        CalculateContactBasis();

        // 각 강체의 질량중심에서 접촉점까지의 벡터
        relativeContactPosition[0] =
            point - bodyA->position;

        relativeContactPosition[1] =
            bodyB != nullptr
            ? point - bodyB->position
            : Vector3();
    }

    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    void Contact::setRestitution(Real restitutionA, Real restitutionB)
    {
        restitution = Real{ 0.5 } * (restitutionA + restitutionB);
    }

}