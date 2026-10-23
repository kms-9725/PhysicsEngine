#include "PlaneCollider.h"

#include <cassert>


namespace PE {

    PlaneCollider::PlaneCollider(
        const Vector3& normal,
        Real offset,
        Real restitution
    )
    {
        assert(!normal.IsNearlyZero());


        Real normalLength =
            normal.Length();


        // 입력 normal이 단위벡터가 아니더라도
        // plane 자체는 동일하게 유지하면서 정규화한다.
        //
        // n · x = d
        //
        // 양변을 |n|으로 나누면
        //
        // (n / |n|) · x = d / |n|

        this->normal =
            normal / normalLength;

        this->offset =
            offset / normalLength;

        this->restitution =
            restitution;
    }
}