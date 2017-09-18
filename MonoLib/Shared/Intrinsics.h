#pragma once

#include <ORUtils/MathTypes.h>

namespace MonoLib
{
    class Intrinsics
    {

    public:

        Intrinsics(void)
        {
            fx = 580;
            fy = 580;
            cx = 320;
            cy = 240;
        }
        void SetFrom(Vector4f in)
        {
            fx = in[0];
            fy = in[1];
            cx = in[2];
            cy = in[3];
            ComputeInverse();
        }

        float fx, fy, cx, cy;
        float fxInv, fyInv, cxInv, cyInv;

    private:

        void ComputeInverse()
        {
            fxInv = 1.0f / fx;
            fyInv = 1.0f / fy;

            cxInv = -1.0f*fxInv*cx;
            cyInv = -1.0f*fyInv*cy;
        }

    };
}
