#pragma once

#include "Shared/MonoDepthEstimator_Shared.h"
#include "MonoDepthEstimator.h"

namespace MonoLib {
    class MonoDepthEstimator_CUDA : public MonoDepthEstimator
    {
    public:
        MonoDepthEstimator_CUDA(Vector2i imgSize, Vector4f intrinsics_raw);
        ~MonoDepthEstimator_CUDA() {}


        void ComputeMaxPixel();

        void SetRefImage(ORUChar4TSImage *frame);
        float EvaluateGT();
        void UpdateForPointCloud() {currDepthFrame->UpdateHostFromDevice();}

        void ReinitOptim();

        void UpdatePhotoError(ORUtils::SE3Pose refToTracker,
                              ORUtils::TimeStampedImage<Vector4u> *frame);

        void SmoothL1();

        void OptimToDepth(bool useRawDepth);

        void RunTVOptimisation(unsigned int iterations);
        void InitOptim();
        void SmoothDTAM();
        void DisplayPhotoVolume(int x, int y);

        void MeasureError();
    };
}
