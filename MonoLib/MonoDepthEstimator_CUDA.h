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

        void SetRefImage(ORUtils::MemoryBlock<float> *frame);
        float EvaluateGT();
        void UpdateForPointCloud() {currDepthFrame->UpdateHostFromDevice();}

        void ReinitOptim();

        void UpdatePhotoError(ORUtils::SE3Pose refToTracker,
                              ORUtils::MemoryBlock<float> *frame);

        void SmoothL1();
        void SmoothHuber();
        void LoadDepth(ORFloatImage depthImage);
        float MeasureError();

        void OptimToDepth(bool useRawDepth);

        void RunTVOptimisation(unsigned int iterations);
        void RunTVL1Optimisation(unsigned int iterations);
        void RunTVL0Optimisation(unsigned int iterations);
        void InitOptim();
        void SmoothDTAM();
        void DisplayPhotoVolume(int x, int y);
    };
}
