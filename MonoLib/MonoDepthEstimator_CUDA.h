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
        void SetRefAndFeatureImage(ORUChar4TSImage *frame,
                                   ORUtils::MemoryBlock<float> *featureImage);
        float EvaluateGT();
        void UpdateForPointCloud() {currDepthFrame->UpdateHostFromDevice();}

        void ReinitOptim();

        void UpdatePhotoError(ORUtils::SE3Pose refToTracker,
                              ORUtils::TimeStampedImage<Vector4u> *frame);

        void UpdatePhotoErrorWithFeatures(ORUtils::SE3Pose refToTracker,
                                          ORUtils::TimeStampedImage<Vector4u> *frame,
                                          ORUtils::MemoryBlock<float> *featureImage);

        void SmoothL1();
        void SmoothHuber();
        void LoadDepth(ORFloatImage depthImage);
        float MeasureError();

        void OptimToDepth(bool useRawDepth);

        void RunTVOptimisationActive(float theta);
        void RunTVOptimisation();
        void RunTVL1Optimisation();
        void InitOptim();
        void SmoothDTAM();
        void DisplayPhotoVolume(int x, int y);
    };
}
