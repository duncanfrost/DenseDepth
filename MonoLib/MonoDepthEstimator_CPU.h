#pragma once

#include "Shared/DenseMonoMath.h"

#include "Shared/MonoDepthEstimator_Shared.h"
#include "MonoDepthEstimator.h"

namespace MonoLib {

    class MonoDepthEstimator_CPU : public MonoDepthEstimator
    {
    private:

        float MeasureErrorGT(float *mu, float *sigma,
                             float *GTDepths, unsigned int width, unsigned int height);


        void SetBuffer(float *data, Vector2i imgSize, float value);


        void UpdatePhotoError(ORUtils::SE3Pose refToTracker,
                              ORUtils::TimeStampedImage<Vector4u> *frame);

        void RunTVOptimisation(unsigned int iterations) {}

        void InitOptim(){}
        void SmoothDTAM(){}

    public:
        MonoDepthEstimator_CPU(Vector2i imgSize, Vector4f intrinsics_raw);
        ~MonoDepthEstimator_CPU();

        void SetRefImage(ORUChar4TSImage *frame);
        void OptimToDepth(bool useRawDepth) {};

        void UpdateForPointCloud() {}

        float EvaluateGT() { return 1e10f; }
        void DisplayPhotoVolume(int x, int y);
        void MeasureError() {};
    };
}
