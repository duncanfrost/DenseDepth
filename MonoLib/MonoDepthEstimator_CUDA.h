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

        void SetupCUDA(Vector2i imgSize)
        {
            int denseBlockSize = 16;
            dimBlock = dim3(denseBlockSize, denseBlockSize);
            int gridX = (int)ceil((float)imgSize.x / (float)denseBlockSize);
            int gridY = (int)ceil((float)imgSize.y / (float)denseBlockSize);
            dimGrid = dim3(gridX, gridY);
        }

        void SetRefImage(ORUChar4TSImage *frame);
        float EvaluateGT();
        void UpdateForPointCloud() {currDepthFrame->UpdateHostFromDevice();}

        void ReinitOptim();

        void UpdatePhotoError(ORUtils::SE3Pose refToTracker,
                              ORUtils::TimeStampedImage<Vector4u> *frame);

        void UpdatePhotoError(ORUtils::SE3Pose refToTracker,
                              ORUtils::TimeStampedImage<unsigned char> *frame);






        void OptimToDepth(bool useRawDepth);

        void RunTVOptimisation(unsigned int iterations);
        void InitOptim();
        void SmoothDTAM();
        void DisplayPhotoVolume(int x, int y);

        void SetGT();
        void MeasureError();

    private:
        dim3 dimGrid;
        dim3 dimBlock;
    };
}
