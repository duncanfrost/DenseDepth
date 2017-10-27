#pragma once
#include <ORUtils/Image.h>
#include <ORUtils/SE3Pose.h>
#include "Intrinsics.h"

#include <iostream>

namespace MonoLib {


    class OptimPyramidLevel
    {

    public:
        MonoLib::Intrinsics intrinsics;

        //Data Images


        //Stuff for TV smoothing
        ORUtils::Image<float> *p;
        ORUtils::Image<float> *qx, *qy;
        ORUtils::Image<float> *g;

        //Photometric errors
        ORUtils::MemoryBlock<float> *photoErrors;

        //Total number of updates for each voxel
        ORUtils::MemoryBlock<int> *nUpdates;

        ORUtils::Image<float> *a;
        ORUtils::Image<float> *d;

        //This is per-pixel TOTAL error (including TV term)
        ORUtils::Image<float> *error;

        //The index of the minimum error in the cost volume
        ORUtils::Image<int> *minIndices;
        // ORUtils::Image<float> *minError;


        //This is per-pixel TOTAL error (including TV term)
        ORUtils::Image<float> *certainty;


        //Intermediate variables for debugging purposes
        //=============================================

        //Gradient of d in x and y direction
        ORUtils::Image<float> *dx, *dy;

        //Divergence of q
        ORUtils::Image<float> *divQ;


        int depthSamples;
        float minIDepth;
        float maxIDepth;

        void Init()
        {
            size_t totalImgSize = a->dataSize;
            float init = 0.5f;
            for (size_t index = 0; index < totalImgSize; index++)
            {
                a->GetData(MEMORYDEVICE_CPU)[index] = init;
                d->GetData(MEMORYDEVICE_CPU)[index] = init;
            }
        }


        void SetDepthLimits(float inputMinDepth, float inputMaxDepth)
        {
            std::cout << "Setting depth limits: " << inputMinDepth << "  -  " << inputMaxDepth << std::endl;
            this->minIDepth = 1 / inputMaxDepth;
            this->maxIDepth = 1 / inputMinDepth;
        }

        OptimPyramidLevel(Vector2i imgSize, int levelId, MemoryDeviceType memoryType,
                          Intrinsics intrinsics, bool skipAllocation = false)
        {

            bool allocateCPU = true;
            bool allocateCUDA = true;

            //Going to do 0 to 4 with 0.1 spacing
            depthSamples = 64;

            SetDepthLimits(0.5,3.5);

            this->d = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->a = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);

            this->qx = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->qy = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->g = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->photoErrors =
                new ORUtils::MemoryBlock<float>(imgSize.x * imgSize.y * depthSamples,
                                                allocateCPU,allocateCUDA);
            this->nUpdates =
                new ORUtils::MemoryBlock<int>(imgSize.x * imgSize.y * depthSamples,
                                                allocateCPU,allocateCUDA);

            this->error = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->certainty = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->minIndices = new ORUtils::Image<int>(imgSize, allocateCPU, allocateCUDA);


            this->dx = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->dy = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->divQ = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->p = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            // this->minError = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);


        }

        ~OptimPyramidLevel()
        {
            //TODO -- delete all the stuff
        }

        void UpdateHostFromDevice()
        {
            p->UpdateHostFromDevice();
            qx->UpdateHostFromDevice();
            qy->UpdateHostFromDevice();
            g->UpdateHostFromDevice();
            photoErrors->UpdateHostFromDevice();
            nUpdates->UpdateHostFromDevice();
            a->UpdateHostFromDevice();
            d->UpdateHostFromDevice();
            error->UpdateHostFromDevice();
            certainty->UpdateHostFromDevice();
            minIndices->UpdateHostFromDevice();
            dx->UpdateHostFromDevice();
            dy->UpdateHostFromDevice();
            divQ->UpdateHostFromDevice();
            // minError->UpdateHostFromDevice();
        }

        void UpdateDeviceFromHost()
        {
            p->UpdateDeviceFromHost();
            qx->UpdateDeviceFromHost();
            qy->UpdateDeviceFromHost();
            g->UpdateDeviceFromHost();
            photoErrors->UpdateDeviceFromHost();
            nUpdates->UpdateDeviceFromHost();
            a->UpdateDeviceFromHost();
            d->UpdateDeviceFromHost();
            error->UpdateDeviceFromHost();
            certainty->UpdateDeviceFromHost();
            minIndices->UpdateDeviceFromHost();
            dx->UpdateDeviceFromHost();
            dy->UpdateDeviceFromHost();
            divQ->UpdateDeviceFromHost();
            // minError->UpdateDeviceFromHost();
        }
    };
}
