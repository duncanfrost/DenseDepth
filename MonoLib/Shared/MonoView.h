// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#pragma once
// #include "Pyramid.h"
// #include "ImagePyramidLevel.h"
#include <ORUtils/SE3Pose.h>
#include <ORUtils/TimeStampedImage.h>
#include "MonoPyramidLevel.h"

namespace MonoLib
{
    class MonoView
    {
    public:
        MonoPyramidLevel *dataImage;
        ORUtils::TimeStampedImage<Vector4u> *colorImageData;

        float meanGrey;

        void MakePointCloud()
        {
            dataImage->UpdatePointCloud();
        }

        MonoView(Vector2i imgSize, MonoLib::Intrinsics intrinsics)
        {
            dataImage = new  MonoPyramidLevel(imgSize,0,MEMORYDEVICE_CPU,intrinsics);

            bool allocateCPU = true;
            bool allocateCUDA = true;

            this->colorImageData =
                new ORUtils::TimeStampedImage<Vector4u>(imgSize,
                                                        allocateCPU,allocateCUDA);
        }


        void DepthFromOptim(float *a, float minDepth, float maxDepth)
        {
            dataImage->DepthFromOptim(a, minDepth, maxDepth);
        } 

        void Init()
        {
            dataImage->Init();
        }

        void DebugDisplay()
        {
            dataImage->DebugDisplay();
        }

        void UpdateHostFromDevice()
        {
            dataImage->UpdateHostFromDevice();
            colorImageData->UpdateHostFromDevice();
        }

        void UpdateDeviceFromHost()
        {
            dataImage->UpdateDeviceFromHost();
            colorImageData->UpdateDeviceFromHost();
        }

        ~MonoView(void)
        {
        }

        // Suppress the default copy constructor and assignment operator
        MonoView(const MonoView&);
        MonoView& operator=(const MonoView&);
    };
}
