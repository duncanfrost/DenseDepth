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
        ORUtils::MemoryBlock<float> *featureImageData;

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


            //TODO: these need to be passed in as aruments
            int featureChannels = 64;
            int featureHeight = 480;
            int featureWidth = 640;
            this->featureImageData = new
                ORUtils::MemoryBlock<float>(featureChannels*featureHeight*featureWidth,
                                            true, true, true);
        }


        void DepthFromOptim(float *a, float minDepth, float maxDepth)
        {
            dataImage->DepthFromOptim(a, minDepth, maxDepth);
        } 

        void Init()
        {
            dataImage->Init();
        }

        void UpdateHostFromDevice()
        {
            dataImage->UpdateHostFromDevice();
            colorImageData->UpdateHostFromDevice();
            featureImageData->UpdateHostFromDevice();
        }

        void UpdateDeviceFromHost()
        {
            dataImage->UpdateDeviceFromHost();
            colorImageData->UpdateDeviceFromHost();
            featureImageData->UpdateDeviceFromHost();
        }

        ~MonoView(void)
        {
        }

        // Suppress the default copy constructor and assignment operator
        MonoView(const MonoView&);
        MonoView& operator=(const MonoView&);
    };
}
