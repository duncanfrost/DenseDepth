// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#pragma once

#include <ORUtils/SE3Pose.h>
#include <ORUtils/ImageTypes.h>
#include "Shared/MonoView.h"
#include "Shared/OptimPyramidLevel.h"
#include "Shared/DenseMonoMath.h"
#include "Shared/Intrinsics.h"
#include <ORUtils/FileUtils.h>


#include <sstream>

#include <vector>
#include <mutex>

namespace MonoLib {
    class MonoDepthEstimator
    {

        struct TVParameters
        {
            TVParameters()
            {
                //Weighting between regulariser and photometric cost
                lambda=0.2;
                // lambda=0.05;
                //this is normally 0.2 for rgb channels

                // Step sizes for gradient ascent/descent 
                sigma_d = 0.01;
                sigma_q = 10;

                // Huber threshold
                epsilon=1e-4;


                //Edge weighting parameters

                // edgeAlpha = 40.0f;
                // edgeBeta = 1.5f;
                edgeAlpha = 0.8f;
                edgeBeta = 0.8f;
            }

            float lambda;
            float sigma_q;
            float sigma_d;
            float epsilon;
            float beta;
            float edgeAlpha, edgeBeta;
        };

    public:

        MonoDepthEstimator(Vector2i imgSize, Vector4f intrinsics_raw)
        {
            isInitialised = false;

            Intrinsics intrinsics;
            intrinsics.SetFrom(intrinsics_raw);

            currDepthFrame = new MonoView(imgSize,intrinsics);
            currDepthFrame->Init();
            currDepthFrame->UpdateDeviceFromHost();

            optimPyramid = new OptimPyramidLevel(imgSize,0,MEMORYDEVICE_CPU,intrinsics);
            optimPyramid->Init();
            optimPyramid->UpdateDeviceFromHost();

            std::string sequence = "rgbd_dataset_freiburg3_long_office_household";
            //		std::string sequence = "rgbd_dataset_freiburg2_xyz";
            std::stringstream ss;
            ss <<  "/home/duncan/Data/TUM/"  << sequence << "/";
            std::string path = ss.str();
            //		depthReader = new DepthReader(path, "depth.txt");


        }

        virtual ~MonoDepthEstimator() {}

        virtual void ReinitOptim() {}
        virtual void SetRefImage(ORUChar4TSImage *frame)
        {
            currDepthFrame->UpdateDeviceFromHost();

            isInitialised = true;

            currDepthFrame->dataImage->nUpdate = 0;
            ReinitOptim();

            updateNumber = 0;
        }

        virtual void SetRefAndFeatureImage(ORUChar4TSImage *frame,
                                           ORUtils::MemoryBlock<float> *featureImage)
        {
            currDepthFrame->UpdateDeviceFromHost();

            isInitialised = true;

            currDepthFrame->dataImage->nUpdate = 0;
            ReinitOptim();

            updateNumber = 0;
        }



        void SetLimitsFromGroundTruth(float *depth_data, Vector2i imgSize)
        {
            float maxDepth = 0;
            float minDepth = 100;
            for (int y = 0; y < imgSize.y; y++)
                for (int x = 0; x < imgSize.x; x++)
                {
                    float depth = depth_data[x + imgSize.x*y];

                    if (depth <= 0)
                        continue;

                    if (depth > maxDepth)
                        maxDepth = depth;

                    if (depth < minDepth)
                        minDepth = depth;
                }

            std::cout << "Max depth " << maxDepth << std::endl;
            std::cout << "Min depth " << minDepth << std::endl;
            minDepth = std::max(minDepth-0.2f,0.2f);
            maxDepth = std::min(maxDepth+0.2f, 5.0f);

            SetLimitsManual(minDepth,maxDepth);
        }

        void SetLimitsManual(float minDepth, float maxDepth)
        {
            optimPyramid->SetDepthLimits(std::max(minDepth,0.2f),maxDepth);
        }

        void WritePhotoErrors(std::string filename)
        {
            OptimPyramidLevel *optimLevel = optimPyramid;
            optimLevel->UpdateHostFromDevice();

            float *photoErrors = optimLevel->photoErrors->GetData(MEMORYDEVICE_CPU);
            Vector2i imgSize = optimLevel-> g->noDims;
            int depthSamples = optimLevel->depthSamples;

            FILE * pFile;
            pFile = fopen (filename.c_str(), "wb");

            if (pFile == NULL)
            {
                std::cout << "Couldn't open file" << std::endl;
                exit(1);
            }
            fwrite (photoErrors , sizeof(float),imgSize.x*imgSize.y*depthSamples, pFile);
            fclose (pFile);
            std::cout << "Written" << std::endl;
        }

        void ReadPhotoErrors(std::string filename)
        {

            OptimPyramidLevel *optimLevel = optimPyramid;
            optimLevel->UpdateHostFromDevice();

            float *photoErrors = optimLevel->photoErrors->GetData(MEMORYDEVICE_CPU);
            Vector2i imgSize = optimLevel-> g->noDims;
            int depthSamples = optimLevel->depthSamples;


            FILE * pFile;
            pFile = fopen (filename.c_str(), "rb");
            if (pFile == NULL)
            {
                std::cout << "Couldn't open file" << std::endl;
                exit(1);
            }
            fread (photoErrors , sizeof(float),imgSize.x*imgSize.y*depthSamples, pFile);
            fclose (pFile);
            optimLevel->UpdateDeviceFromHost();
        }

        void WriteDepth(std::string filename)
        {

            // optimPyramid->a->UpdateHostFromDevice();
            // currDepthFrame->DepthFromOptim(optimPyramid->a->GetData(MEMORYDEVICE_CPU),
            //                optimPyramid->minDepth,optimPyramid->maxDepth);
            MonoPyramidLevel *level = currDepthFrame->dataImage;

            float *depth = level->depth->GetData(MEMORYDEVICE_CPU);
            Vector2i imgSize = level->depth->noDims;

            FILE * pFile;
            pFile = fopen (filename.c_str(), "wb");

            if (pFile == NULL)
            {
                std::cout << "Couldn't open file" << std::endl;
                exit(1);
            }
            fwrite (depth , sizeof(float),imgSize.x*imgSize.y, pFile);
            fclose (pFile);
            std::cout << "Written" << std::endl;
        }

        void WriteDepthPNM(std::string filename)
        {
            MonoPyramidLevel *level = currDepthFrame->dataImage;
            level->depth->UpdateHostFromDevice();
            SaveImageToFile(level->depth, filename.c_str());
        }



        ORUtils::Image<float>* GetDepthMap()
        {
            return currDepthFrame->dataImage->depth;
        }
        
            


        virtual void UpdateForPointCloud() = 0;


        virtual float EvaluateGT() = 0;

        virtual void UpdatePhotoError(ORUtils::SE3Pose refToTracker,
                                      ORUtils::TimeStampedImage<Vector4u> *frame) = 0;

        virtual void UpdatePhotoErrorWithFeatures(ORUtils::SE3Pose refToTracker,
                                                  ORUtils::TimeStampedImage<Vector4u> *frame,
                                                  ORUtils::MemoryBlock<float> *featureImage) = 0;


        virtual void RunTVOptimisationActive(float theta) = 0;
        virtual void RunTVOptimisation() = 0;
        virtual void RunTVL1Optimisation() = 0;
        virtual void InitOptim() = 0;
        virtual void SmoothDTAM() = 0;


        virtual void DisplayPhotoVolume(int x, int y) = 0;
        virtual float MeasureError() = 0;
        virtual void OptimToDepth(bool useRawDepth) = 0;


            



        bool isInitialised;
        MonoView *currDepthFrame;
        OptimPyramidLevel *optimPyramid;


        //is the process computing a depth map



        int updateNumber;
        TVParameters tvSettings;
    };
}
