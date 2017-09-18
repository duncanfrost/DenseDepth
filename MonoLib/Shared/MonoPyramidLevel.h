#pragma once
#include <ORUtils/Image.h>
#include <ORUtils/SE3Pose.h>
#include <ORUtils/MathTypes.h>


#include <fstream>
#include <iostream>
#include <sstream>
#include "Intrinsics.h"

namespace MonoLib {


    class MonoPyramidLevel
    {

    public:
        MonoLib::Intrinsics intrinsics;

        //Data Images
        ORUtils::Image<float> *depth;
        ORUtils::Image<float> *gtDepth;
        ORUtils::Image<Vector3f> *pointRef;
        ORUtils::MemoryBlock<bool> *good;



        //How many photometric updates have we gone through.
        int nUpdate;


        MonoPyramidLevel(Vector2i imgSize, int levelId, MemoryDeviceType memoryType,
                         Intrinsics intrinsics, bool skipAllocation = false)
        {
            this->intrinsics = intrinsics;
            if (levelId == 0 && skipAllocation) return;

            bool allocateCPU = true;
            bool allocateCUDA = true;

            //output from the filters
            this->depth = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
            this->gtDepth = new ORUtils::Image<float>(imgSize, allocateCPU, allocateCUDA);
                        
            this->pointRef = new ORUtils::Image<Vector3f>(imgSize, allocateCPU, allocateCUDA);
            this->good = new ORUtils::Image<bool>(imgSize, allocateCPU, allocateCUDA);
        }

        ~MonoPyramidLevel()
        {
            //TODO -- delete all the stuff
        }


        void DepthFromOptim(float *a, float minIDepth, float maxIDepth)
        {
            size_t totalImgSize = depth->dataSize;
            float iDepthDiff = maxIDepth - minIDepth;
            for (size_t index = 0; index < totalImgSize; index++)
            {
                float inverseDepth = minIDepth + iDepthDiff * a[index];
                depth->GetData(MEMORYDEVICE_CPU)[index] = 1.0f/inverseDepth;
            }
        } 

        void Init()
        {
            float initDepth = 1.25;
            size_t totalImgSize = depth->dataSize;

            for (size_t index = 0; index < totalImgSize; index++)
                depth->GetData(MEMORYDEVICE_CPU)[index] = initDepth;
        }

        void DebugDisplay()
        {
            float *depth_data = depth->GetData(MEMORYDEVICE_CPU);

            unsigned int width = depth->noDims[0];
            unsigned int height = depth->noDims[1];
            for (unsigned int y = 0; y < height; y++)
            {
                for (unsigned int x = 0; x < width; x++)
                {
                    unsigned int index = x + width*y;
                    std::cout << depth_data[index] << " ";
                }
                std::cout << std::endl;
            }
        }

        void UpdatePointCloud()
        {
            float fxInv = intrinsics.fxInv;
            float fyInv = intrinsics.fyInv;
            float cxInv = intrinsics.cxInv;
            float cyInv = intrinsics.cyInv;

            unsigned int width = depth->noDims[0];
            unsigned int height = depth->noDims[1];

            float *depth_data = depth->GetData(MEMORYDEVICE_CPU);
            Vector3f *pointRefs = pointRef->GetData(MEMORYDEVICE_CPU);
            bool *goodData= good->GetData(MEMORYDEVICE_CPU);

            for (unsigned int y = 0; y < height; y++)
                for (unsigned int x = 0; x < width; x++)
                {
                    unsigned int index = x + width*y;

                    float pX = x * fxInv + cxInv;
                    float pY = y * fyInv + cyInv;

                    float depth = depth_data[index];
                    goodData[index] = true;
                    pointRefs[index] = depth * Vector3f(pX,pY,1);
                }
        }

        void WriteToCSV(std::string path)
        {
            std::ofstream myfile;
            myfile.open (path);

            unsigned int width = depth->noDims[0];
            unsigned int height = depth->noDims[1];

            float *data = depth->GetData(MEMORYDEVICE_CPU);
            for (unsigned int y = 0; y < height; y++)
            {
                for (unsigned int x = 0; x < width; x++)
                {
                    unsigned int index = x + width*y;
                    myfile << data[index];
                    if (x < width - 1)
                        myfile << ",";
                }
                if (y < height - 1)
                    myfile << std::endl;
            }
            myfile.close();
            std::cout << "Written" << std::endl;
        }

        void WriteToBinary(std::string path)
        {
            float *depth_data = depth->GetData(MEMORYDEVICE_CPU);
            FILE * pFile;
            pFile = fopen (path.c_str(), "wb");
            fwrite (depth_data , sizeof(float),depth->noDims.x *depth->noDims.y,
                    pFile);
            fclose (pFile);
            std::cout << "Written to binary"  << std::endl;
        }


        void ReadFromBinary(std::string path)
        {
            float *depth_data = depth->GetData(MEMORYDEVICE_CPU);
            FILE * pFile;
            pFile = fopen (path.c_str(), "rb");
            fread(depth_data,sizeof(float),depth->noDims.x *depth->noDims.y, pFile);
            fclose (pFile);
            std::cout << "Read from binary"  << std::endl;
        }


        void EvalFromBinary(std::string path)
        {
            int totalDepthElements = depth->noDims.x*depth->noDims.y;
            float *depth_data = depth->GetData(MEMORYDEVICE_CPU);
            float *inputDepth = new float[totalDepthElements];
            FILE * pFile;
            pFile = fopen (path.c_str(), "rb");
            fread(inputDepth,sizeof(float),depth->noDims.x *depth->noDims.y, pFile);
            fclose(pFile);

            float error = 0;
            int testedDepths = 0;
            for (int i = 0; i < totalDepthElements; i++)
            {
                if (depth_data[i] != depth_data[i])
                    continue;
                if (inputDepth[i] != inputDepth[i])
                    continue;
                float res = inputDepth[i] - depth_data[i];
                error += res*res;
                testedDepths++;
            }
            std::cout << "Error: " << sqrt(error/testedDepths) << std::endl;
        }

        void ReadFromCSV(std::string path)
        {
            std::ifstream myfile;
            myfile.open (path);

            std::string line;
            int row = 0;
            float *data = depth->GetData(MEMORYDEVICE_CPU);
            unsigned int width = depth->noDims[0];

            while (std::getline(myfile,line))
            {
                int col = 0;
                std::stringstream lineStream(line);
                std::string cell;
                while(std::getline(lineStream,cell, ','))
                {

                    float value = atof(cell.c_str());
                    data[col + width*row] = value;
                    col++;
                }

                row++;
            }

            depth->UpdateDeviceFromHost();
        }



        void UpdateHostFromDevice()
        {
            gtDepth->UpdateHostFromDevice();
            pointRef->UpdateHostFromDevice();
            good->UpdateHostFromDevice();
            depth->UpdateHostFromDevice();
        }

        void UpdateDeviceFromHost()
        {
            good->UpdateDeviceFromHost();
            depth->UpdateDeviceFromHost();
            gtDepth->UpdateDeviceFromHost();
            pointRef->UpdateDeviceFromHost();
        }
    };
}
