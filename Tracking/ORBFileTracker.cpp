#include "ORBFileTracker.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <iostream>

ORBFileTracker::ORBFileTracker(const std::string &poseFilePath)
{
    InitORB(poseFilePath);
    useFrameCount = true;
}

void ORBFileTracker::InitORB(const std::string &poseFilePath)
{
    std::ifstream poseFile(poseFilePath.c_str());
    if (!poseFile.is_open())
    {
        std::cout << "Non-existant pose file" << std::endl;
        exit(1);
    }

    std::string line;
    bool endOfFile = false;
    long long count = 0;
    while (!endOfFile)
    {
        if (std::getline(poseFile, line).eof())
        {
            endOfFile = true;
            continue;
        }

        std::stringstream ss(line);
        Eigen::Matrix3f rot;
        Eigen::Vector3f trans;

        for (unsigned int r = 0; r < 3; r++)
        {
            for (unsigned int c = 0; c < 3; c++)
            {
                float val;
                ss >> val;
                rot(r,c) = val;
            }

            float val;
            ss >> val;

            trans[r] = val;
        }

        Sophus::SE3f pose;
        pose.setRotationMatrix(rot);
        pose.translation() = trans;

        timedPoses.push_back(std::make_pair(count, pose.inverse()));
        count++;
    }
}
