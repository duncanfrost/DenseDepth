#include "ARKitFileTracker.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>


ARKitFileTracker::ARKitFileTracker(const std::string file)
{
    Init(file);
    useFrameCount = false;
}

void ARKitFileTracker::Init(const std::string file)
{
    std::ifstream poseFile(file.c_str());
    std::string line;
    bool endOfFile = false;
    Sophus::SE3f firstPose;

    if (!poseFile.is_open())
    {
        std::cout << "Non-existant pose file" << std::endl;
        exit(1);
    }


    while (!endOfFile)
    {
        if (std::getline(poseFile, line).eof())
        {
            endOfFile = true;
            continue;
        }

        if (line[0] == '#')
            continue;

        std::stringstream ss(line);
        std::string token;


        //Get the timestamp
        std::getline(ss, token, ' ');
        double timestampRaw = atof(token.c_str());

        Sophus::SE3f pose;

        Eigen::Matrix3f rot;
        Eigen::Vector3f translation;


        for (unsigned int r = 0; r < 3; r++)
        {
            //First get rotation matrix
            for (unsigned int c = 0; c < 3; c++)
            {
                std::getline(ss,token, ' ');
                rot(r,c) = atof(token.c_str());
            }

            //Then translation
            std::getline(ss,token, ' ');
            translation[r] = atof(token.c_str());
        }

        pose.translation = translation;
        pose.setRotationMatrix(rot);

        long long timestamp = timestampRaw * 1e6;

        timedPoses.push_back(std::make_pair(timestamp,pose));
    }
    poseFile.close();
}
