#include "TUMFileTracker.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>


TUMFileTracker::TUMFileTracker(const std::string &directory, const std::string &listName)
{
    Init(directory, listName);
    useFrameCount = false;
}

void TUMFileTracker::Init(const std::string &directory, const std::string &listName)
{
    std::string fullListPath = directory + listName;
    std::ifstream poseFile(fullListPath.c_str());
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

        std::getline(ss, token, ' ');
        double timestampRaw = atof(token.c_str());

        Sophus::SE3f pose;
        for (unsigned int i = 0; i < 3; i++)
        {
            std::getline(ss,token, ' ');
            pose.translation()[i] = atof(token.c_str());
        }

        Eigen::Quaternionf q;
        std::getline(ss,token, ' ');
        q.x() = atof(token.c_str());
        std::getline(ss,token, ' ');
        q.y() = atof(token.c_str());
        std::getline(ss,token, ' ');
        q.z() = atof(token.c_str());
        std::getline(ss,token, ' ');
        q.w() = atof(token.c_str());
        pose.setQuaternion(q);

        pose = pose.inverse();


        if (timedPoses.size() == 0)
            firstPose = pose;

        pose = pose * firstPose.inverse();

        std::cout << "Timestamp: " << std::setprecision(15) << timestampRaw << std::endl;

        long long timestamp = timestampRaw * 1e6;

        timedPoses.push_back(std::make_pair(timestamp,pose));
    }
    poseFile.close();
}
