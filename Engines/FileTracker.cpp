#include "FileTracker.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <iostream>

FileTracker::FileTracker(const std::string &poseFilePath, FileType type)
{

    if (type == FileTracker::TUM)
        InitTUM(poseFilePath);
    else
        InitORB(poseFilePath);
}

void FileTracker::InitTUM(const std::string &poseFilePath)
{

    std::cout << "Function needs to be written" << std::endl;
    exit(1);

    // std::ifstream poseFile(poseFilePath.c_str());
    // if (!poseFile.is_open())
    // {
    //     std::cout << "Non-existant pose file" << std::endl;
    //     exit(1);
    // }
                

    // std::string line;
    // bool endOfFile = false;
    // ORUtils::SE3Pose firstPose;

    // Matrix4f firstMInv = firstPose.GetInvM();

    // while (!endOfFile)
    // {
    //     if (std::getline(poseFile, line).eof())
    //     {
    //         endOfFile = true;
    //         continue;
    //     }

    //     if (line[0] == '#')
    //         continue;

    //     std::stringstream ss(line);
    //     std::string token;

    //     std::getline(ss, token, ' ');
    //     double timeStamp = atof(token.c_str());

    //     Vector3f trans;
    //     Vector4f quart;

    //     for (unsigned int i = 0; i < 3; i++)
    //     {
    //         std::getline(ss, token, ' ');
    //         trans[i] = (float)atof(token.c_str());
    //     }
    //     std::getline(ss, token, ' ');
    //     quart[0] = (float)atof(token.c_str());
    //     std::getline(ss, token, ' ');
    //     quart[1] = (float)atof(token.c_str());
    //     std::getline(ss, token, ' ');
    //     quart[2] = (float)atof(token.c_str());
    //     std::getline(ss, token, ' ');
    //     quart[3] = (float)atof(token.c_str());

    //     Matrix4f invM;
    //     invM = setRotate(quart, trans);

    //     Matrix4f m;
    //     invM.inv(m);


    //     if (timedPoses.size() == 0)
    //         firstMInv = invM;

    //     m = m * firstMInv;

    //     timedPoses.push_back(std::make_pair((long long)(timeStamp * 1e6), m));
    // }
    // poseFile.close();
    // std::cout << "Got to the end of " << poseFilePath << std::endl;
}

void FileTracker::InitORB(const std::string &poseFilePath)
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

Sophus::SE3f FileTracker::PoseAtTime(long long time, long long &timeOut)
{
    int current = 0;
    long long time2 = timedPoses[current+1].first;

    while (time2 <= time)
    {
        current++;
        time2 = timedPoses[current+1].first;
    }

    Sophus::SE3f pose = timedPoses[current].second;
    timeOut = timedPoses[current].first;
    return pose;
}
