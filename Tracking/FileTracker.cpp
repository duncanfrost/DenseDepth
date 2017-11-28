#include "FileTracker.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <iostream>

Sophus::SE3f FileTracker::PoseAtTime(long long timeIn,
                                     long long frameCount,
                                     long long &timeOut)
{

    long long time = timeIn;
    if (useFrameCount)
        time = frameCount;



    int current = 0;
    long long time2 = timedPoses[current+1].first;

    while (time2 <= time && (unsigned int)(current+1) < timedPoses.size())
    {
        current++;
        time2 = timedPoses[current+1].first;
    }

    Sophus::SE3f poseMin = timedPoses[current].second;
    Sophus::SE3f poseMax = timedPoses[current+1].second;

    long long timeMin = timedPoses[current].first;
    long long timeMax = timedPoses[current+1].first;
    long long timeDist = timeMax - timeMin;

    float weightMax = (float)(timeIn - timeMin) / (float)(timeDist);

    std::cout << "Weight max: " << weightMax << std::endl;



    // if (timeIn  < timeMin || timeIn > timeMax)
    // {
    //     std::cout << "Problem" << std::endl;
    //     std::cout << "TimeMin: " << timeMin << std::endl;
    //     std::cout << "Time in: " << timeIn << std::endl;
    //     std::cout << "TimeMax: " << timeMax << std::endl;
    //     exit(1);
    // }

    Sophus::SE3f poseRelative = (poseMax * poseMin.inverse());
    Sophus::SE3f::Tangent muRelative = weightMax*Sophus::SE3f::log(poseRelative);



    Sophus::SE3f pose = Sophus::SE3f::exp(muRelative) * poseMin;

    long long diffMax = timeMax - timeIn;
    long long diffMin = timeIn - timeMin;

    if (diffMax > diffMin)
        timeOut = diffMax;
    else
        timeOut = diffMin;

    // timeOut = timeDist;

    

    return pose;
}
