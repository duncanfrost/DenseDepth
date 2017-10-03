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

    Sophus::SE3f pose = timedPoses[current].second;
    timeOut = timedPoses[current].first;
    return pose;
}
