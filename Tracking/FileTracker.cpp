#include "FileTracker.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <iostream>

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
