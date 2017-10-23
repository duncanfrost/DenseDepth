#include "DepthSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

DepthSource::DepthSource()
{
    frameNumber = 0;
}

void DepthSource::GetDepthForTimeStamp(long long timestamp)
{

}

