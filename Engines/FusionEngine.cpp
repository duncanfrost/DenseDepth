#include "FusionEngine.h"
#include <ORUtils/MathTypes.h>

FusionEngine::FusionEngine(PhoneSource* source, FileTracker* tracker)
{
    this->source = source;
    this->tracker = tracker;
    currTrackerData = new TrackerData();
    map = new GlobalMap();


    imgSize.width = 160;
    imgSize.height = 120;


    Vector4f intrinsics;
    float fx = (683.8249/640)*imgSize.width;
    float fy = (683.6179/480)*imgSize.height;
    float cx = (317.6438/640)*imgSize.width;
    float cy = (239.5907/480)*imgSize.height;

    intrinsics[0] = fx;
    intrinsics[1] = fy;
    intrinsics[2] = cx;
    intrinsics[3] = cy;


    fxInv = 1.0f / fx;
    fyInv = 1.0f / fy;

    cxInv = -1.0f*fxInv*cx;
    cyInv = -1.0f*fyInv*cy;

    framesProcessed = 0;
}


void FusionEngine::Process()
{
    source->GrabNewFrame();
    image = source->Image();
    timeStamp = source->TimeStamp();

    long long count = source->FrameNumber();

    std::cout << "Timestamp: " << timeStamp << std::endl;

    long long timeOut;
    currPose = tracker->PoseAtTime(count, timeOut);

    currTrackerData->trackerPose = currPose;
    currTrackerData->frame = image;

    if (framesProcessed > 150 && framesProcessed % 30 == 0 )
        MakePointCloud();

    framesProcessed++;
}

void FusionEngine::MakePointCloud()
{
    std::stringstream inPath;
    inPath << "/home/duncan/Data/P9/SidewaysLong/depth3/" << timeStamp << "000000.png";

    cv::Mat depthRaw = cv::imread(inPath.str(), CV_LOAD_IMAGE_ANYDEPTH );
    cv::Mat depth;
    cv::resize(depthRaw, depth, imgSize);

    cv::Mat imageResized;
    cv::resize(image, imageResized, imgSize);

    cv::Mat depthFloat; 
    depth.convertTo(depthFloat, CV_32FC1); 

    Sophus::SE3f invPose = currPose.inverse();

    for (int y = 0; y < depthFloat.rows; y++)
        for (int x = 0; x < depthFloat.cols; x++)
        {
            float depth = depthFloat.at<float>(y,x) / 5000;

            if (depth < 0.0001)
                continue;
            cv::Vec3b color = imageResized.at<cv::Vec3b>(y,x);

            MapPoint *mp = new MapPoint();
            mp->c1 = color[2];
            mp->c2 = color[1];
            mp->c3 = color[0];

            float pX = x * fxInv + cxInv;
            float pY = y * fyInv + cyInv;

            Eigen::Vector3f posFrame = depth * Eigen::Vector3f(pX,pY,1);
            mp->position = invPose * posFrame;  

            map->mappoints.push_back(mp);
        }

}