#include "FusionEngine.h"
#include <ORUtils/MathTypes.h>
#include "ActiveFunctions.h"

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
    source->SetFrameNumber(0);
}


void FusionEngine::Process()
{
    source->GrabNewFrame(false);
    image = source->Image();
    timeStamp = source->TimeStamp();

    long long count = source->FrameNumber();
    if (count > 1700)
        return;


    long long timeOut;
    currPose = tracker->PoseAtTime(count, timeOut);

    currTrackerData->trackerPose = currPose;
    currTrackerData->frame = image;

    std::cout << "Count: " << count << std::endl;

    if (true)
    {
        std::cout << "Fusion active" << std::endl;
        MakePointCloud();
    }

    framesProcessed++;
}

void FusionEngine::MakePointCloud()
{

    // if (framesProcessed % 20 != 0)
    //     return;

    std::stringstream inPath;
    inPath << "/home/duncan/Data/P9/Office/depth3/" << timeStamp << "000000.png";

    std::cout << "Path:"  << std::endl;
    std::cout << inPath.str()  << std::endl;
    cv::Mat depthRaw = cv::imread(inPath.str(), CV_LOAD_IMAGE_ANYDEPTH );

    if (depthRaw.rows == 0)
    {
        return;
    }

    cv::Mat depth;
    cv::resize(depthRaw, depth, imgSize);

    cv::Mat imageResized;
    cv::resize(image, imageResized, imgSize);

    cv::Mat depthFloat; 
    depth.convertTo(depthFloat, CV_32FC1); 




    // std::stringstream certPath;
    // certPath << "/home/duncan/Data/P9/SidewaysLong/cert/" << timeStamp << "000000.png";
    // cv::Mat certRaw = cv::imread(certPath.str(), CV_LOAD_IMAGE_ANYDEPTH );
    // cv::Mat certIm;
    // cv::resize(certRaw, certIm, imgSize);
    
    // cv::Mat certFloat; 
    // certIm.convertTo(certFloat, CV_32FC1); 

    // certFloat = certFloat / 5000;
    // certFloat = certFloat - 10;

    // std::cout << certFloat  << std::endl;
    // exit(1);

    

    Sophus::SE3f invPose = currPose.inverse();

    for (int y = 0; y < depthFloat.rows; y++)
        for (int x = 0; x < depthFloat.cols; x++)
        {
            float depth = depthFloat.at<float>(y,x) / 5000;
            // float cert = certFloat.at<float>(y,x);

            if (depth < 0.3)
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
