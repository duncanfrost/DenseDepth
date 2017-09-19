#include "MonoEngine.h"
#include <ORUtils/MathTypes.h>

Sophus::SE3f MonoEngine::invRefPose;

inline ORUtils::SE3Pose SophusToOR(Sophus::SE3f pose)
{
    ORUtils::SE3Pose out;

    Matrix4f mat;
    
    for (unsigned int r = 0; r < 4; r++)
        for (unsigned int c = 0; c < 4; c++)
        {
            mat.at(c,r) = pose.matrix()(r,c);
        }

    out.SetM(mat);
    
    return out;
}

inline void ORToCV(ORUtils::Image<float> *imageIn, cv::Mat in)
{
    for (int y = 0; y < imageIn->noDims.y; y++)
        for (int x = 0; x < imageIn->noDims.x; x++)
        {
            int index = x + imageIn->noDims.x*y;
            in.at<float>(y,x) = imageIn->GetData(MEMORYDEVICE_CPU)[index]; 
        }
}

inline void ORToCVConvert(ORUtils::Image<float> *imageIn, cv::Mat in)
{
    for (int y = 0; y < imageIn->noDims.y; y++)
        for (int x = 0; x < imageIn->noDims.x; x++)
        {
            int index = x + imageIn->noDims.x*y;
            in.at<short>(y,x) = 5000*imageIn->GetData(MEMORYDEVICE_CPU)[index]; 
        }
}

MonoEngine::MonoEngine(PhoneSource* source, FileTracker* tracker)
{
    this->source = source;
    this->tracker = tracker;
    currTrackerData = new TrackerData();
    map = new GlobalMap();

    imgSize.x = 640;
    imgSize.y = 480;
    Vector4f intrinsics;


    float fx = 683.8249;
    float fy = 683.6179;
    float cx = 317.6438;
    float cy = 239.5907;


    intrinsics[0] = fx;
    intrinsics[1] = fy;
    intrinsics[2] = cx;
    intrinsics[3] = cy;
    
    monoDepthEstimator = new MonoLib::MonoDepthEstimator_CUDA(imgSize, intrinsics);

    orImage = new ORUChar4TSImage(imgSize, true, true, true);


    for (unsigned int i = 0; i < BUFFERSIZE; i++)
        imageBuffer[i] = new ORUChar4TSImage(imgSize, true, true, true);


    hasReferenceFrame = false;
    useRawDepth = true;
    needsKeyFrame = false;
    bufferTop = 0;
    framesProcessed = 0;
}


void MonoEngine::Process()
{
    source->GrabNewFrame();
    image = source->Image();
    timeStamp = source->TimeStamp();

    long long count = source->FrameNumber();

    std::cout << "Timestamp: " << timeStamp << std::endl;

    long long timeOut;
    currPose = tracker->PoseAtTime(count, timeOut);

    ConvertToOR(image, orImage);


    // Sample();
    SaveToBuffer(orImage, currPose);


    // if (needsKeyFrame)
    // {
        // AddKeyFrame(orImage, currPose);
        // needsKeyFrame = false;
    // }

    if (framesProcessed > 102)
        SmoothPhotoBuffer(200);

    currTrackerData->trackerPose = currPose;
    currTrackerData->frame = image;

    framesProcessed++;
}


void MonoEngine::AddKeyFrame(ORUChar4TSImage *inImage, Sophus::SE3f inPose)
{
    std::cout << "About to make new keyframe" << std::endl;


    std::cout << "In Image: " << inImage << std::endl;
    Vector4u data = inImage->GetData(MEMORYDEVICE_CPU)[0];
    std::cout << "Data: " << (int)data[0] << std::endl;


    KeyFrame *kf = new KeyFrame();
    kf->pose = inPose;
    map->keyframeList.push_back(kf);

    inImage->UpdateDeviceFromHost();
    monoDepthEstimator->SetRefImage(inImage);
    invRefPose = kf->pose.inverse();

    monoDepthEstimator->SetLimitsManual(0.5,2);
    hasReferenceFrame = true;
}

void MonoEngine::ConvertToOR(cv::Mat inImage, ORUChar4TSImage *outImage)
{
    for (int y = 0; y < inImage.rows; y++)
    {
        for (int x = 0; x < inImage.cols; x++)
        {
            cv::Vec3b val = inImage.at<cv::Vec3b>(y,x);
            Vector4u orVal;
            orVal[0] = val[2];
            orVal[1] = val[1];
            orVal[2] = val[0];
            orVal[3] = 0;

            int index = x + inImage.cols*y;
            outImage->GetData(MEMORYDEVICE_CPU)[index] = orVal;
        }
    }

    outImage->UpdateDeviceFromHost();
}

void MonoEngine::Sample()
{
    if (!hasReferenceFrame)
        return;


    std::cout << "Sampling" << std::endl;
    Sophus::SE3f inPose = currPose*invRefPose;
    monoDepthEstimator->UpdatePhotoError(SophusToOR(inPose), orImage);
}

void MonoEngine::SmoothPhoto(int iterations)
{
    monoDepthEstimator->RunTVOptimisation(iterations);
}

void MonoEngine::SmoothPhotoBuffer(int iterations)
{
    SampleFromBufferMid();
    SmoothPhoto(iterations);

    monoDepthEstimator->currDepthFrame->dataImage->depth->UpdateHostFromDevice();


    cv::Mat testIm(imgSize.y, imgSize.x, CV_16UC1); 
    ORToCVConvert(monoDepthEstimator->currDepthFrame->dataImage->depth, testIm);


    std::stringstream outPath;
    outPath << "/home/duncan/Data/P9/SidewaysLong/depth/" << timeStampBuffer[50] << "000000.png";
    cv::imwrite(outPath.str(), testIm);

    // cv::namedWindow( "Debug", cv::WINDOW_AUTOSIZE );// Create a window for display.
    // cv::imshow( "Debug", testIm );                   // Show our image inside it.
    // cv::waitKey(0); 
    // cv::destroyWindow("Debug");






}


void MonoEngine::MakePointCloud(bool useRawDepth)
{
    // If you want to use raw depth, don't use OptimToDepth
    // inside monodepthestimator and use it here

    // monoDepthEstimator->OptimToDepth(useRawDepth);
    monoDepthEstimator->UpdateForPointCloud();
    monoDepthEstimator->currDepthFrame->MakePointCloud();
}

void MonoEngine::GetPointCloud(unsigned int &width,
                               unsigned int &height, Vector3f **points,
                               Vector4u **colorData, bool **goodData)
{

    this->MakePointCloud(useRawDepth);

    MonoLib::MonoPyramidLevel *dataPyramidLevel =
        monoDepthEstimator->currDepthFrame->dataImage;

    *points = dataPyramidLevel->pointRef->GetData(MEMORYDEVICE_CPU);
    *goodData = dataPyramidLevel->good->GetData(MEMORYDEVICE_CPU);
    *colorData = monoDepthEstimator->currDepthFrame->colorImageData->GetData(MEMORYDEVICE_CPU);

    // monoDepthEstimator->currDepthFrame->colorImageData->UpdateHostFromDevice();

    // std::cout << "Color data here: " << (int)monoDepthEstimator->currDepthFrame->colorImageData->GetData(MEMORYDEVICE_CPU)[0][0] << std::endl;

    width = dataPyramidLevel->depth->noDims[0];
    height = dataPyramidLevel->depth->noDims[1];
}


void MonoEngine::SampleFromBufferMid()
{
    unsigned int nMid = 50; 
    ORUChar4TSImage *rgbImage = imageBuffer[nMid];
    Sophus::SE3f kfPose = poseBuffer[nMid];


    AddKeyFrame(rgbImage, kfPose);


    // monoDepthEstimator->optimPyramid->g->UpdateHostFromDevice();
    // cv::Mat testIm(imgSize.y, imgSize.x, CV_32FC1); 
    // ORToCV(monoDepthEstimator->optimPyramid->g, testIm);
    // std::cout << testIm << std::endl;
    // cv::namedWindow( "Debug", cv::WINDOW_AUTOSIZE );// Create a window for display.
    // cv::imshow( "Debug", testIm );                   // Show our image inside it.
    // cv::waitKey(0); 
    // cv::destroyWindow("Debug");



    for (unsigned int i = 0; i < BUFFERSIZE; i++)
    {
        if (i == nMid)
            continue;
        // std::cout << "Sampling: " << i << std::endl;
        Sophus::SE3f trackingPose = poseBuffer[i];
        Sophus::SE3f inPose = trackingPose*invRefPose;
        ORUChar4TSImage *inputRGBImage = imageBuffer[i];
        inputRGBImage->UpdateDeviceFromHost();
        monoDepthEstimator->UpdatePhotoError(SophusToOR(inPose), inputRGBImage);
    }
}

void MonoEngine::SaveToBuffer(ORUChar4TSImage *inputRGBImage,
                              Sophus::SE3f inputPose)
{
    std::cout << "Frames processed: " << framesProcessed << std::endl;

    ORUChar4TSImage *lastImage = imageBuffer[BUFFERSIZE - 1];

    for (int i = BUFFERSIZE-1; i > 0; i--)
    {
        imageBuffer[i] = imageBuffer[i-1];
        poseBuffer[i] = poseBuffer[i-1];
        timeStampBuffer[i] = timeStampBuffer[i-1];
    }

    //No need to do this for the pose buffer
    imageBuffer[0] = lastImage;
    imageBuffer[0]->SetFrom(inputRGBImage, MEMCPYDIR_CPU_TO_CPU);

    poseBuffer[0] = inputPose;
    timeStampBuffer[0] = timeStamp;
    if (bufferTop < BUFFERSIZE)
        bufferTop++;
}
