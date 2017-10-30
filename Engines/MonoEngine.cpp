#include "MonoEngine.h"
#include <ORUtils/MathTypes.h>
#include "ActiveFunctions.h"
#include "ORConvert.h"

Sophus::SE3f MonoEngine::invRefPose;

MonoEngine::MonoEngine(ImageSource* source, DepthSource* depthSource,
                       FileTracker *tracker, Settings settings)
{
    this->source = source;
    this->tracker = tracker;
    this->depthSource = depthSource;

    
    currTrackerData = new TrackerData();
    map = new GlobalMap();

    // imgSize.x = 640;
    // imgSize.y = 480;

    imgSize.x = 160;
    imgSize.y = 120;

    Vector4f intrinsics;
    float fx = (settings.fx/640)*imgSize.x;
    float fy = (settings.fy/480)*imgSize.y;
    float cx = (settings.cx/640)*imgSize.x;
    float cy = (settings.cy/480)*imgSize.y;

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
    nMid = BUFFERSIZE/2; 
    paused = false;
}


void MonoEngine::Process()
{
    if (paused)
        return;

    source->GrabNewFrame();
    cv::Mat rawImage = source->Image();
    image = PreProcessImage(rawImage);
    timeStamp = source->TimeStamp();
    long long count = source->FrameNumber();
    long long timeOut;


    currPose = tracker->PoseAtTime(timeStamp, count, timeOut);

    ConvertToOR(image, orImage);


    // Sample();
    SaveToBuffer(orImage, currPose);


    // if (needsKeyFrame)
    // {
        // AddKeyFrame(orImage, currPose);
        // needsKeyFrame = false;
    // }

    if (SampleActive(count, BUFFERSIZE))
    {
        SmoothPhotoBuffer(200);
        // WritePhotoErrors("/home/duncan/photo.bin");
        // exit(1);
    }

    std::cout << "Framenumber: " << count << std::endl;
        

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
    // monoDepthEstimator->RunTVOptimisation(iterations);
    monoDepthEstimator->RunTVL1Optimisation(iterations);
}

void MonoEngine::SmoothPhotoBuffer(int iterations)
{
    SampleFromBufferMid();
    SmoothPhoto(iterations);


    //Update images from device
    monoDepthEstimator->optimPyramid->certainty->UpdateHostFromDevice();
    monoDepthEstimator->currDepthFrame->dataImage->depth->UpdateHostFromDevice();
    monoDepthEstimator->optimPyramid->nUpdates->UpdateHostFromDevice();


    long long timestamp = timeStampBuffer[nMid];

    if (depthSource != NULL)
        depthSource->GetDepthForTimeStamp(timestamp);
    else
        std::cout << "No depth source" << std::endl;


    paused = true;


    // cv::Mat testIm(imgSize.y, imgSize.x, CV_16UC1); 
    // // ORToCVConvertUpdates(monoDepthEstimator->currDepthFrame->dataImage->depth,
    //                      monoDepthEstimator->optimPyramid->nUpdates,
                         // testIm);


    // float minVal = 999;

    // for (int y = 0; y < monoDepthEstimator->optimPyramid->certainty->noDims.y; y++)
    // {
    //     for (int x = 0; x < monoDepthEstimator->optimPyramid->certainty->noDims.x; x++)
    //     {
    //         int index = x + monoDepthEstimator->optimPyramid->certainty->noDims.x*y;
    //         float val = monoDepthEstimator->optimPyramid->certainty->GetData(MEMORYDEVICE_CPU)[index];
    //         if (val < minVal)
    //             minVal = val;
    //     }
    // }
        
    // std::cout << "MIN VAL: " << minVal << std::endl;

    // cv::Mat certIm(imgSize.y, imgSize.x, CV_16UC1);
    // ORToCVConvert(monoDepthEstimator->optimPyramid->certainty,
    //               certIm,5000);

    
    // std::stringstream outPath;
    // outPath << "/home/duncan/Data/P9/Office3/depth3/" << timeStampBuffer[nMid] << "000000.png";

    
    // cv::Mat imUp;
    // cv::resize(testIm, imUp, cv::Size(), 4.0f, 4.0f);
    // cv::imwrite(outPath.str(), imUp);


    // std::stringstream outPathCert;
    // outPathCert << "/home/duncan/Data/P9/Office3/cert/" << timeStampBuffer[nMid] << "000000.png";
    
    // cv::resize(certIm, imUp, cv::Size(), 4.0f, 4.0f);
    // cv::imwrite(outPathCert.str(), imUp);



}

void MonoEngine::WriteEmpty()
{
    cv::Mat testIm(imgSize.y, imgSize.x, CV_16UC1); 
    std::stringstream outPath;
    outPath << "/home/duncan/Data/P9/Office3/depth3/" << timeStamp << "000000.png";
    cv::Mat imUp;
    cv::resize(testIm, imUp, cv::Size(), 4.0f, 4.0f);
    cv::imwrite(outPath.str(), imUp);
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
                               Vector4u **colorData)
{

    this->MakePointCloud(useRawDepth);

    MonoLib::MonoPyramidLevel *dataPyramidLevel =
        monoDepthEstimator->currDepthFrame->dataImage;

    *points = dataPyramidLevel->pointRef->GetData(MEMORYDEVICE_CPU);
    *colorData = monoDepthEstimator->currDepthFrame->colorImageData->GetData(MEMORYDEVICE_CPU);

    // monoDepthEstimator->currDepthFrame->colorImageData->UpdateHostFromDevice();

    // std::cout << "Color data here: " << (int)monoDepthEstimator->currDepthFrame->colorImageData->GetData(MEMORYDEVICE_CPU)[0][0] << std::endl;

    width = dataPyramidLevel->depth->noDims[0];
    height = dataPyramidLevel->depth->noDims[1];
}


void MonoEngine::SampleFromBufferMid()
{
    ORUChar4TSImage *rgbImage = imageBuffer[nMid];
    Sophus::SE3f kfPose = poseBuffer[nMid];



    AddKeyFrame(rgbImage, kfPose);

    for (int i = 0; i < BUFFERSIZE; i++)
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

    std::cout << "Timestamp here: " << timeStamp << std::endl;
    std::cout << "mid timestamp:" << timeStampBuffer[nMid] << std::endl;
    std::cout << "Nmid: " << nMid << std::endl;


    if (bufferTop < BUFFERSIZE)
        bufferTop++;
}


void MonoEngine::WritePhotoErrors(std::string path)
{
    monoDepthEstimator->WritePhotoErrors(path);
}

cv::Mat MonoEngine::PreProcessImage(cv::Mat image)
{
    cv::Size outSize;
    outSize.width = imgSize.x;
    outSize.height = imgSize.y;
    
    cv::Mat imResized;
    cv::resize(image, imResized, outSize);

    cv::Size kernelSize;
    kernelSize.width = 3;
    kernelSize.height = 3;

    cv::Mat imOut;
    cv::GaussianBlur(imResized, imOut, kernelSize, 3);


    return imOut;
}
