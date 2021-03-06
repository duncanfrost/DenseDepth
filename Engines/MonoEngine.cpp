#include "MonoEngine.h"
#include <ORUtils/MathTypes.h>
#include "ActiveFunctions.h"
#include "ORConvert.h"
#include "Timer.h"

Sophus::SE3f MonoEngine::invRefPose;


MonoEngine::MonoEngine(ImageSource* source, DepthSource* depthSource,
                       TUMFeatureSource* featureSource,
                       FileTracker *tracker, Settings settings)
{
    this->featureSource = featureSource;
    this->source = source;
    this->tracker = tracker;
    this->depthSource = depthSource;
    this->settings = settings;
    Init();

    featureChannels = 64;
    featureHeight = settings.targetSizeY;
    featureWidth = settings.targetSizeX;
    featureImage = new ORUtils::MemoryBlock<float>(featureChannels*featureHeight*featureWidth, true, true, true);
    featureSource->SetData(featureImage->GetData(MEMORYDEVICE_CPU));
}

MonoEngine::MonoEngine(ImageSource* source, DepthSource* depthSource,
                       FileTracker *tracker, Settings settings)
{
    this->source = source;
    this->tracker = tracker;
    this->depthSource = depthSource;
    this->settings = settings;
    this->featureSource = NULL;
    Init();
}

void MonoEngine::Init()
{
    currTrackerData = new TrackerData();
    currDepthData = new TrackerData();


    map = new GlobalMap();

    //This is the target size
    imgSize.x = settings.targetSizeX;
    imgSize.y = settings.targetSizeY;

    currDepthData->frame = cv::Mat(imgSize.y, imgSize.x, CV_8UC3);

    Vector4f intrinsics;
    float fx = (settings.fx/(float)settings.inputSizeX)*(float)imgSize.x;
    float fy = (settings.fy/(float)settings.inputSizeY)*(float)imgSize.y;
    float cx = (settings.cx/(float)settings.inputSizeX)*(float)imgSize.x;
    float cy = (settings.cy/(float)settings.inputSizeY)*(float)imgSize.y;

    intrinsics[0] = fx;
    intrinsics[1] = fy;
    intrinsics[2] = cx;
    intrinsics[3] = cy;
    
    monoDepthEstimator = new MonoLib::MonoDepthEstimator_CUDA(imgSize, intrinsics);
    orImage = new ORUChar4TSImage(imgSize, true, true, true);


    source->SetFrameNumber(1);
    if (featureSource)
        featureSource->SetFrameNumber(1);


    hasReferenceFrame = false;
    useRawDepth = true;
    needsKeyFrame = true;
    bufferTop = 0;
    framesProcessed = 0;
    nMid = BUFFERSIZE/2; 
    paused = false;
    processCount = 0;


    //Active processing stuff
    thetaEnd = 1e-4;
    beta = 0.002;
    theta = 0.2;
    processActive = false;

    depthErrorFile.open("/home/duncan/deptherror.txt");

    goodPoint = new bool[imgSize.x*imgSize.y];
}


void MonoEngine::Process()
{
    if (paused)
        return;

    source->GrabNewFrame();

    if (featureSource)
    {
        featureSource->GrabNewFrame();
        // featureSource->GrabNewFrameDebug();
    }

    cv::Mat rawImage = source->Image();
    currImage = PreProcessImage(rawImage);
    timeStamp = source->TimeStamp();

    std::cout << "Timestamp: " << timeStamp << std::endl;

    long long count = source->FrameNumber();
    long long timeOut;
    currPose = tracker->PoseAtTime(timeStamp, count, timeOut);

    std::cout << "Time out: " << timeOut << std::endl;

    if (timeOut < 8000 || !settings.checkTimeDiff)
        // ProcessBuffer();
    ProcessKeyFrame();
    else
    {
        std::cout << "====================PROBLEM===============" << std::endl;
        needsKeyFrame = true;
        processCount = 0;
    }
        





    currTrackerData->trackerPose = currPose;
    currTrackerData->frame = currImage;

    framesProcessed++;
}

void MonoEngine::ProcessKeyFrame()
{
    Sample();

    if (needsKeyFrame)
    {
        AddKeyFrame(currImage, currPose);
        needsKeyFrame = false;
        processCount = 0;
    }

    if (processCount > BUFFERSIZE)
    {
        SmoothPhoto();
        needsKeyFrame = true;
        exit(1);
    }


    processCount++;
}

void MonoEngine::ProcessBuffer()
{
    SaveToBuffer();
    if (SampleActive(processCount, BUFFERSIZE))
    {
        SmoothPhotoBuffer();
    }
    processCount++;
}


void MonoEngine::AddKeyFrame(cv::Mat inImage, Sophus::SE3f inPose)
{
    std::cout << "About to make new keyframe" << std::endl;

    KeyFrame *kf = new KeyFrame();
    kf->pose = inPose;
    map->keyframeList.push_back(kf);

    ConvertToOR(inImage, orImage);
    orImage->UpdateDeviceFromHost();
    if (featureSource)
    {
        featureImage->UpdateDeviceFromHost();
        monoDepthEstimator->SetRefAndFeatureImage(orImage, featureImage);
    }
    else
        monoDepthEstimator->SetRefImage(orImage);
    
    invRefPose = kf->pose.inverse();
    timeStampRef = timeStamp;

    monoDepthEstimator->SetLimitsManual(0.3,3);
    hasReferenceFrame = true;
}


void MonoEngine::Sample()
{
    if (!hasReferenceFrame)
        return;


    std::cout << "Sampling" << std::endl;
    Sophus::SE3f inPose = currPose*invRefPose;
    ConvertToOR(currImage, orImage);
    orImage->UpdateDeviceFromHost();
    if (featureSource)
    {
        featureImage->UpdateDeviceFromHost();
        monoDepthEstimator->UpdatePhotoErrorWithFeatures(SophusToOR(inPose),
                                                         orImage, featureImage);
    }
    else
        monoDepthEstimator->UpdatePhotoError(SophusToOR(inPose), orImage);
}

void MonoEngine::SmoothPhoto()
{
    if (settings.useTVSmoothing)
        monoDepthEstimator->RunTVOptimisation();
    else
        monoDepthEstimator->InitOptim();
    MeasureDepthError();
    ProcessDepthData();
}

void MonoEngine::SmoothPhotoBuffer()
{
    SampleFromBufferMid();


    SmoothPhoto();


    // Update images from device
    monoDepthEstimator->optimPyramid->certainty->UpdateHostFromDevice();
    monoDepthEstimator->currDepthFrame->dataImage->depth->UpdateHostFromDevice();
    monoDepthEstimator->optimPyramid->nUpdates->UpdateHostFromDevice();

    float error = monoDepthEstimator->MeasureError();
    std::cout << "Error: " << error << std::endl;


    // long long timestamp = timeStampBuffer[nMid];
    // LoadGTDepth(timestamp);

    error = monoDepthEstimator->MeasureError();
    std::cout << "Error after depth load: " << error << std::endl;

    // VisualizeDepth();
    // paused = true;
}

void MonoEngine::WriteEmpty()
{
    cv::Mat testIm(imgSize.y, imgSize.x, CV_16UC1); 
    std::stringstream outPath;

    monoDepthEstimator->optimPyramid->d->UpdateDeviceFromHost();
    monoDepthEstimator->OptimToDepth(false);

    outPath << "/home/duncan/Data/P9/Office3/depth3/" << timeStamp << "000000.png";
    cv::Mat imUp;
    cv::resize(testIm, imUp, cv::Size(), 4.0f, 4.0f);
    cv::imwrite(outPath.str(), imUp);
}


void MonoEngine::MakePointCloud()
{
    // If you want to use raw depth, don't use OptimToDepth
    // inside monodepthestimator and use it here

    // monoDepthEstimator->OptimToDepth(useRawDepth);
    monoDepthEstimator->UpdateForPointCloud();
    monoDepthEstimator->currDepthFrame->MakePointCloud();
}

void MonoEngine::GetPointCloud(unsigned int &width,
                               unsigned int &height, Vector3f **points,
                               bool **goodData,
                               Vector4u **colorData)
{
    this->MakePointCloud();
    MonoLib::MonoPyramidLevel *dataPyramidLevel =
        monoDepthEstimator->currDepthFrame->dataImage;

    *points = dataPyramidLevel->pointRef->GetData(MEMORYDEVICE_CPU);
    *colorData = monoDepthEstimator->currDepthFrame->colorImageData->GetData(MEMORYDEVICE_CPU);
    monoDepthEstimator->optimPyramid->nUpdates->UpdateHostFromDevice();
    dataPyramidLevel->depth->UpdateHostFromDevice();
    for (int y = 0; y < imgSize.y;  y++)
        for (int x = 0; x < imgSize.x;  x++)
        {
            unsigned int index = x + imgSize.x * y;
            int nUpdates = monoDepthEstimator->optimPyramid->nUpdates->GetData(MEMORYDEVICE_CPU)[index];
            float gtEst = dataPyramidLevel->depth->GetData(MEMORYDEVICE_CPU)[index];

            if (nUpdates < 140 || gtEst > 2.8)
                goodPoint[index] = false;
            else
                goodPoint[index] = true;
        }

    *goodData = goodPoint;


    

    // monoDepthEstimator->currDepthFrame->colorImageData->UpdateHostFromDevice();

    // std::cout << "Color data here: " << (int)monoDepthEstimator->currDepthFrame->colorImageData->GetData(MEMORYDEVICE_CPU)[0][0] << std::endl;

    width = dataPyramidLevel->depth->noDims[0];
    height = dataPyramidLevel->depth->noDims[1];
}


void MonoEngine::SampleFromBufferMid()
{
    // ORUChar4TSImage *rgbImage = imageBuffer[nMid];
    Sophus::SE3f kfPose = poseBuffer[nMid];
    AddKeyFrame(imageBuffer[nMid], kfPose);

    for (int i = 0; i < BUFFERSIZE; i++)
    {
        if (i == nMid)
            continue;
        // std::cout << "Sampling: " << i << std::endl;
        Sophus::SE3f trackingPose = poseBuffer[i];
        Sophus::SE3f inPose = trackingPose*invRefPose;
        cv::Mat inputRGBImage = imageBuffer[i];

        ConvertToOR(inputRGBImage, orImage);
        orImage->UpdateDeviceFromHost();
        monoDepthEstimator->UpdatePhotoError(SophusToOR(inPose), orImage);
    }


}

void MonoEngine::SaveToBuffer()
{
    cv::Mat lastImage = imageBuffer[BUFFERSIZE - 1];

    for (int i = BUFFERSIZE-1; i > 0; i--)
    {
        imageBuffer[i] = imageBuffer[i-1];
        poseBuffer[i] = poseBuffer[i-1];
        timeStampBuffer[i] = timeStampBuffer[i-1];
    }



    //No need to do this for the pose buffer
    imageBuffer[0] = lastImage;
    imageBuffer[0] = currImage.clone();

    poseBuffer[0] = currPose;
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
    // imOut = imResized;


    return imOut;
}


void MonoEngine::LoadGTDepth(long long timestamp)
{
    if (depthSource != NULL)
        depthSource->GetDepthForTimeStamp(timestamp);
    else
        std::cout << "No depth source" << std::endl;
    cv::Mat gtDepth = depthSource->Image();

    float minIDepth = monoDepthEstimator->optimPyramid->minIDepth;
    float maxIDepth = monoDepthEstimator->optimPyramid->maxIDepth;
    float iDepthDiff = maxIDepth - minIDepth;
    int depthSamples = monoDepthEstimator->optimPyramid->depthSamples; 
    float dIDepth =iDepthDiff / (depthSamples - 1);

    std::cout << "MinIDepth: " << minIDepth << std::endl;
    std::cout << "MaxIDepth: " << maxIDepth << std::endl;
    std::cout << "IDepthDiff: " << iDepthDiff << std::endl;
    std::cout << "dIDepth: " << dIDepth << std::endl;


    // float iDepth = 2.0f;
    float dData = 0.512;
    float z = dData * (depthSamples - 1);
    int z1 = floor(z);
    int z2 = ceil(z);
    float dz = z - z1;
    float weight1 = (1 - dz);
    float weight2 = dz;

    


    float iDepth1 = ((float)z1 * dIDepth) + minIDepth;
    float iDepth2 = ((float)z2 * dIDepth) + minIDepth;


    // std::cout << "IDepth orig: " << iDepth << std::endl;

    std::cout << "Z: " << z << std::endl;
    std::cout << "Z1: " << z1 << std::endl;
    std::cout << "Z2: " << z2 << std::endl;
    std::cout << "Weight1 :" << weight1 << std::endl;
    std::cout << "Weight2 :" << weight2 << std::endl;
    std::cout << "iDepth1: " << iDepth1 << std::endl;
    std::cout << "iDepth2: " << iDepth2 << std::endl;


    float minDepth = 1/maxIDepth;
    float maxDepth = 1/minIDepth;

    for (int y = 0; y < imgSize.y; y++)
        for (int x = 0; x < imgSize.x; x++)
        {
            unsigned int index = x + imgSize.x * y;
            float val = monoDepthEstimator->optimPyramid->d->GetData(MEMORYDEVICE_CPU)[index];
            val = gtDepth.at<float>(y,x);
            if (val >= minDepth && val <= maxDepth)
            {
                float inverseDepth = 1/val;
                float data = (inverseDepth - minDepth) / iDepthDiff;
                monoDepthEstimator->optimPyramid->d->GetData(MEMORYDEVICE_CPU)[index] = data;
            }
        }

    monoDepthEstimator->optimPyramid->d->UpdateDeviceFromHost();
    monoDepthEstimator->OptimToDepth(false);

}

void MonoEngine::VisualizeDepth()
{
    cv::Mat imOut = cv::Mat(imgSize.y, imgSize.x, CV_8UC1);
    monoDepthEstimator->optimPyramid->d->UpdateHostFromDevice();
    for (int y = 0; y < imgSize.y; y++)
        for (int x = 0; x < imgSize.x; x++)
        {
            unsigned int index = x + imgSize.x * y;
            float val = monoDepthEstimator->optimPyramid->d->GetData(MEMORYDEVICE_CPU)[index];
            unsigned char pix = val * 256;
            imOut.at<unsigned char>(y,x) = pix;
        }

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", imOut );                   // Show our image inside it.
    cv::waitKey(0);                                          // Wait for a keystroke in the window
    cv::destroyWindow("Display window");
}

void MonoEngine::MeasureDepthError()
{
    if (depthSource != NULL)
        depthSource->GetDepthForTimeStamp(timeStampRef);
    else
    {
        std::cout << "No depth source" << std::endl;
        return;
    }
    cv::Mat gtDepth = depthSource->Image();

    cv::Mat gtDepthResized;

    cv::Size outSize;
    outSize.width = imgSize.x;
    outSize.height = imgSize.y;
    cv::resize(gtDepth, gtDepthResized, outSize);

    MonoLib::MonoPyramidLevel *dataPyramidLevel =
        monoDepthEstimator->currDepthFrame->dataImage;

    dataPyramidLevel->depth->UpdateHostFromDevice();
    monoDepthEstimator->optimPyramid->nUpdates->UpdateHostFromDevice();


    float error = 0;
    int pixelCount = 0;
    for (int y = 0; y < imgSize.y; y++)
        for (int x = 0; x < imgSize.x; x++)
        {
            float gtTrue = gtDepthResized.at<float>(y,x);

            if (gtTrue <= 0)
                continue;

            unsigned int index = x + imgSize.x * y;
            goodPoint[index] = false;

            int nUpdates = monoDepthEstimator->optimPyramid->nUpdates->GetData(MEMORYDEVICE_CPU)[index];
            float gtEst = dataPyramidLevel->depth->GetData(MEMORYDEVICE_CPU)[index];

            if (nUpdates < 140 || gtEst > 2.8f || gtTrue > 2.8f)
                continue;

            float diff = gtTrue - gtEst;
            error += diff*diff;
            pixelCount++;

            goodPoint[index] = true;

        }

    error /= (float)pixelCount;
    error = sqrt(error);
    std::cout << "Depth error : " << error << std::endl;
    if (depthErrorFile.is_open())
        depthErrorFile << error << std::endl;

}

void MonoEngine::ProcessDepthData()
{
    MonoLib::MonoPyramidLevel *dataPyramidLevel =
        monoDepthEstimator->currDepthFrame->dataImage;

    monoDepthEstimator->optimPyramid->d->UpdateHostFromDevice();


    for (int y = 0; y < imgSize.y; y++)
        for (int x = 0; x < imgSize.x; x++)
        {
            unsigned int index = x + imgSize.x * y;
            float d = monoDepthEstimator->optimPyramid->d->GetData(MEMORYDEVICE_CPU)[index];
            cv::Vec3b out;
            out[0] = d*255;
            out[1] = d*255;
            out[2] = d*255;

            currDepthData->frame.at<cv::Vec3b>(y,x) = out;
        }

}

void MonoEngine::SmoothPhotoActive()
{
    monoDepthEstimator->InitOptim();
    theta = 0.2;
    processActive = true;
}

void MonoEngine::DoSmoothProcess()
{
    if (!processActive)
        return;

    for (unsigned int i = 0; i < 30; i++)
    {
        if (theta > thetaEnd)
        {
            theta = theta*(1-beta);
            monoDepthEstimator->RunTVOptimisationActive(theta);
        }
        else
        {
            processActive = false;
        }
    }

    ProcessDepthData();


}

