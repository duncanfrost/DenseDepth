#include "MonoEngine.h"
#include <ORUtils/MathTypes.h>
#include "ActiveFunctions.h"
#include "ORConvert.h"

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
    featureHeight = 480;
    featureWidth = 640;
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
    map = new GlobalMap();



    //This is the target size
    imgSize.x = 640;
    imgSize.y = 480;

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
    depthMap = new rmd::Depthmap(imgSize.x, imgSize.y, fx, cx, fy, cy);
    orImage = new ORUChar4TSImage(imgSize, true, true, true);






    hasReferenceFrame = false;
    useRawDepth = true;
    needsKeyFrame = true;
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

    if (featureSource != NULL)
    {
        featureSource->GrabNewFrame();
    }

    cv::Mat rawImage = source->Image();
    currImage = PreProcessImage(rawImage);
    timeStamp = source->TimeStamp();

    std::cout << "Timestamp: " << timeStamp << std::endl;

    long long count = source->FrameNumber();
    long long timeOut;


    currPose = tracker->PoseAtTime(timeStamp, count, timeOut);

    Sample();
    // SaveToBuffer();

    if (needsKeyFrame)
    {
        AddKeyFrame(currImage, currPose);
        needsKeyFrame = false;
    }

    // if (SampleActive(count, BUFFERSIZE))
    // {
    //     SmoothPhotoBuffer(200);
    //     // SmoothPhotoRemode(200);
    // }

    std::cout << "Framenumber: " << count << std::endl;
        
    currTrackerData->trackerPose = currPose;
    currTrackerData->frame = currImage;

    framesProcessed++;
}


void MonoEngine::AddKeyFrame(cv::Mat inImage, Sophus::SE3f inPose)
{
    std::cout << "About to make new keyframe" << std::endl;

    KeyFrame *kf = new KeyFrame();
    kf->pose = inPose;
    map->keyframeList.push_back(kf);

    ConvertToOR(inImage, orImage);
    orImage->UpdateDeviceFromHost();
    if (featureSource != NULL)
        monoDepthEstimator->SetRefAndFeatureImage(orImage, featureImage);
    else
        monoDepthEstimator->SetRefImage(orImage);
    
    invRefPose = kf->pose.inverse();

    monoDepthEstimator->SetLimitsManual(0.5,2);
    hasReferenceFrame = true;
}

void MonoEngine::AddKeyFrame_Remode(cv::Mat inImage, Sophus::SE3f inPose)
{
    std::cout << "About to make new keyframe" << std::endl;

    KeyFrame *kf = new KeyFrame();
    kf->pose = inPose;
    map->keyframeList.push_back(kf);



    Eigen::Quaternionf q = inPose.unit_quaternion();
    Eigen::Vector3f t =  inPose.translation();
    rmd::SE3<float> rmdPose(q.w(), q.x(), q.y(), q.z(), t[0], t[1], t[2]);

    ConvertToOR(inImage, orImage);
    orImage->UpdateDeviceFromHost();
    monoDepthEstimator->SetRefImage(orImage);
    invRefPose = kf->pose.inverse();


    cv::Mat greyMat;
    cv::cvtColor(inImage, greyMat, CV_BGR2GRAY);

    depthMap->setReferenceImage(greyMat, rmdPose, 0.5f, 2.0f);


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
    if (featureSource != NULL)
        monoDepthEstimator->UpdatePhotoErrorWithFeatures(SophusToOR(inPose),
                                                         orImage, featureImage);
    else
        monoDepthEstimator->UpdatePhotoError(SophusToOR(inPose), orImage);
}

void MonoEngine::Sample2()
{
    //This is just to test interpolation
    std::cout << "Sample2" << std::endl;
    float *featureData = featureImage->GetData(MEMORYDEVICE_CPU);
    float outData[64];


    std::cout << "Data before" << std::endl;

    for (unsigned int i = 0; i < 64; i++)
        std::cout << outData[i] << " ";
    std::cout << std::endl;

    Vector2f pos(100.3, 100.8);
    InterpolateFeature(featureData, outData, pos, featureWidth, featureChannels);

    int c = 31;

    int ix = (int)pos.x;
    int iy = (int)pos.y;

    float dataTL = featureData[(featureChannels*(ix + featureWidth*iy)) + c];
    float dataTR = featureData[(featureChannels*(ix + 1 + featureWidth*iy)) + c];
    float dataBL = featureData[(featureChannels*(ix + featureWidth*(iy + 1))) + c];
    float dataBR = featureData[(featureChannels*(ix + 1 + featureWidth*(iy + 1))) + c];

    float interp = outData[c];

    float dx = pos.x - (int)pos.x;
    float dy = pos.y - (int)pos.y;
    float weight_tl = (1.0f - dx) * (1.0f - dy);
    float weight_tr = (dx)        * (1.0f - dy);
    float weight_bl = (1.0f - dx) * (dy);
    float weight_br = (dx)        * (dy);


    float sum = dataTL*weight_tl + dataTR*weight_tr + dataBL*weight_bl +
        dataBR*weight_br;
    std::cout << "interp1: " << sum << std::endl;
    std::cout << "interp: " << interp << std::endl;
    // exit(1);



}

void MonoEngine::SmoothPhoto(int iterations)
{
    monoDepthEstimator->RunTVOptimisation(iterations);
    VisualizeDepth();
    paused = true;
    // monoDepthEstimator->RunTVL1Optimisation(iterations);
    // monoDepthEstimator->RunTVL0Optimisation(iterations);
}

void MonoEngine::SmoothPhotoBuffer(int iterations)
{
    SampleFromBufferMid();


    SmoothPhoto(iterations);


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

    VisualizeDepth();
    paused = true;
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

void MonoEngine::SampleFromBufferMid_Remode()
{
    // ORUChar4TSImage *rgbImage = imageBuffer[nMid];
    Sophus::SE3f kfPose = poseBuffer[nMid];
    AddKeyFrame_Remode(imageBuffer[nMid], kfPose);

    for (int i = 0; i < BUFFERSIZE; i++)
    {
        if (i == nMid)
            continue;

        Sophus::SE3f trackingPose = poseBuffer[i];
        cv::Mat inputRGBImage = imageBuffer[i];

        Eigen::Quaternionf q = trackingPose.unit_quaternion();
        Eigen::Vector3f t =  trackingPose.translation();
        rmd::SE3<float> rmdPose(q.w(), q.x(), q.y(), q.z(), t[0], t[1], t[2]);

        std::cout << "Updating depth map: " << i << std::endl;

        cv::Mat greyMat;
        cv::cvtColor(inputRGBImage, greyMat, CV_BGR2GRAY);
        depthMap->update(greyMat, rmdPose);
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

void MonoEngine::SmoothPhotoRemode(int iterations)
{
    SampleFromBufferMid_Remode();
    // SmoothPhoto(iterations);


    depthMap->downloadDenoisedDepthmap(0.6, 3000);
    // depthMap->downloadDepthmap();
    cv::Mat im = depthMap->getDepthmap();






    cv::Mat imOut = cv::Mat(imgSize.y, imgSize.x, CV_8UC1);

    for (int y = 0; y < imgSize.y; y++)
        for (int x = 0; x < imgSize.x; x++)
        {
            unsigned int index = x + imgSize.x * y;
            float val = im.at<float>(y,x);
            unsigned char pix = val * 256;
            imOut.at<unsigned char>(y,x) = pix;
            monoDepthEstimator->currDepthFrame->dataImage->depth->GetData(MEMORYDEVICE_CPU)[index] = val;
        }

    monoDepthEstimator->currDepthFrame->dataImage->depth->UpdateDeviceFromHost();

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", imOut );                   // Show our image inside it.
    cv::waitKey(0);                                          // Wait for a keystroke in the window

    // paused = true;

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
