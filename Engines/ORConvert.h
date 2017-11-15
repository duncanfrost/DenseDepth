#include <ORUtils/MathTypes.h>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

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

inline void ORToCVConvert(ORUtils::Image<float> *imageIn, cv::Mat in, float mult)
{
    for (int y = 0; y < imageIn->noDims.y; y++)
        for (int x = 0; x < imageIn->noDims.x; x++)
        {
            int index = x + imageIn->noDims.x*y;
            in.at<short>(y,x) = mult*(imageIn->GetData(MEMORYDEVICE_CPU)[index] + 10);
        }
}

inline void ORToCVConvertUpdates(ORUtils::Image<float> *imageIn,
                                 ORUtils::MemoryBlock<int> *updates,
                                 cv::Mat in)
{
    for (int y = 0; y < imageIn->noDims.y; y++)
        for (int x = 0; x < imageIn->noDims.x; x++)
        {
            int index = x + imageIn->noDims.x*y;
            int noUpdates = updates->GetData(MEMORYDEVICE_CPU)[index];

            short val = 0; 
            if (noUpdates > 0.5*BUFFERSIZE)
                val = 5000*imageIn->GetData(MEMORYDEVICE_CPU)[index];

            in.at<short>(y,x) = val; 
        }
}


inline void ConvertToOR(cv::Mat inImage, ORUChar4TSImage *outImage)
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

inline void ConvertToOR(cv::Mat inImage, ORUtils::MemoryBlock<float> *outImage)
{
    for (int y = 0; y < inImage.rows; y++)
    {
        for (int x = 0; x < inImage.cols; x++)
        {
            cv::Vec3b val = inImage.at<cv::Vec3b>(y,x);
            // Vector4u orVal;
            // orVal[0] = val[2];
            // orVal[1] = val[1];
            // orVal[2] = val[0];
            // orVal[3] = 0;

            int index = x + inImage.cols*y;
            outImage->GetData(MEMORYDEVICE_CPU)[index] = val[0];
        }
    }

    outImage->UpdateDeviceFromHost();
}
