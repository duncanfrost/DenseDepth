#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <MonoLib/MonoDepthEstimator_CUDA.h>
#include <opencv2/imgproc/imgproc.hpp>

int main(void)
{
    cv::Mat imInColor = cv::imread("noisyLena.png");


    cv::Mat imIn;
    cv::cvtColor(imInColor, imIn, cv::COLOR_BGR2GRAY);

    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", imIn );                   // Show our image inside it.
    cv::waitKey(0);                                          // Wait for a keystroke in the window

    std::cout<< imIn.channels() << std::endl;

    MonoLib::MonoDepthEstimator_CUDA *monoDepthEstimator;

    Vector2i imgSize;
    imgSize.x = imIn.cols;
    imgSize.y = imIn.rows;

    Vector4f intrinsics;
    intrinsics[0] = 100;
    intrinsics[1] = 100;
    intrinsics[2] = 100;
    intrinsics[3] = 100;

    std::cout << "Image size: " << imgSize << std::endl;
    monoDepthEstimator = new MonoLib::MonoDepthEstimator_CUDA(imgSize, intrinsics);

    // for (int y = 0; y < 10; y++)
    // {
    //     for (int x = 0; x < 10; x++)
    //     {
    //         unsigned char pix = imIn.at<unsigned char>(y,x);
    //         std::cout << (int)pix << " ";
    //     }
    //     std::cout << std::endl;
    // }

    std::cout<< "Size: " << monoDepthEstimator->optimPyramid->d->noDims << std::endl;

    for (int y = 0; y < imgSize.y; y++)
        for (int x = 0; x < imgSize.x; x++)
        {
            unsigned char pix = imIn.at<unsigned char>(y,x);

            unsigned int index = x + imgSize.x * y;
            float val = (float)pix / 256;

            // std::cout << (int)pix << "->" << val << std::endl;
            monoDepthEstimator->optimPyramid->d->GetData(MEMORYDEVICE_CPU)[index] = val;
            monoDepthEstimator->optimPyramid->a->GetData(MEMORYDEVICE_CPU)[index] = val;
        }

    monoDepthEstimator->optimPyramid->a->UpdateDeviceFromHost();
    monoDepthEstimator->optimPyramid->d->UpdateDeviceFromHost();
    monoDepthEstimator->SmoothL1();

    monoDepthEstimator->optimPyramid->d->UpdateHostFromDevice();

    cv::Mat imOut = imIn.clone();
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


}
