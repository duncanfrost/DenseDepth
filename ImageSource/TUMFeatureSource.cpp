#include "TUMFeatureSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>

TUMFeatureSource::TUMFeatureSource(const std::string& listFile)
{
    nChannels = 64;
    height = 480;
    width = 640;
    data = NULL;
    PathsFromListFile(rgbImagePaths, rgbTimeStamps, listFile);
}

void TUMFeatureSource::GrabNewFrame()
{

    if (data == NULL)
        data = new float[nChannels * height * width];

    std::string path = rgbImagePaths[frameNumber];
    timeStamp = rgbTimeStamps[frameNumber];

    FILE * pFile;
    pFile = fopen (path.c_str(), "rb");
    fread (data, sizeof(float), nChannels * height * width, pFile);
    fclose(pFile);

    if (frameNumber < rgbImagePaths.size()-1)
        frameNumber++;
}

void TUMFeatureSource::GrabNewFrameDebug()
{

    if (data == NULL)
        data = new float[nChannels * height * width];

    std::string path = rgbImagePaths[frameNumber];
    timeStamp = rgbTimeStamps[frameNumber];

    FILE * pFile;
    pFile = fopen (path.c_str(), "rb");
    fread (data, sizeof(float), nChannels * height * width, pFile);
    fclose(pFile);


    for (int c = 0; c < 10; c++)
    {
        cv::Mat imOut = cv::Mat(height, width, CV_8UC1);
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
            {
                // float data = channel.at<float>(y,x);
                int index = c + nChannels*x + nChannels*width*y; 
                float data_pix = data[index];
                data_pix = data_pix + 160;
                imOut.at<unsigned char>(y,x) = data_pix;
            }

        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow( "Display window", imOut );                   // Show our image inside it.
        cv::waitKey(0);                                          // Wait for a keystroke in the window
        cv::destroyWindow("Display window");
    }


    if (frameNumber < rgbImagePaths.size()-1)
        frameNumber++;
}


void TUMFeatureSource::PathsFromListFile(std::vector<std::string> &imagePaths,
                                  std::vector<long long> &timeStamps,
                                  std::string listPath)
{
    imagePaths.clear();
    std::ifstream listFile(listPath.c_str());
    if (!listFile)
    {
        std::cout <<  "error: rgb image list file doesn't exist" << std::endl;
        exit(1);
    }
    std::string line;

    //This is the directory that the list file is located in.
    std::string listDir = listPath.substr(0,listPath.find_last_of("/")+1);
    std::cout << "listDir" << std::endl;
    std::cout << listDir << std::endl;

    while (std::getline(listFile, line))
    {
        //Hashtags are considered comments
        if (line[0] == '#')
            continue;

        double timestampRaw;
        std::string path;

        std::istringstream iss(line);

        //Full paths are the root directory joined with whatever is in the listfile
        if (iss >> timestampRaw)
        {

            long long timestamp = timestampRaw * 1e6;

            std::string imagePath;
            iss >> imagePath;

            
            std::stringstream fullPath;
            fullPath << listDir << imagePath;

            imagePaths.push_back(fullPath.str());
            timeStamps.push_back(timestamp);
        }
    }
}
