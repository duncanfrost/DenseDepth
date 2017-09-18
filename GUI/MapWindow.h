//==============================================================================
//MapWindow.h: Window to display the world map
//==============================================================================
#pragma once

#include <vector>

#include "Window.h"

#include "../MonoLib/Shared/DenseMonoMath.h"

class MapWindow:public Window
{
public:
    MapWindow(std::string title, int width, int height);
    MapWindow(std::string title, int width, int height);

    void Draw();
    void setEvents();

    void ProcessMouseMotion(int x, int y);
    void ProcessMouseClicks(int button, int state, int x, int y);
    void ProcessKeyboard(unsigned char key, int x, int y);

protected:

    void SetInitCamPose()
    {
        poseUpdateMouseCam= DenseMono::SE3f::Tangent::Zero();
        poseUpdateMouseWorld= DenseMono::SE3f::Tangent::Zero();
        poseUpdateKeyboard= DenseMono::SE3f::Tangent::Zero();
        // Set the initial camera pose;
        DenseMono::SE3f::Tangent v1 = DenseMono::SE3f::Tangent::Zero();
        v1[2] = 3.0f;
        DenseMono::SE3f::Tangent v2 = DenseMono::SE3f::Tangent::Zero();
        v2[3] = (float)(M_PI_4);

        mse3ViewerFromWorldOR = DenseMono::SE3f::exp(v1) * DenseMono::SE3f::exp(v2);
    }
    GlobalMap *map;
    DenseMono::SE3f mse3ViewerFromWorldOR;

    DenseMono::Vector2f lastMousePosition;
    DenseMono::Vector3f massCentre;
    unsigned int updateType;
    bool mouseUp;
    bool onlyGoodPoints;
    TrackerData *trackerData;

    ITMLib::MonoEngine<ITMVoxel> *monoEngine;
    ITMLib::StereoEngine<ITMVoxel> *stereoEngine;
    bool useStereo;

    float *sigma2_data;
    Vector4u *color_data;
    DenseMono::Vector3f *refPoint_data;

    DenseMono::SE3f::Tangent poseUpdateMouseWorld; //Rotates the world
    DenseMono::SE3f::Tangent poseUpdateKeyboard; //Moves cam in fps style
    DenseMono::SE3f::Tangent poseUpdateMouseCam; //Move cam using mouse

    void SetupFrustum();
    void SetupModelView(DenseMono::SE3f se3WorldFromCurrent = DenseMono::SE3f());
    void DrawCamera(DenseMono::SE3f se3CfromW, bool bSmall,
                    float r, float g, float b);
    void DrawOrigin();
    void UpdateCameraFromInput();


    void DrawMap(void);
    void DrawDenseMap();
    unsigned int interestLevel;
    bool updateDenseMaps;
    unsigned int imwidth, imheight;
};
