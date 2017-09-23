//==============================================================================
//FusionWindow.h: Window to display the world map
//==============================================================================
#pragma once

#include <vector>

#include "Window.h"

#include <MonoLib/Shared/Map.h>
#include <MonoLib/Shared/TrackerData.h>
#include <Engines/FusionEngine.h>

class FusionWindow:public Window
{
public:
    FusionWindow(std::string title, int width, int height, FusionEngine *engine);

    void Draw();
    void setEvents();

    void ProcessMouseMotion(int x, int y);
    void ProcessMouseClicks(int button, int state, int x, int y);
    void ProcessKeyboard(unsigned char key, int x, int y);

protected:

    void SetInitCamPose()
    {
        poseUpdateMouseCam= Sophus::SE3f::Tangent::Zero();
        poseUpdateMouseWorld= Sophus::SE3f::Tangent::Zero();
        poseUpdateKeyboard= Sophus::SE3f::Tangent::Zero();
        // Set the initial camera pose;
        Sophus::SE3f::Tangent v1 = Sophus::SE3f::Tangent::Zero();
        v1[2] = 3.0f;
        Sophus::SE3f::Tangent v2 = Sophus::SE3f::Tangent::Zero();
        v2[3] = (float)(M_PI_4);

        mse3ViewerFromWorld = Sophus::SE3f::exp(v1) * Sophus::SE3f::exp(v2);
    }
    GlobalMap *map;
    Sophus::SE3f mse3ViewerFromWorld;

    Eigen::Vector2f lastMousePosition;
    Eigen::Vector3f massCentre;
    unsigned int updateType;
    bool mouseUp;
    bool onlyGoodPoints;
    TrackerData *trackerData;

    FusionEngine *fusionEngine;
    bool useStereo;

    float *sigma2_data;
    Vector4u *color_data;
    DenseMono::Vector3f *refPoint_data;

    Sophus::SE3f::Tangent poseUpdateMouseWorld; //Rotates the world
    Sophus::SE3f::Tangent poseUpdateKeyboard; //Moves cam in fps style
    Sophus::SE3f::Tangent poseUpdateMouseCam; //Move cam using mouse

    void SetupFrustum();
    void SetupModelView(Sophus::SE3f se3WorldFromCurrent = Sophus::SE3f());
    void DrawCamera(Sophus::SE3f se3CfromW, bool bSmall,
                    float r, float g, float b);
    void DrawOrigin();
    void UpdateCameraFromInput();


    void DrawMap(void);
    void DrawDenseMap();
    unsigned int interestLevel;
    bool freeCam;
    unsigned int imwidth, imheight;
};
