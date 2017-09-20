#include "FusionWindow.h"
#include "glTools.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <cstring>

FusionWindow::FusionWindow(std::string title, int width, int height, FusionEngine *engine)
{
    this->title = title;
    this->width = width;
    this->height = height;
    this->map = engine->GetMap();
    this->trackerData = engine->GetARData();
    this->fusionEngine = engine;

    useStereo = false;
    updateType = 0;
    massCentre = Eigen::Vector3f(0,0,0);
    mouseUp = true;

    updateDenseMaps = true;

    SetInitCamPose();
}

void FusionWindow::Draw()
{
    glClearColor(0,0,0,0);
    glClearDepth(1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    UpdateCameraFromInput();
    DrawMap();
    DrawOrigin();
}

void FusionWindow::SetupFrustum()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double zNear = 0.03;
    glFrustum(-zNear, zNear, 0.75*zNear,-0.75*zNear,zNear,500);
    glScalef(1,1,-1);
    return;
}

void FusionWindow::SetupModelView(Sophus::SE3f se3WorldFromCurrent)
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrix(mse3ViewerFromWorld * se3WorldFromCurrent);
    return;
}

void FusionWindow::DrawCamera(Sophus::SE3f se3CfromW, bool bSmall,
                           float r, float g, float b)
{
    SetupModelView(se3CfromW.inverse());
    SetupFrustum();

    if(bSmall)
        glLineWidth(1);
    else
        glLineWidth(3);

    glBegin(GL_LINES);
    glColor3f(r,g,b);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.1f, 0.0f, 0.0f);
    glColor3f(r,g,b);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.1f, 0.0f);
    glColor3f(1,1,1);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.1f);
    glEnd();
}

void FusionWindow::DrawOrigin()
{
    Sophus::SE3f origin;
    SetupModelView(origin);
    SetupFrustum();

    glLineWidth(1);

    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.1f, 0.0f, 0.0f);
    glColor3f(0,1,0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.1f, 0.0f);
    glColor3f(0,0,1);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.1f);
    glEnd();
}


void FusionWindow::UpdateCameraFromInput()
{
    Sophus::SE3f se3CamFromMC;
    se3CamFromMC.translation() = mse3ViewerFromWorld * massCentre;
    mse3ViewerFromWorld = Sophus::SE3f::exp(poseUpdateMouseCam)  *
        se3CamFromMC * Sophus::SE3f::exp(poseUpdateMouseWorld) *
        se3CamFromMC.inverse() * mse3ViewerFromWorld;

    poseUpdateMouseWorld.setZero();
    poseUpdateMouseCam.setZero();

    mse3ViewerFromWorld = Sophus::SE3f::exp(poseUpdateKeyboard)
        * mse3ViewerFromWorld;
    poseUpdateKeyboard.setZero();
}




void FusionWindow::DrawMap(void)
{

    //Draw
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    //    glEnable(GL_BLEND);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColorMask(1,1,1,1);
    glEnable(GL_DEPTH_TEST);



    DrawCamera(trackerData->trackerPose,false,1,0,0);


    DrawDenseMap();

    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


}

void FusionWindow::ProcessMouseMotion(int x, int y)
{
    if (updateType)
    {
        Eigen::Vector2f position((float)x, (float)y);
        Eigen::Vector2f motion = position -  lastMousePosition;
        lastMousePosition = position;
        if (mouseUp)
        {
            mouseUp=false;
            return;
        }

        float dSensitivity = 0.01f;

        switch(updateType)
        {
        case 1:
            poseUpdateMouseWorld[3] -= motion[1] * dSensitivity;
            poseUpdateMouseWorld[4] += motion[0] * dSensitivity;
            break;

        case 2:
            poseUpdateMouseCam[4] -= motion[0] * dSensitivity;
            poseUpdateMouseCam[3] += motion[1] * dSensitivity;
            break;

        case 3:
            poseUpdateMouseCam[5] -= motion[0] * dSensitivity;
            poseUpdateMouseCam[2] += motion[1] * dSensitivity;
            break;
        }
    }
    Window::ProcessMouseMotion(x,y);
}

void FusionWindow::ProcessMouseClicks(int button,int state, int x, int y)
{
    if(state == GLUT_DOWN && button == GLUT_LEFT_BUTTON)
        updateType = 1;

    if(state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON)
        updateType = 2;

    if(state == GLUT_DOWN && button == GLUT_MIDDLE_BUTTON)
        updateType = 3;

    if(state == GLUT_UP)
    {
        updateType = 0;
        mouseUp = true;
    }
    Window::ProcessMouseClicks(button,state,x,y);
}

void FusionWindow::ProcessKeyboard(unsigned char key, int x, int y)
{
    if (key == 'w')
        poseUpdateKeyboard[2] = -0.1f;

    if (key == 'a')
        poseUpdateKeyboard[0] = 0.1f;

    if (key == 's')
        poseUpdateKeyboard[2] = 0.1f;

    if (key == 'd')
        poseUpdateKeyboard[0] = -0.1f;

    if (key == 't')
        updateDenseMaps = !updateDenseMaps;

    Window::ProcessKeyboard(key,x,y);
}

void FusionWindow::DrawDenseMap()
{
    SetupFrustum();
    SetupModelView();


    std::vector<MapPoint*> mappoints = fusionEngine->GetMap()->mappoints;

    massCentre = Eigen::Vector3f(0,0,0);

    for (unsigned int i = 0; i < mappoints.size(); i++)
        {

            MapPoint *mp = mappoints[i];
            Eigen::Vector3f position = mp->position;

            if (std::isnan(position[0]) || std::isnan(position[1]) || std::isnan(position[2]))
                continue;
            // massCentre[0] += position[0];
            // massCentre[1] += position[1];
            // massCentre[2] += position[2];
            glColor3ub(mp->c1,mp->c2,mp->c3);
            glPointSize(1);
            glBegin(GL_POINTS);
            glVertex3f(position[0],position[1],position[2]);
            glEnd();
        }
}
