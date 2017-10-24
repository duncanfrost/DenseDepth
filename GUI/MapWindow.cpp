#include "MapWindow.h"
#include "glTools.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <cstring>

MapWindow::MapWindow(std::string title, int width, int height, MonoEngine *engine)
{
    this->title = title;
    this->width = width;
    this->height = height;
    this->map = engine->GetMap();
    this->trackerData = engine->GetARData();
    this->monoEngine = engine;

    useStereo = false;
    updateType = 0;
    massCentre = Eigen::Vector3f(0,0,0);
    mouseUp = true;

    refPoint_data = new DenseMono::Vector3f[1000*1000];
    color_data = new Vector4u[1000*1000];
    sigma2_data = new float[1000*1000];
    updateDenseMaps = true;

    SetInitCamPose();
}

void MapWindow::Draw()
{
    glClearColor(0,0,0,0);
    glClearDepth(1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    UpdateCameraFromInput();
    DrawMap();
    DrawOrigin();
}

void MapWindow::SetupFrustum()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    double zNear = 0.03;
    glFrustum(-zNear, zNear, 0.75*zNear,-0.75*zNear,zNear,500);
    glScalef(1,1,-1);
    return;
}

void MapWindow::SetupModelView(Sophus::SE3f se3WorldFromCurrent)
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrix(mse3ViewerFromWorld * se3WorldFromCurrent);
    return;
}

void MapWindow::DrawCamera(Sophus::SE3f se3CfromW, bool bSmall,
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

void MapWindow::DrawOrigin()
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


void MapWindow::UpdateCameraFromInput()
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




void MapWindow::DrawMap(void)
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

    for(unsigned int j=0; j<map->keyframeList.size(); j++)
    {
        KeyFrame *kf = map->keyframeList[j];
        DrawCamera(kf->pose,true,0,0,1);
    }


    DrawDenseMap();

    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


}

void MapWindow::ProcessMouseMotion(int x, int y)
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

void MapWindow::ProcessMouseClicks(int button,int state, int x, int y)
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

void MapWindow::ProcessKeyboard(unsigned char key, int x, int y)
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

void MapWindow::DrawDenseMap()
{
    SetupFrustum();
    SetupModelView();

    Vector3f *refPoints;
    Vector4u *colorData;

    if (updateDenseMaps)
    {
        monoEngine->GetPointCloud(imwidth,imheight,&refPoints, &colorData);
        std::memcpy(refPoint_data,refPoints,imwidth*imheight*sizeof(DenseMono::Vector3f));
        std::memcpy(color_data,colorData,imwidth*imheight*4*sizeof(unsigned char));
    }


    Sophus::SE3f invPose;
    invPose = MonoEngine::invRefPose;

    massCentre = Eigen::Vector3f(0,0,0);

    for (unsigned int y = 0; y < imheight; y++)
        for (unsigned int x = 0; x < imwidth; x++)
        {
            unsigned int index = x + imwidth*y;

            DenseMono::Vector3f pointRef = refPoint_data[index];
            Eigen::Vector3f pointRefE;
            pointRefE[0] = pointRef[0];
            pointRefE[1] = pointRef[1];
            pointRefE[2] = pointRef[2];

            Eigen::Vector3f pointWorld = invPose*pointRefE;
            Vector4u color = color_data[index];
            unsigned char c1 = color[0];
            unsigned char c2 = color[1];
            unsigned char c3 = color[2];

            if (std::isnan(pointRef[0]) || std::isnan(pointRef[1]) || std::isnan(pointRef[2]))
                continue;
            massCentre[0] += pointWorld[0];
            massCentre[1] += pointWorld[1];
            massCentre[2] += pointWorld[2];
            glColor3ub(c1,c2,c3);
                    glPointSize(1);
            glBegin(GL_POINTS);
            glVertex3f(pointWorld[0],pointWorld[1],pointWorld[2]);
            glEnd();
        }
    massCentre /= (float)(imheight*imwidth);
}
