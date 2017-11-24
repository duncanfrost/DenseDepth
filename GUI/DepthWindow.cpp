#include "DepthWindow.h"
#include "glTools.h"
#include "VisualisationModule.h"

TextureSet DepthWindow::tempTextureImageWindow;
TextureSet DepthWindow::textureMask;



DepthWindow::DepthWindow(std::string title, int width, int height,
		   TrackerData *trackerData)
{
    this->title = title;
    this->trackerData = trackerData;
    this->width = width;
    this->height = height;
    interestLevel = 0;
}

void DepthWindow::Draw()
{
    cv::Mat imFrame=trackerData->frame;
    LoadTexture(imFrame,tempTextureImageWindow);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    DrawTextureBackground(tempTextureImageWindow);
    // DrawTextureBackground(textureMask);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glEnable(GL_POINT_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    set2DGLProjection();
    glPointSize(3.0);

    ClearConsole();

    glColor3f(1,1,1);
    unset2DGLProjection();


}

void DepthWindow::ProcessKeyboard(unsigned char key, int x, int y)
{
    switch (key) {
    case '1':
	interestLevel = 0;
	break;
    case '2':
	interestLevel = 1;
	break;
    case '3':
	interestLevel = 2;
	break;
    case '4':
	interestLevel = 3;
	break;
    case '5':
	interestLevel = 4;
	break;
    default:
	break;
    }

    Window::ProcessKeyboard(key,x,y);
}
