#include "VisualisationModule.h"
#include <iostream>

VisualisationModule::VisualisationModule(void (*idleFunction)())
{
    this->idleFunction=idleFunction;
    this->useAutoPosition=true;
    Window::SetVisuModule(this);
}

void VisualisationModule::PrepareLoop()
{
    int argc = 0;
    char** argv;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutIdleFunc(*idleFunction);

    for (std::vector<Window*>::iterator it = windowList.begin();
         it != windowList.end(); it++)
    {
        (*it)->CreateAppWindow();
        (*it)->SetCallbacks();
    }

    if(useAutoPosition)
    {

        int screenWidth1 = glutGet(GLUT_SCREEN_WIDTH)/2;
        int screenWidth = glutGet(GLUT_SCREEN_WIDTH);
        int x_current=0,y_current=0;
        int prevHeight = 0;
        for (std::vector<Window*>::iterator it = windowList.begin();
             it != windowList.end(); it++)
        {
            if(x_current+(*it)->Width()>screenWidth1)
            {
                x_current=0;
                y_current+= prevHeight + 10;
            }

            glutSetWindow((*it)->WindowNumber());
            glutPositionWindow(x_current,y_current);
            x_current+=(*it)->Width();
            x_current+=10;
            prevHeight = (*it)->Height();
        }
    }

}

void VisualisationModule::StartLoop(void)
{
    PrepareLoop();
    glutMainLoop();
}

void VisualisationModule::DrawWindows()
{
    for (std::vector<Window*>::iterator it = windowList.begin();
         it != windowList.end(); it++)
    {
        Window* window = *it;
        if (!window->shouldDraw)
            continue;
        glutSetWindow(window->WindowNumber());
        window->Draw();
        window->DrawObjects();
        window->DrawConsole();
        glutSwapBuffers();
        glutPostRedisplay();
    }
}

VisualisationModule::~VisualisationModule()
{
    for (std::vector<Window*>::iterator it = windowList.begin();
         it != windowList.end(); it++)
        glutDestroyWindow((*it)->WindowNumber());
}
