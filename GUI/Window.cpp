#include "Window.h"
#include "VisualisationModule.h"
#include "glTools.h"
#include <iostream>

VisualisationModule* Window::visuModule = NULL;

//These three functions are static as we cannot pass non-static member functions
//to glut.
void Window::StaticProcessMouseClick(int button, int state, int x, int y)
{
    VisualisationModule *vs = Window::VisuModule();
    int nGlutWindow=glutGetWindow();
    Window* window = vs->GetWindow(nGlutWindow-1);
    window->ProcessMouseClicks(button,state,x,y);
}

void Window::StaticDisplay()
{
}

void Window::StaticProcessMouseMotion(int x, int y)
{
    VisualisationModule *vs = Window::VisuModule();
    int nGlutWindow=glutGetWindow();
    Window* window = vs->GetWindow(nGlutWindow-1);
    window->ProcessMouseMotion(x,y);
}
void Window::StaticProcessKeyboard(unsigned char key, int x, int y)
{
    VisualisationModule *vs = Window::VisuModule();
    int nGlutWindow=glutGetWindow();
    Window* window = vs->GetWindow(nGlutWindow-1);
    window->ProcessKeyboard(key,x,y);
}

void Window::StaticResizeWindow(int width, int height)
{
    VisualisationModule *vs = Window::VisuModule();
    int nGlutWindow=glutGetWindow();
    Window* window = vs->GetWindow(nGlutWindow-1);
    window->ResizeWindow(width,height);

}

void Window::SetCallbacks(void)
{
    glutKeyboardFunc(Window::StaticProcessKeyboard);
    glutMouseFunc(Window::StaticProcessMouseClick);
    glutMotionFunc(Window::StaticProcessMouseMotion);
    glutReshapeFunc(Window::StaticResizeWindow);
    glutDisplayFunc(Window::StaticDisplay);
}


void Window::DrawConsole()
{
    if (shouldDrawConsole)
    {
        std::stringstream out;
        out << " " << console.str();
        drawText(10, 10, out.str().c_str(), 0,1,1, GLUT_BITMAP_HELVETICA_18);
    }
}

void Window::ClearConsole()
{
    console.str(std::string());
    console.clear();
}

void Window::DrawObjects()
{
    //Go through all our buttons and call their draw functions
    for (std::vector<Button>::iterator it = buttonList.begin();
         it != buttonList.end(); it++)
        (*it).Draw();
}

void Window::ProcessMouseClicks(int button, int state, int x, int y)
{
    //Go through all our buttons and call their check for click functions
    for (std::vector<Button>::iterator it = buttonList.begin();
         it != buttonList.end(); it++)
        (*it).CheckClick(state,x,y);
}

void Window::ProcessMouseMotion(int x, int y)
{
    //Go through all our buttons and call their check for motion functions
    for (std::vector<Button>::iterator it = buttonList.begin();
         it != buttonList.end(); it++)
        (*it).CheckMotion(x,y);
}
void Window::ProcessKeyboard(unsigned char key, int x, int y)
{
    if (keyboardFunction)
        (keyboardFunction)(key,x,y);
}


void Window::ResizeWindow(int width, int height)
{
	this->width = width;
	this->height = height;
	glViewport(0,0,width,height);
	glutInitWindowSize(width, height);
}

void Window::CreateAppWindow()
{
    glutInitWindowSize(width, height);
    windowNumber = glutCreateWindow(title.c_str());
}
