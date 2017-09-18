//==============================================================================
//VisualisatoinModule.h: Manages and draws a collection of gui windows.
//==============================================================================

#pragma once

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "Window.h"
#include <list>
#include <vector>

class VisualisationModule
{
 public:

    VisualisationModule(void (*idleFunction)());

    //Accessors
    void AddWindow(Window* window) {
        windowList.push_back(window);
    }

    Window* GetWindow(int nWindow) {
        return windowList[nWindow];
    }

    void StartLoop(void);
    void DrawWindows();



    void SetKeyboardFunction(void (*keyboardFunction)(unsigned char key,
                                                       int x,
                                                       int y)) {

        for (unsigned int i = 0; i < windowList.size(); i++)
            windowList[i]->SetKeyboardFunction(keyboardFunction);
    }

    ~VisualisationModule();

 protected:
    void PrepareLoop();
    std::vector<Window*> windowList;
    void (*idleFunction)();
    bool useAutoPosition;
};

