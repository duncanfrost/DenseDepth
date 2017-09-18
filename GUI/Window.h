//==============================================================================
//Window.h: Basic abstract window class.
//==============================================================================

#pragma once

#include <string>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "Button.h"
#include <vector>
#include <sstream>

#define BUTTON_WIDTH 90
#define BUTTON_HEIGHT 40
#define BUTTON_MARGIN 10

class VisualisationModule;

class Window
{
public:
    Window(){
        shouldDraw = true;
        shouldDrawConsole = true;
    }
    ~Window(){}

    void CreateAppWindow();
    virtual void SetCallbacks(void);
    virtual void Draw()=0;
    void DrawObjects();

    //Accessors
    int WindowNumber(){return windowNumber;}
    int Width(){return width;}
    int Height(){return height;}

    void SetKeyboardFunction(void (*keyboardFunction)(unsigned char key,
                                                       int x,
                                                       int y)) {
        this->keyboardFunction = keyboardFunction;
    }

    virtual void ProcessMouseMotion(int x, int y);
    virtual void ProcessMouseClicks(int button, int state, int x, int y);
    virtual void ProcessKeyboard(unsigned char key, int x, int y);
    void ResizeWindow(int width, int height);

    //Static callbacks
    static void StaticProcessMouseClick(int button, int state, int x, int y);
    static void StaticProcessMouseMotion(int x, int y);
    static void StaticProcessKeyboard(unsigned char key, int x, int y);
    static void StaticDisplay(void);
    static void StaticResizeWindow(int width, int height);

    static void SetVisuModule(VisualisationModule* visuModule) {
        Window::visuModule = visuModule;
    }

    static VisualisationModule* VisuModule(void) {
        return visuModule;
    }

    void DrawConsole();
    void ClearConsole();


    bool shouldDraw;
protected:
    bool shouldDrawConsole;
    static VisualisationModule* visuModule;
    std::vector<Button> buttonList;
    std::string title;
    std::stringstream console;
    int windowNumber;
    int width,height;
    void (*drawFunction)() = NULL;
    void (*keyboardFunction)(unsigned char key, int x, int y) = NULL;
};
