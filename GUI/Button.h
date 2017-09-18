//==============================================================================
//Button.h: Button for windows
//==============================================================================

#pragma once

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <string>

class Button
{
 public:
    Button(std::string caption,
           void (*clickFunction)(), int x, int y, int w, int h);
    bool CheckClick(int state, int x, int y);
    void CheckMotion(int x,int y);

    void Draw();

 protected:
    void (*clickFunction)();

    std::string caption;
    int x; int y;
    int w; int h;

    bool clicked;
    bool mouseIn;
};
