//==============================================================================
//ARWindow.h: Window to display the map as viewed from the current tracking pose
//==============================================================================

#pragma once

#include "Window.h"
#include "glTexture.h"
#include "../MonoLib/Shared/DenseMonoMath.h"
#include "../MonoLib/Shared/TrackerData.h"


class ARWindow:public Window
{
public:
    ARWindow(std::string title, int width, int height, TrackerData *trackerData);
    void ProcessKeyboard(unsigned char key, int x, int y);
    void Draw();

protected:
    unsigned int level;
    TrackerData *trackerData;
    static TextureSet tempTextureImageWindow;
    static TextureSet textureMask;
    unsigned int interestLevel;
};
