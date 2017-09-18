#include <iostream>
#include <GUI/VisualisationModule.h>
#include <GUI/ARWindow.h>
#include <Engines/MonoEngine.h>
#include <ImageSource/PhoneSource.h>

void Idle(void);
void KeyboardFunction(unsigned char key, int x, int y);
VisualisationModule *visModule;
MonoEngine *engine;
PhoneSource *source;

int main(void)
{
    std::string filename1 = "/home/duncan/Data/P9/SidewaysLong/log.txt";
    std::string gtFile = "/home/duncan/Data/P9/SidewaysLong/CameraTrajectory.txt";

    source = new PhoneSource(filename1);
    engine = new MonoEngine(source);

    visModule = new VisualisationModule(&Idle);
    visModule->AddWindow(new ARWindow("AR",640,480,engine->GetARData()));

    visModule->SetKeyboardFunction(&KeyboardFunction);
    visModule->StartLoop();
}

void Idle(void)
{
    visModule->DrawWindows();
    engine->Process();
}

void KeyboardFunction(unsigned char key, int x, int y)
{
    switch(key) {
    case 27://esc
        exit(0);
        break;
    }
}
