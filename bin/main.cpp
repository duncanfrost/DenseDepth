#include <iostream>
#include <GUI/VisualisationModule.h>
#include <GUI/ARWindow.h>
#include <GUI/MapWindow.h>
#include <Engines/MonoEngine.h>
#include <Engines/FileTracker.h>
#include <ImageSource/PhoneSource.h>

void Idle(void);
void KeyboardFunction(unsigned char key, int x, int y);
VisualisationModule *visModule;
MonoEngine *engine;
PhoneSource *source;
FileTracker *tracker;
bool paused = false;

int main(void)
{
    std::string filename1 = "/home/duncan/Data/P9/SidewaysLong/log.txt";
    std::string gtFile = "/home/duncan/Data/P9/SidewaysLong/CameraTrajectory.txt";

    source = new PhoneSource(filename1);
    tracker = new FileTracker(gtFile, FileTracker::ORB);
    engine = new MonoEngine(source, tracker);

    visModule = new VisualisationModule(&Idle);
    visModule->AddWindow(new ARWindow("AR",640,480,engine->GetARData()));
    visModule->AddWindow(new MapWindow("Map",640,480,engine));

    visModule->SetKeyboardFunction(&KeyboardFunction);
    visModule->StartLoop();
}

void Idle(void)
{
    if (!paused)
        engine->Process();
    visModule->DrawWindows();
}

void KeyboardFunction(unsigned char key, int x, int y)
{
    switch(key) {
    case 27://esc
        exit(0);
        break;

    case ' ':
        engine->SampleFromBufferMid();
        break;

    case 'p':
        paused = !paused;
        break;
        
    case 'o':
        engine->SmoothPhotoBuffer(200);
        break;

    }
}
