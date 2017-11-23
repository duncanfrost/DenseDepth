#include <iostream>
#include <GUI/VisualisationModule.h>
#include <GUI/ARWindow.h>
#include <GUI/MapWindow.h>
#include <Engines/MonoEngine.h>
#include <Tracking/ORBFileTracker.h>
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
    std::string filename1 = "/home/duncan/Data/P9/Office3/log.txt";
    std::string gtFile = "/home/duncan/Data/P9/Office3/CameraTrajectory.txt";

    source = new PhoneSource(filename1);
    tracker = new ORBFileTracker(gtFile);


    MonoEngine::Settings settings;
    settings.fx = 683.8249;
    settings.fy = 683.6179;
    settings.cx = 317.6438;
    settings.cy = 239.5907;

    settings.inputSizeX = 640;
    settings.inputSizeY = 480;
    settings.targetSizeX = 640;
    settings.targetSizeY = 480;


    engine = new MonoEngine(source, NULL, tracker, settings);

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
        engine->WritePhotoErrors("/home/duncan/photo.bin");
        break;
    }
}
