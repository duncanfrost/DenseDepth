#include <iostream>
#include <GUI/VisualisationModule.h>
#include <GUI/ARWindow.h>
#include <GUI/MapWindow.h>
#include <Engines/MonoEngine.h>
#include <Tracking/ARKitFileTracker.h>
#include <ImageSource/ARKitSource.h>

void Idle(void);
void KeyboardFunction(unsigned char key, int x, int y);
VisualisationModule *visModule;
MonoEngine *engine;
ARKitSource *source;
FileTracker *tracker;

int main(int argc, char* argv[])
{
    // std::string sequence = "seq4";
    std::string sequence = argv[1];

    source = new ARKitSource("/home/duncan/Data/ARKit/" + sequence + "/","Exposure.txt");
    tracker = new ARKitFileTracker("/home/duncan/Data/ARKit/" + sequence + "/Poses.txt");

    MonoEngine::Settings settings;
    settings.fx = 1012.501526;
    settings.fy = 1012.501526;
    settings.cx = 635.658752;
    settings.cy = 341.148590;
    settings.inputSizeX = 1280;
    settings.inputSizeY = 720;
    settings.targetSizeX = 1280/2;
    settings.targetSizeY = 720/2;

    engine = new MonoEngine(source, NULL, tracker, settings);

    visModule = new VisualisationModule(&Idle);
    visModule->AddWindow(new ARWindow("AR",settings.targetSizeX,settings.targetSizeY, engine->GetARData()));
    visModule->AddWindow(new ARWindow("Inverse Depth",settings.targetSizeX,settings.targetSizeY,
                                      engine->GetDepthData()));
    visModule->AddWindow(new MapWindow("Map",1000, 500,engine));

    visModule->SetKeyboardFunction(&KeyboardFunction);
    visModule->StartLoop();
}

void Idle(void)
{
    engine->Process();
    engine->DoSmoothProcess();


    visModule->DrawWindows();
}

void KeyboardFunction(unsigned char key, int x, int y)
{
    switch(key) {
    case 27://esc
        exit(0);
        break;

    case ' ':
        engine->AddKeyFrameManual();

        break;

    case 'h':
        engine->Sample();
        break;

    case 'p':
        engine->TogglePaused();
        break;
        
    case 'o':
        engine->SmoothPhotoActive();
        break;

    case 'y':
        engine->monoDepthEstimator->InitOptim();
        break;
        

        // engine->SmoothPhotoBuffer(200);
        // engine->WritePhotoErrors("/home/duncan/photo.bin");
        break;
    }
}
