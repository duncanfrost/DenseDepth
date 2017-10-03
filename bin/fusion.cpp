#include <iostream>
#include <GUI/VisualisationModule.h>
#include <GUI/ARWindow.h>
#include <GUI/FusionWindow.h>
#include <GUI/MapWindow.h>
#include <Engines/FusionEngine.h>
#include <Engines/FileTracker.h>
#include <ImageSource/PhoneSource.h>
#include <thread>
#include <unistd.h>

void Idle(void);
void KeyboardFunction(unsigned char key, int x, int y);
VisualisationModule *visModule;
FusionEngine *engine;
PhoneSource *source;
FileTracker *tracker;
bool paused = false;

int main(void)
{
    std::string filename1 = "/home/duncan/Data/P9/Office3/log.txt";
    std::string gtFile = "/home/duncan/Data/P9/Office3/CameraTrajectory.txt";

    source = new PhoneSource(filename1);
    tracker = new FileTracker(gtFile);
    engine = new FusionEngine(source, tracker);

    visModule = new VisualisationModule(&Idle);
    visModule->AddWindow(new ARWindow("AR",640,480,engine->GetARData()));
    visModule->AddWindow(new FusionWindow("Fusion",640,480, engine));

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

    case 'p':
        paused = !paused;
        std::cout << "Toggle paused: " << paused << std::endl;
        break;
    }
}
