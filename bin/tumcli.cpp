#include <iostream>
#include <GUI/VisualisationModule.h>
#include <GUI/ARWindow.h>
#include <GUI/MapWindow.h>
#include <Engines/MonoEngine.h>
#include <Tracking/TUMFileTracker.h>
#include <ImageSource/TUMSource.h>
#include <ImageSource/TUMDepthSource.h>

void Idle(void);
void KeyboardFunction(unsigned char key, int x, int y);
VisualisationModule *visModule;
MonoEngine *engine;
TUMSource *source;
TUMDepthSource *depthSource;
FileTracker *tracker;
bool paused = false;

int main(void)
{
    std::string filename1 = "/home/duncan/Data/TUM/rgbd_dataset_freiburg2_desk/rgb.txt";
    std::string filename2 = "/home/duncan/Data/TUM/rgbd_dataset_freiburg2_desk/depth.txt";


    std::string poseDirectory = "/home/duncan/Data/TUM/rgbd_dataset_freiburg2_desk/";

    source = new TUMSource(filename1);
    depthSource = new TUMDepthSource(filename2);
    tracker = new TUMFileTracker(poseDirectory, "groundtruth.txt");

    MonoEngine::Settings settings;
    settings.fx = 520.908620;
    settings.fy = 521.007327;
    settings.cx = 325.141442;
    settings.cy = 249.701764;

    engine = new MonoEngine(source, depthSource, tracker, settings);

    for (unsigned int i = 0; i < 300; i++)
        engine->Process();
}
