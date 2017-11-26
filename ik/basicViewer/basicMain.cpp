#include "windows.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include "aBasicViewer.h"

int main(int argc, char** argv)
{
    ABasicViewer viewer;
    viewer.init(argc, argv);
    viewer.loadModel("../models/Warrok.fbx");
    viewer.run();
    return 0;
}

