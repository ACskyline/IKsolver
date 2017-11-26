#include "IKViewer.h"

int main(int argc, char** argv)
{
	IKViewer viewer;
	viewer.init(argc, argv);
    viewer.loadModel("../models/BetaCharacter.fbx");
	viewer.run();
	return 0;
}

