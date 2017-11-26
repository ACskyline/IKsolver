#include "BVHViewer.h"

int main(int argc, char** argv)
{
	BVHViewer viewer;
	viewer.init(argc, argv);
    viewer.loadModel("../models/BetaCharacter.fbx");
    viewer.loadMotion("../motions/Beta/Beta.bvh");
    viewer.loadDir("../motions/Beta/");
	viewer.run();
	return 0;
}

