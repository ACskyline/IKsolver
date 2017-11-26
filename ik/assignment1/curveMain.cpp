#include "CurveViewer.h"

int main(int argc, char** argv)
{
	CurveViewer viewer;
	viewer.init(argc, argv);
	viewer.run();
	return 0;
}

