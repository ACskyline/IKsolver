#include <string>
#include "BVHViewer.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <algorithm>
#include <AntTweakBar.h>

#pragma warning(disable : 4018)

BVHViewer::BVHViewer() 
{
		
	mBVHController = mActor.getBVHController();
}

BVHViewer::~BVHViewer()
{
}

void BVHViewer::load(const std::string& filename)
{
    loadMotion(filename);  // loads a motion file (assumed to be in bvh format)
}

void BVHViewer::loadMotion(const std::string& filename)
{
    mBVHController->load(filename);
	ASkeleton* skeleton = mBVHController->getSkeleton();
    mFBX->SetPose(*skeleton);
    mCurrentLayer = mFBX->GetNumLayers() - 1;
    mFilename = pruneName(filename);
}

void BVHViewer::initializeGui()
{
    ABasicViewer::initializeGui();
}

void BVHViewer::onTimer(int value)
{
    // always update FBX time
    double dt = mClock->totalElapsedTime(); // needs to be called before base in case clock is reset
    mFbxTime = mFbxTime + dt;

	ABasicViewer::onTimer(value);
    mBVHController->update(mCurrentTime); // this updates the skeleton pose using the bvh data at the current time
	ASkeleton* skeleton = mBVHController->getSkeleton();
    mFBX->SetPose(*skeleton);
}

void BVHViewer::draw3DView()
{
    glViewport(0, 0, (GLsizei)mWindowWidth, (GLsizei)mWindowHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE); // Draw the front face only, except for the texts and lights.
    glEnable(GL_LIGHTING);

    // Set the view to the current camera settings.
    mCamera.draw();

    GLfloat pos[4];
    pos[0] = mCamera.getPosition()[0];
    pos[1] = mCamera.getPosition()[1];
    pos[2] = mCamera.getPosition()[2];
    pos[3] = 1.0;
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    glDisable(GL_TEXTURE_2D);
    mFBX->Draw(mFbxTime, 1);

    glDisable(GL_LIGHTING);
    displayGrid();

	//originally in  BVHController::drawOpenGL()
	glColor4f(0, 0, 1, 1);
	glBegin(GL_LINES);
	ASkeleton* skeleton = mActor.getSkeleton();
	for (int i = 0; i < skeleton->getNumJoints(); i++)
	{
		AJoint* jointnode = skeleton->getJointByID(i);
		AJoint* pParent = jointnode->getParent();
		if (!pParent) continue;

		vec3 startPosition = pParent->getGlobalTranslation();
		vec3 endPosition = jointnode->getGlobalTranslation();

		glVertex3f(startPosition[0], startPosition[1], startPosition[2]);
		glVertex3f(endPosition[0], endPosition[1], endPosition[2]);
	}
	glEnd();
	
}

