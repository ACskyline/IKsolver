#include <string>
#include "IKViewer.h"
#include "GL/glew.h"
#include "GL/glut.h"
#include <algorithm>
#include <AntTweakBar.h>
 
#pragma warning(disable : 4018)

IKViewer::IKViewer() 
{
	mEndJointID = 10;

	mTargetPosition = vec3(0.0);
	mType = LIMB;
	mIKChainSize = -1;

	mIKController = mActor.getIKController();
}

IKViewer::~IKViewer()
{
}

void IKViewer::initializeGui()
{
    ABasicViewer::initializeGui();

    TwDefine(" 'File controls' size='200 300' position='5 185' iconified=true fontresizable=false alpha=200");
    TwDefine(" 'Player controls' size='200 175' position='5 5' iconified=true fontresizable=false alpha=200");

    mPoseEditBar = TwNewBar("Edit Pose");
    TwDefine(" 'Edit Pose' size='200 500' position='5 5' iconified=false fontresizable=false alpha=200");
    TwEnumVal ikTypeEV[] =
    {
        { LIMB, "Limb" },
        { CCD, "CCD" },
		{ PSEUDO, "PseudoInv" },
		{ OTHER, "Other" }
    };
    ikType = TwDefineEnum("IKType", ikTypeEV, 4);
    TwAddVarCB(mPoseEditBar, "Type", ikType, onSetIKCb, onGetIKCb, this, " ");
    TwAddButton(mPoseEditBar, "Reset Pose", ResetIKCb, this, " ");
    TwAddVarRW(mPoseEditBar, "Epsilon", TW_TYPE_DOUBLE, &IKController::gIKEpsilon, " ");
    TwAddVarRW(mPoseEditBar, "Max Iterations", TW_TYPE_INT32, &IKController::gIKmaxIterations, " ");
    TwAddVarRW(mPoseEditBar, "Chain Length", TW_TYPE_INT32, &mIKChainSize, " ");
    TwAddVarCB(mPoseEditBar, "X", TW_TYPE_DOUBLE, onSetGoalXCb, onGetGoalXCb, this, " group='Target/Goal position'");
    TwAddVarCB(mPoseEditBar, "Y", TW_TYPE_DOUBLE, onSetGoalYCb, onGetGoalYCb, this, " group='Target/Goal position'");
    TwAddVarCB(mPoseEditBar, "Z", TW_TYPE_DOUBLE, onSetGoalZCb, onGetGoalZCb, this, " group='Target/Goal position'");
}

bool IKViewer::loadModel(const std::string& filename)
{
    bool success = ABasicViewer::loadModel(filename);

	if (success)
    {
		ASkeleton* skeleton = mIKController->getSkeleton();
		ASkeleton* IKskeleton = mIKController->getIKSkeleton();
		*IKskeleton = mFBX->ExportSkeleton();

		mFBX->GetPose(0, *IKskeleton);
		IKskeleton->update();

		*skeleton = *IKskeleton;

        loadIKJoints();
    }
	reset();
	return success;
}

void IKViewer::reset()
{
	ASkeleton* skeleton = mIKController->getSkeleton();
    mFBX->GetPose(0, *skeleton);
    skeleton->update();

    // make the Left hand the default joint selected
	mEndJointID = 10; // Left Hand Joint ID
	onSelectIKJoint(mEndJointID);
	mTargetPosition = skeleton->getJointByID(mEndJointID)->getGlobalTranslation();
	drawOverlay();


    glutPostRedisplay();

}

void IKViewer::onTimer(int value)
{
    // always update FBX time
    double dt = mClock->totalElapsedTime(); // needs to be called before base in case clock is reset
    mFbxTime = mFbxTime + dt;

	ABasicViewer::onTimer(value); 
	ASkeleton* skeleton = mIKController->getSkeleton();
	mFBX->SetPose(*skeleton);
}

void IKViewer::draw3DView()
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

    mFBX->Draw(mFbxTime, 1);

    glDisable(GL_LIGHTING);
    displayGrid();
}

void DrawCircle(const vec3& p)
{
    int vertices = 20;
    double r = 5.0;
    double tmpX, tmpY, tmpZ;
    double Angle, Angle0;

    Angle = -(2 * 3.14) / vertices;
    Angle0 = 0.0;

    tmpX = p[0];
    tmpY = p[1];
    tmpZ = p[2];

    glBegin(GL_LINE_LOOP);

    for (int i = 0; i < vertices; i++) {
        glVertex3f(tmpX + r * cos(i*Angle + Angle0), tmpY + r * sin(i*Angle + Angle0), tmpZ);
    }
    glEnd();
}

void IKViewer::drawOverlay()
{
    int screenX, screenY;
    mCamera.worldToScreen(mTargetPosition, screenX, screenY);

    glColor3f(1.0, 0.0, 1.0);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, mWindowWidth, 0, mWindowHeight);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    DrawCircle(vec3(screenX, screenY, 0));
    glBegin(GL_LINES);
    glVertex3f(screenX + 10.0, screenY, 0);
    glVertex3f(screenX - 10.0, screenY, 0);
    glVertex3f(screenX, screenY + 10.0, 0);
    glVertex3f(screenX, screenY - 10.0, 0);
    glEnd();
}

void TW_CALL IKViewer::ResetIKCb(void* clientData)
{
    IKViewer* viewer = (IKViewer*)clientData;
    viewer->reset();
}

void TW_CALL IKViewer::SelectIKCb(void* clientData)
{
    EffectorData* data = (EffectorData*)clientData;
    IKViewer* viewer = data->viewer;
    int jointid = data->jointid;
    std::string name = data->name;

    std::cout << "Select joint: " << name.c_str() << std::endl;
    viewer->onSelectIKJoint(jointid);
}

void IKViewer::onSelectIKJoint(int selectedJoint)
{
    mEndJointID = selectedJoint;
	ASkeleton* skeleton = mIKController->getSkeleton();
    mTargetPosition = skeleton->getJointByID(mEndJointID)->getGlobalTranslation();

}

void IKViewer::loadIKJoints()
{
    for (int i = 0; i < mEffectorData.size(); i++)
    {
        TwRemoveVar(mPoseEditBar, mEffectorData[i]->name.c_str());
        delete mEffectorData[i];
    }
    mEffectorData.clear();

    char buff[256];
    for (int i = 0; i < mIKController->getSkeleton()->getNumJoints(); i++)
    {
        AJoint* joint = mIKController->getSkeleton()->getJointByID(i);

        EffectorData* ld = new EffectorData;
        ld->viewer = this;
        ld->jointid = i;
        ld->name = joint->getName();
        mEffectorData.push_back(ld);

        sprintf(buff, " label='%s' group='Select Joint'", joint->getName().c_str());
        TwAddButton(mPoseEditBar, mEffectorData.back()->name.c_str(), SelectIKCb, ld, buff);
    }
}

void IKViewer::updateIK() // assumes joint mEndjointID already selected with desired location in mTargetPosition
{

	mTarget.setLocalTranslation(mTargetPosition);
	mTarget.update();  // updates target local to global transform

	switch (mType)
	{
	case LIMB:
		mIKController->IKSolver_Limb(mEndJointID, mTarget);
		break;
	case CCD:
		mIKController->IKSolver_CCD(mEndJointID, mTarget);
		break;
	case PSEUDO:
		mIKController->IKSolver_PseudoInv(mEndJointID, mTarget);
		break;
	case OTHER:
		mIKController->IKSolver_Other(mEndJointID, mTarget);
		break;
	}

}

void IKViewer::onMouseMotion(int pX, int pY)
{
    // Check GUI first
    if (TwEventMouseMotionGLUT(pX, pY)) return;

    if (mModifierState == GLUT_ACTIVE_CTRL && mEndJointID > -1 && mButtonState == GLUT_LEFT_BUTTON)
    {
        //std::cout << pX << " " << pY << std::endl;
        if (mSelectedRecticle) // update pos with new p
        {
            double dsqr = DistanceSqr(vec3(mLastIKX, mWindowHeight - mLastIKY, 0), vec3(pX, pY, 0));
            //std::cout << dsqr << std::endl;
            if (dsqr > 300.0)
            {
                vec3 target(0, 0, 0);
                vec3 current(0, 0, 0);
                mCamera.screenToWorld(mLastIKX, mWindowHeight - mLastIKY, current);
                mCamera.screenToWorld(pX, mWindowHeight - pY, target);

                // Did we click on the IK sphere previously?
                vec3 eye = mCamera.getPosition();

                double distToScreen = Distance(eye, current);
                double distToScreenTarget = Distance(current, target);
                double distToObject = Distance(eye, mTargetPosition);
                double distToTarget = distToObject*distToScreenTarget / distToScreen; // similar triangles

                vec3 dir = target - current;
                dir.Normalize();
                dir = dir * distToTarget;
                mTargetPosition = mTargetPosition + dir;

                updateIK();
                mLastIKX = pX; 
                mLastIKY = pY;
            }
        }
    }


    int deltaX = mLastX - pX;
    int deltaY = mLastY - pY;
    bool moveLeftRight = abs(deltaX) > abs(deltaY);
    bool moveUpDown = !moveLeftRight;
    if (mModifierState != GLUT_ACTIVE_CTRL && mButtonState == GLUT_LEFT_BUTTON)  // Rotate
    {
        if (moveLeftRight && deltaX > 0) mCamera.orbitLeft(deltaX);
        else if (moveLeftRight && deltaX < 0) mCamera.orbitRight(-deltaX);
        else if (moveUpDown && deltaY > 0) mCamera.orbitUp(deltaY);
        else if (moveUpDown && deltaY < 0) mCamera.orbitDown(-deltaY);
    }
    else if (mButtonState == GLUT_MIDDLE_BUTTON) // Zoom
    {
        if (moveUpDown && deltaY > 0) mCamera.moveForward(deltaY);
        else if (moveUpDown && deltaY < 0) mCamera.moveBack(-deltaY);
    }
    else if (mButtonState == GLUT_RIGHT_BUTTON) // Pan
    {
        if (moveLeftRight && deltaX > 0) mCamera.moveLeft(deltaX);
        else if (moveLeftRight && deltaX < 0) mCamera.moveRight(-deltaX);
        else if (moveUpDown && deltaY > 0) mCamera.moveUp(deltaY);
        else if (moveUpDown && deltaY < 0) mCamera.moveDown(-deltaY);
    }

    mLastX = pX;
    mLastY = pY;
}

void IKViewer::onMouse(int pButton, int pState, int pX, int pY)
{
    // Are we in the recticle?
    ABasicViewer::onMouse(pButton, pState, pX, pY);

    if (mModifierState == GLUT_ACTIVE_CTRL && mEndJointID > -1)
    {
        int screenX, screenY;
        mCamera.worldToScreen(mTargetPosition, screenX, screenY);

        double d = (screenX - pX)*(screenX - pX) + (screenY - mWindowHeight + pY)*(screenY - mWindowHeight + pY);
        mSelectedRecticle = (d <= 25);  // recticle has radius 5 in screen space

        mLastIKX = pX;
        mLastIKY = pY;
    }
    else
    {
        mSelectedRecticle = false;
    }
}

void TW_CALL IKViewer::onSetIKCb(const void *value, void *clientData)
{
    IKViewer* viewer = ((IKViewer*)clientData);
    IKType v = *(const IKType *)value;  // for instance
    viewer->mType = v;
}

void TW_CALL IKViewer::onGetIKCb(void *value, void *clientData)
{
    IKViewer* viewer = ((IKViewer*)clientData);
    *static_cast<IKType *>(value) = viewer->mType;
}

void TW_CALL IKViewer::onSetGoalXCb(const void *value, void *clientData)
{
    IKViewer* viewer = ((IKViewer*)clientData);
    double v = *(const double *)value;  // for instance
    viewer->mTargetPosition[0] = v;
    viewer->updateIK();
}

void TW_CALL IKViewer::onGetGoalXCb(void *value, void *clientData)
{
    IKViewer* viewer = ((IKViewer*)clientData);
    *static_cast<double *>(value) = viewer->mTargetPosition[0];
}

void TW_CALL IKViewer::onSetGoalYCb(const void *value, void *clientData)
{
    IKViewer* viewer = ((IKViewer*)clientData);
    double v = *(const double *)value;  // for instance
    viewer->mTargetPosition[1] = v;
    viewer->updateIK();
}

void TW_CALL IKViewer::onGetGoalYCb(void *value, void *clientData)
{
    IKViewer* viewer = ((IKViewer*)clientData);
    *static_cast<double *>(value) = viewer->mTargetPosition[1];
}

void TW_CALL IKViewer::onSetGoalZCb(const void *value, void *clientData)
{
    IKViewer* viewer = ((IKViewer*)clientData);
    double v = *(const double *)value;  // for instance
    viewer->mTargetPosition[2] = v;
    viewer->updateIK();
}

void TW_CALL IKViewer::onGetGoalZCb(void *value, void *clientData)
{
    IKViewer* viewer = ((IKViewer*)clientData);
    *static_cast<double *>(value) = viewer->mTargetPosition[2];
}