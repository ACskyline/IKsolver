#include "aBVHController.h"
#include "aVector.h"
#include "aRotation.h"
#include <iostream>
#include <GL/glut.h>

#include "aActor.h"


#pragma warning(disable:4018)

BVHController::BVHController() 
{
	mActor = NULL;
	mSkeleton = NULL;
	mFps = 120.0;
	mDt = 0.008333;
}

BVHController::~BVHController()
{
    clear();
}

void BVHController::clear()
{
    mFilename = "";
    mActor->clear();
    mRootMotion.clear();
    mMotion.clear();
}

ASkeleton* BVHController::getSkeleton()
{ 
	return mSkeleton; 
}

const ASkeleton* BVHController::getSkeleton() const
{
	return mSkeleton;
}

AActor* BVHController::getActor()
{
	return mActor;
}


void BVHController::setActor(AActor* actor)

{
	mActor = actor;
	mSkeleton = mActor->getSkeleton();
}


void BVHController::update(double time)
{
	ASkeleton* skeleton = mActor->getSkeleton();

	// TODO: Update transforms at each Skeleton joint given spline motion data in mRootMotion and mMotion for value of time. 

	int jNum = skeleton->getNumJoints();

	skeleton->getRootNode()->setLocalTranslation(mRootMotion.getValue(time));

	for (int i = 0; i < jNum; i++)
	{
		AJoint* jTemp = skeleton->getJointByID(i);
		jTemp->setLocalRotation(mMotion[i].getCachedValue(time).ToRotation());
	}

	skeleton->update();

}

bool BVHController::load(const std::string& filename)
{
    std::ifstream inFile(filename.c_str());
    if (!inFile.is_open())
    {
        std::cout << "WARNING: Could not open " << filename.c_str() << std::endl;
        return false;
    }

    clear();
    bool status = loadSkeleton(inFile) && loadMotion(inFile);

    if (status)
    {
        mFilename = filename;
        //std::string motionName = filename;
        //std::string annName = motionName.replace(motionName.end() - 4, motionName.end(), ".ann");
        //m_motion.LoadANNFile(annName, m_skeleton);
    }
    inFile.close();
    return status;
}

bool BVHController::loadSkeleton(std::ifstream& inFile)
{
    clear();

    vec3 offsets;
    std::string readString, jointname;
    unsigned int channelCount;
	ASkeleton* skeleton = mActor->getSkeleton();

    inFile >> readString;
    if (readString != "HIERARCHY")
        return false;
    inFile >> readString;
    if (readString != "ROOT" && readString != "JOINT")
        return false;
    inFile.get(); //" "
    getline(inFile, jointname);// jointnode name
    AJoint* jointnode = new AJoint(jointname);
	skeleton->addJoint(jointnode, true);
    inFile >> readString; // "{"
    inFile >> readString; // "OFFSET"
    inFile >> offsets[0] >> offsets[1] >> offsets[2];
    jointnode->setLocalTranslation(offsets);
    inFile >> readString;
    if (readString != "CHANNELS")
        return false;
    inFile >> channelCount;
    jointnode->setNumChannels(channelCount);
    getline(inFile, readString);	// " Xposition Yposition Zposition Zrotation Xrotation Yrotation"
    jointnode->setRotationOrder(readString);
    inFile >> readString;
    while (readString != "}")
    {
        if (!loadJoint(inFile, jointnode, readString))
        {
            return false;
        }
        inFile >> readString;
    }
    if (readString != "}") return false;

	skeleton->update();
    return true;
}

bool BVHController::loadJoint(std::ifstream &inFile, AJoint *pParent, std::string prefix)
{
    std::string readString, jointname;
    vec3 offsets;
    unsigned int channelCount;
	ASkeleton* skeleton = mActor->getSkeleton();

    if (prefix == "JOINT")
    {
        inFile.get(); //" "
        getline(inFile, jointname);// jointnode name
        AJoint* jointnode = new AJoint(jointname);

		skeleton->addJoint(jointnode, false);
        AJoint::Attach(pParent, jointnode);
        inFile >> readString; // "{"
        inFile >> readString; // "OFFSET"
        inFile >> offsets[0] >> offsets[1] >> offsets[2];
        jointnode->setLocalTranslation(offsets);
        inFile >> readString; // "CHANNELS"
        inFile >> channelCount;
        jointnode->setNumChannels(channelCount);

        getline(inFile, readString);// " Zrotation Xrotation Yrotation"
        jointnode->setRotationOrder(readString);

        inFile >> readString; // "Joint" or "}" or "End"
        while (readString != "}")
        {
            if (loadJoint(inFile, jointnode, readString) == false)
                return false;
            inFile >> readString; // "Joint" or "}" or "End"
        }
        return true;
    }
    else if (prefix == "End")
    {
        inFile.get(); //" "
        getline(inFile, jointname);// jointnode name
        if (jointname.find("Site") != std::string::npos)
        {
            jointname = pParent->getName() + "Site";
        }

        AJoint* jointnode = new AJoint(jointname);
        jointnode->setNumChannels(0);
		skeleton->addJoint(jointnode, false);
        AJoint::Attach(pParent, jointnode);
        inFile >> readString; // "{"
        inFile >> readString; // "OFFSET"
        inFile >> offsets[0] >> offsets[1] >> offsets[2];
        jointnode->setLocalTranslation(offsets);
        inFile >> readString; // "}"
        return true;
    }
    else return false;
}



bool BVHController::loadMotion(std::ifstream& inFile)
{
    std::string readString;
    unsigned int frameCount;
    inFile >> readString;
    if (readString != "MOTION")
        return false;
    inFile >> readString;
    if (readString != "Frames:")
        return false;
    inFile >> frameCount;
    inFile >> readString; // "Frame"
    getline(inFile, readString); // " Time: 0.033333"
    mDt = atof(&(readString.c_str()[6]));
    mFps = 1.0 / mDt;

	ASkeleton* skeleton = mActor->getSkeleton();
    // Init rotation curves
	for (unsigned int i = 0; i < skeleton->getNumJoints(); i++)
    {
        ASplineQuat q;
        q.setFramerate(mFps);
        q.setInterpolationType(ASplineQuat::LINEAR);
        mMotion[i] = q;
    }
    mRootMotion.setFramerate(mFps);
    mRootMotion.setInterpolationType(ASplineVec3::LINEAR);

    // Read frames
    for (unsigned int i = 0; i < frameCount; i++)
    {
       loadFrame(inFile);
    }

    mRootMotion.computeControlPoints();
    mRootMotion.cacheCurve();
	for (unsigned int i = 0; i < skeleton->getNumJoints(); i++)
    {
        mMotion[i].cacheCurve();
    }
    return true;
}

void BVHController::loadFrame(std::ifstream& inFile)
{
    float tx, ty, tz, r1, r2, r3;
    double t = mDt * mRootMotion.getNumKeys();
	ASkeleton* skeleton = mActor->getSkeleton();
	for (unsigned int i = 0; i < skeleton->getNumJoints(); i++)
    {
        tx = ty = tz = 0.0f;
        r1 = r2 = r3 = 0.0f;

		AJoint* pJoint = skeleton->getJointByID(i);
        if (pJoint->getNumChannels() == 6)
        {
            inFile >> tx >> ty >> tz;
            inFile >> r1 >> r2 >> r3;
        }
        else if (pJoint->getNumChannels() == 3)
        {
            inFile >> r1 >> r2 >> r3;
        }
        else
        {
        }

		if (skeleton->getRootNode() == pJoint)
        {
            mRootMotion.appendKey(t, vec3(tx, ty, tz), false);
        }

        quat q = ComputeBVHRot(r1, r2, r3, pJoint->getRotationOrder());
        mMotion[i].appendKey(t, q, false);
    }
}

quat BVHController::ComputeBVHRot(float r1, float r2, float r3, const std::string& rotOrder) // For BVH
{
    mat3 m;
    float ry, rx, rz;

    if (rotOrder == "xyz")
    {
        rx = r1; ry = r2; rz = r3;
        m.FromEulerAngles(mat3::XYZ, vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "xzy")
    {
        rx = r1; rz = r2; ry = r3;
		m.FromEulerAngles(mat3::XZY, vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "yxz")
    {
        ry = r1; rx = r2; rz = r3;
		m.FromEulerAngles(mat3::YXZ, vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "yzx")
    {
        ry = r1; rz = r2; rx = r3;
		m.FromEulerAngles(mat3::YZX, vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "zxy")
    {
        rz = r1; rx = r2; ry = r3;
		m.FromEulerAngles(mat3::ZXY, vec3(rx, ry, rz) * Deg2Rad);
    }
    else if (rotOrder == "zyx")
    {
        rz = r1; ry = r2; rx = r3;
		m.FromEulerAngles(mat3::ZYX, vec3(rx, ry, rz) * Deg2Rad);
    }
    return m.ToQuaternion();
}
