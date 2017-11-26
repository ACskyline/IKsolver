
#ifndef BVHCONTROLLER_H_
#define BVHCONTROLLER_H_

#pragma once

#include <map>
#include <string>
#include <fstream>

#include "aJoint.h"
#include "aSkeleton.h"

#include "aSplineVec3.h"
#include "aSplineQuat.h"


class AActor;  // forward declaration since BVHController class references AActor and AActor class references BVHController

class BVHController
{
public:
    BVHController();
    virtual ~BVHController();
    virtual void update(double time);
    virtual bool load(const std::string& filename);

	ASkeleton* getSkeleton(); // skeleton contains the joint transform hierarchy
	const ASkeleton* getSkeleton() const;
	AActor* getActor();
	void setActor(AActor* actor);

protected:
    virtual quat ComputeBVHRot(float r1, float r2, float r3, const std::string& rotOrder);
    virtual bool loadSkeleton(std::ifstream &inFile);
    virtual bool loadJoint(std::ifstream &inFile, AJoint *pParent, std::string prefix);
    virtual bool loadMotion(std::ifstream &inFile);
    virtual void loadFrame(std::ifstream& inFile);
    virtual void clear();

protected:
    std::string mFilename;
    AActor* mActor;
	ASkeleton* mSkeleton;
    double mFps;
    double mDt;
    ASplineVec3 mRootMotion;
    std::map<int, ASplineQuat> mMotion;
};

#endif