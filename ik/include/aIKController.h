#ifndef IKController_H_
#define IKController_H_

#pragma once


#include "aJoint.h"
#include "aSkeleton.h"
#include "aTarget.h"

class AActor;  // forward declaration since IKController class references AActor and AActor class references IKController

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
class AIKchain
{
public:
	AIKchain();
	virtual ~AIKchain();

	AJoint* getJoint(int index);
	void setJoint(int index, AJoint* pJoint);

	double getWeight(int index);
	std::vector<double>& getWeights();
	void setWeight(int index, double weight);
	void setWeights(std::vector<double> weights);

	int getSize();

	std::vector<AJoint*>& getChain();
	void setChain(std::vector<AJoint*> chain);

protected:
	std::vector<AJoint*> mChain;
	std::vector<double> mWeights;
	double mWeight0;
};

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
class IKController
{
public:
    IKController();
    virtual ~IKController();


	ASkeleton* getSkeleton();         
	const ASkeleton* getSkeleton() const;

	ASkeleton* getIKSkeleton(); 
	const ASkeleton* getIKSkeleton() const;

	AActor* getActor();
	void setActor(AActor* actor);

	bool IKSolver_Limb(int endJointID, const ATarget& target);
	bool IKSolver_CCD(int endJointID, const ATarget& target);
	bool IKSolver_CCD2(int endJointID, const ATarget& target);
	bool IKSolver_PseudoInv(int endJointID, const ATarget& target);
	bool IKSolver_Other(int endJointID, const ATarget& target);

protected:

	AActor* m_pActor;
	ASkeleton* m_pSkeleton;
	ASkeleton mIKSkeleton, mIKSkeleton1, mIKSkeleton2;

	bool mValidChain = true;
	bool mvalidLimbIKchains;
	bool mvalidCCDIKchains;

    AIKchain mRhandIKchain; // IK chain of joint pointers starting with Rhand joint 
	AIKchain mLhandIKchain; // IK chain of joint pointers starting with Lhand joint     
	AIKchain mRfootIKchain; // IK chain of joint pointers starting with Rfoot joint 
	AIKchain mLfootIKchain; // IK chain of joint pointers starting with Lfoot joint 
	AIKchain mIKchain;  // IK chain of joint pointers starting with end joint 

	int createLimbIKchains();
	int createCCDIKchains();

	AIKchain createIKchain(int endJointID, int baseJointID, int chainSize, ASkeleton* pSkeleton);
	AIKchain createIKchain(int endJointID, int desiredChainSize, ASkeleton* pSkeleton);

	int computeLimbIK(ATarget target, AIKchain& IKchain, const vec3 axis, ASkeleton* pIKSkeleton);
	int computeCCDIK(ATarget target, AIKchain&, ASkeleton* pIKSkeleton);

    int mEndJointID = -1;
    int mChainSize = -1;

	int mNumJoints = 0;
	
	enum EndJointIndex { ROOT, LHAND, RHAND, LFOOT, RFOOT } mEndJointIndex;

	// End Joint IDs
	// assumes skeleton structure associated with beta character
	static const int mRootID = 0;   
	static const int mLhandID = 10;
	static const int mRhandID = 29;
	static const int mLfootID = 47;
	static const int mRfootID = 51;

	ATarget mLfootTarget;
	ATarget mRfootTarget;
	ATarget mLhandTarget;
	ATarget mRhandTarget;
	ATarget mHeadTarget;

	// Limb (Geometric) IK variables
	ATarget mTarget0;       // target associated with end joint
	ATarget mTarget1;       // target associated with middle joint - used to specify rotation of middle joint about end/base axis  
	AJoint* m_pEndJoint;    // Joint 2
	AJoint* m_pMiddleJoint; // Joint 1
	AJoint* m_pBaseJoint;   // Joint 0
	double m_length01;
	double m_length12;

	vec3 m_rotationAxis;    // axis of rotation for middle joint.  Values can be axisX, axisY or axisZ

	// CCD IK variables
	double mWeight0;

public:
    static double gIKEpsilon;
    static int gIKmaxIterations;

};

#endif