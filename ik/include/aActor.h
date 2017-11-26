#ifndef Actor_H_
#define Actor_H_

//#pragma once

#include "aTransform.h"

#include "aJoint.h"
#include "aSkeleton.h"
#include "aBVHController.h"
#include "aIKController.h"
#include "aBehaviorController.h"


//class BVHController;
//class IKController;
class BehaviorController;

class AActor //: ASkeleton
{

public:
	AActor();
	AActor(const AActor* actor); 
	virtual ~AActor();

	virtual AActor& operator=(const AActor& actor); 
	void clear();
	void update();


	ASkeleton* getSkeleton();
	void setSkeleton(ASkeleton* pExternalSkeleton);
	void resetSkeleton();
	BVHController* getBVHController();
	IKController* getIKController();
	BehaviorController* getBehaviorController();

protected:
	// the actor owns the skeleton and controllers
	ASkeleton* m_pSkeleton;
	ASkeleton* m_pInternalSkeleton;
	BVHController *m_BVHController;
	IKController *m_IKController;
	BehaviorController* m_BehaviorController;
};

#endif