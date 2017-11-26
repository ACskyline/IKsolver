#ifndef Actor_H_
#define Actor_H_

//#pragma once

//#define BEHAVIOR

#include "aTransform.h"

#include "aJoint.h"
#include "aSkeleton.h"
#include "aBVHController.h"
#include "aIKController.h"
#include "aBehaviorController.h"



class BVHController;
class IKController;


#ifdef BEHAVIOR

	class BehaviorController;
#endif


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

#ifdef BEHAVIOR
	BehaviorController* getBehaviorController();
#endif


protected:
	// the actor owns the skeleton and controllers
	ASkeleton* m_pSkeleton;
	ASkeleton* m_pInternalSkeleton;
	BVHController *m_BVHController;
	IKController *m_IKController;

#ifdef BEHAVIOR
	BehaviorController* m_BehaviorController;
#endif	

};
#endif