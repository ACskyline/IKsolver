#include "aIKActor.h"

AIKActor::AIKActor() : AActor()
{
	mIKController.setActor(this);
}

AIKActor::AIKActor(const AIKActor* actor)
{
	*this = *actor;
}

IKController* AIKActor::getIKController()
{
	return &mIKController;
}
