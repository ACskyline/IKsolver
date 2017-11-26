#ifndef IK_Actor_H_
#define IK_Actor_H_

#pragma once

#include "aActor.h"
#include "aIKController.h"

class AIKActor : AActor
{

public:
	AIKActor();
	AIKActor(const AIKActor* actor);

	IKController* getIKController();

protected:
	IKController mIKController;

};

#endif

