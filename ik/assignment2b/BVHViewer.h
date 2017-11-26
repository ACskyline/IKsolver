#pragma once

#include "aBasicViewer.h"
#include "aActor.h"
#include "aBVHController.h"


class BVHViewer : public ABasicViewer
{
public:
    BVHViewer();
    virtual ~BVHViewer();
    virtual void loadMotion(const std::string& filename);

protected:

    virtual void initializeGui();
    virtual void draw3DView();
    virtual void onTimer(int value);
    virtual void load(const std::string& filename);

protected:
    double mFbxTime;
	AActor mActor;
	BVHController* mBVHController;
};
