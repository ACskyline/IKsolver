#ifndef IKViewer_H_
#define IKViewer_H_

#include "aBasicViewer.h"

#include "aJoint.h"
#include "aSkeleton.h"

#include "aActor.h"
#include "aIKController.h"

class IKViewer : public ABasicViewer
{
public:
    IKViewer();
    virtual ~IKViewer();
    virtual bool loadModel(const std::string& filename);

protected:

    virtual void initializeGui();
    virtual void draw3DView();
    virtual void drawOverlay();
    virtual void onTimer(int value);
    virtual void onSelectIKJoint(int selectedJoint);
    virtual void loadIKJoints();
    virtual void updateIK();
    virtual void reset();
    virtual void onMouseMotion(int pX, int pY);
    virtual void onMouse(int button, int state, int x, int y);

    static void TW_CALL ResetIKCb(void* clientData);
    static void TW_CALL SelectIKCb(void* clientData);
    static void TW_CALL onSetIKCb(const void *value, void *clientData);
    static void TW_CALL onGetIKCb(void *value, void *clientData);

    static void TW_CALL onSetGoalXCb(const void *value, void *clientData);
    static void TW_CALL onGetGoalXCb(void *value, void *clientData);
    static void TW_CALL onSetGoalYCb(const void *value, void *clientData);
    static void TW_CALL onGetGoalYCb(void *value, void *clientData);
    static void TW_CALL onSetGoalZCb(const void *value, void *clientData);
    static void TW_CALL onGetGoalZCb(void *value, void *clientData);

protected:
    AActor mActor;
	ATarget mTarget;
    IKController* mIKController;

    vec3 mTargetPosition;
    int mEndJointID;
    int mIKChainSize;
    bool mSelectedRecticle;
    double mFbxTime;
    int mLastIKX, mLastIKY;

    TwBar *mPoseEditBar;
    struct EffectorData { IKViewer* viewer; int jointid; std::string name; };
    std::vector<EffectorData*> mEffectorData;

    enum IKType { LIMB, CCD, PSEUDO, OTHER} mType;
    TwType ikType;
};

#endif
