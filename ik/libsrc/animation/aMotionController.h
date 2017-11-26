#ifndef MotionController_H_
#define MotionController_H_

#include <map>
#include "aCurve.h"
#include "aAnimatables.h"

class AMotionController
{
public:
    AMotionController();
    virtual ~AMotionController();

    virtual void update(double dt);
    virtual void drawOpenGL();
    
    Actor& getSkeleton() { return mSkeleton; }
    const Actor& getSkeleton() const { return mSkeleton; }

    virtual void setSkeleton(const Actor& actor);
    virtual void addMotion(const std::string& name, const AMotion& motion);
    virtual void play(const std::string& name);
    virtual void stop();

protected:
    virtual void clear();
    virtual void computeBlend(const std::string& nextMotion, double blendTime);
    virtual void setPose(AMotion& motion, double time);

protected:
    Actor mSkeleton;
    double mFps;
    double mTime, mStartTime; //normalized time (depends on current motion)

    AMotion mBlend;
    std::map<std::string, AMotion> mMotions;
    std::string mCurrent, mNext;

public:
    double mBlendTime; // TODO: Add params for me
};

#endif
