#ifndef AMotion_H_
#define AMotion_H_

#include "aSplineVec3.h"
#include "aSplineQuat.h"
#include <map>

// A convenience class for collecting together character motions
// Assumes that only the root can be translated (e.g. limbs do not change length)
class AMotion
{
public:
    AMotion() : mRootMotion(), mMotion() {}
    void clear() { mRootMotion.clear(); mMotion.clear(); }
    double getDuration() const { return mRootMotion.getDuration(); }
    double getNormalizedTime(double t) const { return mRootMotion.getNormalizedTime(t); }

    ASplineVec3 mRootMotion;
    std::map<int, ASplineQuat> mMotion;
};

#endif

