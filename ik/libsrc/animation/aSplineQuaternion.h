#ifndef ASplineQuaternion_H_
#define ASplineQuaternion_H_

#include "aRotation.h"
#include <map>
#include <vector>

class ASplineQuaternion 
{
public:
    enum InterpolationType { LINEAR, CUBIC };
    typedef std::pair<double, Quaternion> Key;

public:
    ASplineQuaternion();
    virtual ~ASplineQuaternion();

    void setLooping(bool loop);
    bool getLooping() const;

    void setFramerate(double fps);
    double getFramerate() const;

    void setInterpolationType(InterpolationType type);
    InterpolationType getInterpolationType() const;

    Quaternion getValue(double t);

    void editKey(int keyID, const Quaternion& value);
    void appendKey(double time, const Quaternion& value, bool updateCurve = true);
    void appendKey(const Quaternion& value, bool updateCurve = true);
    void deleteKey(int keyID);
    Quaternion getKey(int keyID);
    int getNumKeys() const;
    void cacheCurve();

    int getNumCurveSegments() const;
    Quaternion getCurvePoint(int i) const;

    void clear();
    double getDuration() const;
    double getNormalizedTime(double t) const; // takes a time t and returns a fraction

protected:

    void interpolateLinear();
    void interpolateCubic();

protected:
    double mDt;
    bool mLooping;
    std::vector<Key> mKeys;
    std::vector<Quaternion> mCachedCurve;
    InterpolationType mType;
};

#endif