#ifndef ASplineQuat_H_
#define ASplineQuat_H_

#include "aRotation.h"
#include <map>
#include <vector>

class ASplineQuat 
{
public:
    enum InterpolationType { LINEAR, CUBIC };
    typedef std::pair<double, quat> Key;

public:
    ASplineQuat();
    virtual ~ASplineQuat();

    void setLooping(bool loop);
    bool getLooping() const;

    void setFramerate(double fps);
    double getFramerate() const;

    void setInterpolationType(InterpolationType type);
    InterpolationType getInterpolationType() const;

    void editKey(int keyID, const quat& value);
    void appendKey(double time, const quat& value, bool updateCurve = true);
    void appendKey(const quat& value, bool updateCurve = true);
    void deleteKey(int keyID);
    quat getKey(int keyID);
    int getNumKeys() const;

    void cacheCurve();

    int getNumCurveSegments() const;
	int getCurveSegment(double t);
	quat getCachedValue(double t);
	quat getCubicValue(double t);
	quat getLinearValue(double t);
	void computeControlPoints(quat& startQuat, quat& endQuat);

    void clear();
    double getDuration() const;
    double getNormalizedTime(double t) const; // takes a time t and returns a value u that ranges from 0 to 1

protected:

    void createSplineCurveLinear();
    void createSplineCurveCubic();


protected:
    double mDt;
    bool mLooping;
    std::vector<Key> mKeys;
    std::vector<quat> mCachedCurve;
	std::vector<quat> mCtrlPoints;
    InterpolationType mType;
};

#endif