#ifndef ASplineVec3_H_
#define ASplineVec3_H_

#include "aVector.h"
#include <map>
#include <vector>

class AInterpolatorVec3;

// class for managing keys, control points, and curves
class ASplineVec3 
{
public:
	enum InterpolationType { LINEAR, CUBIC_BERNSTEIN, CUBIC_CASTELJAU, CUBIC_MATRIX, CUBIC_HERMITE, CUBIC_BSPLINE };
    typedef std::pair<double, vec3> Key;

public:
    ASplineVec3();
    virtual ~ASplineVec3();

    void setLooping(bool loop);
    bool getLooping() const;

    void setFramerate(double fps);
    double getFramerate() const;

    void setInterpolationType(InterpolationType type);
    InterpolationType getInterpolationType() const;

    vec3 getValue(double t);

    void editControlPoint(int ctrlPointID, const vec3& value);
    void appendKey(double time, const vec3& value, bool updateCurve = true);
    void editKey(int keyID, const vec3& value); 
    void appendKey(const vec3& value, bool updateCurve = true);
    void deleteKey(int keyID);
    vec3 getKey(int keyID);
    vec3 getControlPoint(int ID);
    int getNumControlPoints() const;
    int getNumKeys() const;

    int getNumCurveSegments() const;
    vec3 getCurvePoint(int i) const;

    void clear();
    double getDuration() const;
    double getNormalizedTime(double t) const; // takes a time t and returns a fraction

    // Update curve -- by default, these do not need to be called manually
    void cacheCurve();
    void computeControlPoints();

protected:
    bool mLooping;
    AInterpolatorVec3* mInterpolator;
    std::vector<Key> mKeys;
    std::vector<vec3> mCtrlPoints;
    std::vector<vec3> mCachedCurve;
    vec3 mStartPoint, mEndPoint; // for controlling end point behavior
};

// class for implementing different interpolation algorithms
class AInterpolatorVec3
{
public:
    virtual ~AInterpolatorVec3() {}
    ASplineVec3::InterpolationType getType() const { return mType;  }

    // Given an ordered list of keys (<time,vec3> tuples) and control points, fill the curve
    virtual void interpolate(
        const std::vector<ASplineVec3::Key>& keys, 
        const std::vector<vec3>& ctrlPoints, 
        std::vector<vec3>& curve);

    // Given an ordered list of keys, compute corresponding control points
    // The start and end points are additionally set to specify the behavior at the endpoints
    virtual void computeControlPoints(
        const std::vector<ASplineVec3::Key>& keys, 
        std::vector<vec3>& ctrlPoints, 
        vec3& startPt, vec3& endPt) {}

    // The framerate determines the number of samples between each key
    void setFramerate(double fps);
    double getFramerate() const;
    double getDeltaTime() const; 

protected:
    AInterpolatorVec3(ASplineVec3::InterpolationType t);

    // Given keys, control points, current segment start index, and the time, compute an interpolated value
    // the time is the fraction between keys[segment] and keys[segment+1]
    // you can assume that segment+1 is a valid index
    virtual vec3 interpolateSegment(
        const std::vector<ASplineVec3::Key>& keys, 
        const std::vector<vec3>& ctrlPoints, 
        int segment, double u) = 0;

protected:
    ASplineVec3::InterpolationType mType;
    double mDt;
};

class ALinearInterpolatorVec3 : public AInterpolatorVec3
{
public:
    ALinearInterpolatorVec3() : AInterpolatorVec3(ASplineVec3::LINEAR) {}
    virtual vec3 interpolateSegment(
        const std::vector<ASplineVec3::Key>& keys, 
        const std::vector<vec3>& ctrlPoints, 
        int segment, double u);
};

class ACubicInterpolatorVec3 : public AInterpolatorVec3
{
public:
    virtual ~ACubicInterpolatorVec3() {}
    virtual void computeControlPoints(
        const std::vector<ASplineVec3::Key>& keys, 
        std::vector<vec3>& ctrlPoints, 
        vec3& startPt, vec3& endPt);

protected:
    ACubicInterpolatorVec3(ASplineVec3::InterpolationType t) : AInterpolatorVec3(t) {}
};

class ABernsteinInterpolatorVec3 : public ACubicInterpolatorVec3
{
public:
    ABernsteinInterpolatorVec3() : ACubicInterpolatorVec3(ASplineVec3::CUBIC_BERNSTEIN) {}
    virtual vec3 interpolateSegment(
        const std::vector<ASplineVec3::Key>& keys, 
        const std::vector<vec3>& ctrlPoints, 
        int segment, double u);
};

class ACasteljauInterpolatorVec3 : public ACubicInterpolatorVec3
{
public:
    ACasteljauInterpolatorVec3() : ACubicInterpolatorVec3(ASplineVec3::CUBIC_CASTELJAU) {}
    virtual vec3 interpolateSegment(
        const std::vector<ASplineVec3::Key>& keys, 
        const std::vector<vec3>& ctrlPoints, 
        int segment, double u);
};

class AMatrixInterpolatorVec3 : public ACubicInterpolatorVec3
{
public:
    AMatrixInterpolatorVec3() : ACubicInterpolatorVec3(ASplineVec3::CUBIC_MATRIX) {}
    virtual vec3 interpolateSegment(
        const std::vector<ASplineVec3::Key>& keys, 
        const std::vector<vec3>& ctrlPoints, 
        int segment, double u);
};

class AHermiteInterpolatorVec3 : public ACubicInterpolatorVec3
{
public:
    AHermiteInterpolatorVec3() : ACubicInterpolatorVec3(ASplineVec3::CUBIC_HERMITE), mClampedEndpoints(false) {}

    virtual vec3 interpolateSegment(
        const std::vector<ASplineVec3::Key>& keys, 
        const std::vector<vec3>& ctrlPoints, 
        int segment, double u);

    virtual void computeControlPoints(
        const std::vector<ASplineVec3::Key>& keys, 
        std::vector<vec3>& ctrlPoints, 
        vec3& startPt, vec3& endPt);

protected:
    bool mClampedEndpoints;
};

class ABSplineInterpolatorVec3 : public ACubicInterpolatorVec3
{
public:
    ABSplineInterpolatorVec3() : ACubicInterpolatorVec3(ASplineVec3::CUBIC_BSPLINE) {}

    virtual vec3 interpolateSegment(
        const std::vector<ASplineVec3::Key>& keys, 
        const std::vector<vec3>& ctrlPoints, 
        int segment, double u);

    virtual void computeControlPoints(
        const std::vector<ASplineVec3::Key>& keys, 
        std::vector<vec3>& ctrlPoints, 
        vec3& startPt, vec3& endPt);

protected:
    std::vector<double> mKnots;
};


#endif