#include "aSplineVec3.h"
#include <algorithm>
#include <Eigen/Dense>

#pragma warning(disable:4018)
#pragma warning(disable:4244)

ASplineVec3::ASplineVec3() : mInterpolator(new ALinearInterpolatorVec3())
{
}

ASplineVec3::~ASplineVec3()
{
	delete mInterpolator;
}

void ASplineVec3::setFramerate(double fps)
{
	mInterpolator->setFramerate(fps);
}

double ASplineVec3::getFramerate() const
{
	return mInterpolator->getFramerate();
}

void ASplineVec3::setLooping(bool loop)
{
	mLooping = loop;
}

bool ASplineVec3::getLooping() const
{
	return mLooping;
}

void ASplineVec3::setInterpolationType(ASplineVec3::InterpolationType type)
{
	double fps = getFramerate();

	delete mInterpolator;
	switch (type)
	{
	case LINEAR: mInterpolator = new ALinearInterpolatorVec3(); break;
	case CUBIC_BERNSTEIN: mInterpolator = new ABernsteinInterpolatorVec3(); break;
	case CUBIC_CASTELJAU: mInterpolator = new ACasteljauInterpolatorVec3(); break;
	case CUBIC_MATRIX: mInterpolator = new AMatrixInterpolatorVec3(); break;
	case CUBIC_HERMITE: mInterpolator = new AHermiteInterpolatorVec3(); break;
	case CUBIC_BSPLINE: mInterpolator = new ABSplineInterpolatorVec3(); break;
	};

	mInterpolator->setFramerate(fps);
	computeControlPoints();
	cacheCurve();
}

ASplineVec3::InterpolationType ASplineVec3::getInterpolationType() const
{
	return mInterpolator->getType();
}

void ASplineVec3::editKey(int keyID, const vec3& value)
{
	assert(keyID >= 0 && keyID < mKeys.size());
	mKeys[keyID].second = value;
	computeControlPoints();
	cacheCurve();
}

void ASplineVec3::editControlPoint(int ID, const vec3& value)
{
	assert(ID >= 0 && ID < mCtrlPoints.size() + 2);
	if (ID == 0)
	{
		mStartPoint = value;
		computeControlPoints();
	}
	else if (ID == mCtrlPoints.size() + 1)
	{
		mEndPoint = value;
		computeControlPoints();
	}
	else mCtrlPoints[ID - 1] = value;
	cacheCurve();
}

void ASplineVec3::appendKey(double time, const vec3& value, bool updateCurve)
{
	mKeys.push_back(Key(time, value));

	if (mKeys.size() >= 2)
	{
		int totalPoints = mKeys.size();

		//If there are more than 1 interpolation point, set up the 2 end points to help determine the curve.
		//They lie on the tangent of the first and last interpolation points.
		vec3 tmp = mKeys[0].second - mKeys[1].second;
		double n = tmp.Length();
		mStartPoint = mKeys[0].second + (tmp / n) * n * 0.25; // distance to endpoint is 25% of distance between first 2 points

		tmp = mKeys[totalPoints - 1].second - mKeys[totalPoints - 2].second;
		n = tmp.Length();
		mEndPoint = mKeys[totalPoints - 1].second + (tmp / n) * n * 0.25;
	}

	if (updateCurve)
	{
		computeControlPoints();
		cacheCurve();
	}
}

void ASplineVec3::appendKey(const vec3& value, bool updateCurve)
{
	if (mKeys.size() == 0)
	{
		appendKey(0, value, updateCurve);
	}
	else
	{
		double lastT = mKeys[mKeys.size() - 1].first;
		appendKey(lastT + 1, value, updateCurve);
	}
}

void ASplineVec3::deleteKey(int keyID)
{
	assert(keyID >= 0 && keyID < mKeys.size());
	mKeys.erase(mKeys.begin() + keyID);
	computeControlPoints();
	cacheCurve();
}

vec3 ASplineVec3::getKey(int keyID)
{
	assert(keyID >= 0 && keyID < mKeys.size());
	return mKeys[keyID].second;
}

int ASplineVec3::getNumKeys() const
{
	return mKeys.size();
}

vec3 ASplineVec3::getControlPoint(int ID)
{
	assert(ID >= 0 && ID < mCtrlPoints.size() + 2);
	if (ID == 0) return mStartPoint;
	else if (ID == mCtrlPoints.size() + 1) return mEndPoint;
	else return mCtrlPoints[ID - 1];
}

int ASplineVec3::getNumControlPoints() const
{
	return mCtrlPoints.size() + 2; // include endpoints
}

void ASplineVec3::clear()
{
	mKeys.clear();
}

double ASplineVec3::getDuration() const
{
	return mKeys[mKeys.size() - 1].first;
}

double ASplineVec3::getNormalizedTime(double t) const
{
	return (t / getDuration());
}

vec3 ASplineVec3::getValue(double t)
{
	if (mCachedCurve.size() == 0) return vec3();

	double dt = mInterpolator->getDeltaTime();
	int rawi = (int)(t / dt); // assumes uniform spacing
	int i = rawi % mCachedCurve.size();
	double frac = t - rawi*dt;
	int inext = i + 1;
	if (!mLooping) inext = std::min<int>(inext, mCachedCurve.size() - 1);
	else inext = inext % mCachedCurve.size();

	vec3 v1 = mCachedCurve[i];
	vec3 v2 = mCachedCurve[inext];
	vec3 v = v1*(1 - frac) + v2 * frac;
	return v;
}

void ASplineVec3::cacheCurve()
{
	mInterpolator->interpolate(mKeys, mCtrlPoints, mCachedCurve);
}

void ASplineVec3::computeControlPoints()
{
	mInterpolator->computeControlPoints(mKeys, mCtrlPoints, mStartPoint, mEndPoint);
}

int ASplineVec3::getNumCurveSegments() const
{
	return mCachedCurve.size();
}

vec3 ASplineVec3::getCurvePoint(int i) const
{
	return mCachedCurve[i];
}

//---------------------------------------------------------------------
AInterpolatorVec3::AInterpolatorVec3(ASplineVec3::InterpolationType t) : mDt(1.0 / 120.0), mType(t)
{
}

void AInterpolatorVec3::setFramerate(double fps)
{
	mDt = 1.0 / fps;
}

double AInterpolatorVec3::getFramerate() const
{
	return 1.0 / mDt;
}

double AInterpolatorVec3::getDeltaTime() const
{
	return mDt;
}

void AInterpolatorVec3::interpolate(const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints, std::vector<vec3>& curve)
{
	vec3 val = 0.0;
	double u = 0.0;

	curve.clear();

	int numSegments = keys.size() - 1;
	for (int segment = 0; segment < numSegments; segment++)
	{
		for (double t = keys[segment].first; t < keys[segment + 1].first - FLT_EPSILON; t += mDt)
		{

			// TODO: Compute u, fraction of duration between segment and segmentnext, for example,
			// u = 0.0 when t = keys[segment-1].first  
			// u = 1.0 when t = keys[segment].first
			u = (t - keys[segment].first) / (keys[segment + 1].first - keys[segment].first);

			val = interpolateSegment(keys, ctrlPoints, segment, u);
			curve.push_back(val);
		}
	}
	// add last point
	if (keys.size() > 1)
	{
		u = 1.0;
		val = interpolateSegment(keys, ctrlPoints, numSegments - 1, u);
		curve.push_back(val);
	}


}

vec3 ALinearInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double u)
{

	vec3 curveValue(0, 0, 0);
	vec3 key0 = keys[segment].second;
	vec3 key1 = keys[segment + 1].second;

	// TODO: 
	//Step 1: Create a Lerp helper function
	//Step 2: Linear interpolate between key0 and key1 so that u = 0 returns key0 and u = 1 returns key1

	curveValue = Lerp(u, key0, key1);

	return curveValue;
}

vec3 AInterpolatorVec3::Lerp(double u, const vec3 &key0, const vec3 &key1)
{
	return (1 - u)*key0 + u*key1;
}

vec3 ABernsteinInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double t)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	curveValue = (1 - t)*(1 - t)*(1 - t)*b0 + 3 * t*(1 - t)*(1 - t)*b1 + 3 * t*t*(1 - t)*b2 + t*t*t*b3;

	return curveValue;

}

vec3 ACasteljauInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double t)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  deCsteljau alogithm

	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	vec3 b01 = Lerp(t, b0, b1);
	vec3 b11 = Lerp(t, b1, b2);
	vec3 b21 = Lerp(t, b2, b3);
	vec3 b02 = Lerp(t, b01, b11);
	vec3 b12 = Lerp(t, b11, b21);
	vec3 b03 = Lerp(t, b02, b12);

	curveValue = b03;

	return curveValue;
}

vec3 AMatrixInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double t)
{
	vec3 b0;
	vec3 b1;
	vec3 b2;
	vec3 b3;
	vec3 curveValue(0, 0, 0);

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  matrix method f(u) = GMU
	// Hint: Using Eigen::MatrixXd data representations for a matrix operations

	b0 = ctrlPoints[4 * segment];
	b1 = ctrlPoints[4 * segment + 1];
	b2 = ctrlPoints[4 * segment + 2];
	b3 = ctrlPoints[4 * segment + 3];

	Eigen::MatrixXd G(3, 4);
	Eigen::MatrixXd M(4, 4);
	Eigen::MatrixXd U(4, 1);
	Eigen::MatrixXd F(3, 1);

	G << b0[0], b1[0], b2[0], b3[0],
		b0[1], b1[1], b2[1], b3[1],
		b0[2], b1[2], b2[2], b3[2];

	M << 1, -3, 3, -1,
		0, 3, -6, 3,
		0, 0, 3, -3,
		0, 0, 0, 1;

	U << 1,
		t,
		t*t,
		t*t*t;

	F = G * M * U;

	curveValue = vec3(F(0, 0), F(1, 0), F(2, 0));

	return curveValue;
}

vec3 AHermiteInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double t)
{

	vec3 p0 = keys[segment].second;
	vec3 p1 = keys[segment + 1].second;
	vec3 q0 = ctrlPoints[segment]; // slope at p0
	vec3 q1 = ctrlPoints[segment + 1]; // slope at p1
	vec3 curveValue(0, 0, 0);

	// TODO: Compute the interpolated value h(u) using a cubic Hermite polynomial  

	curveValue = (1 - 3 * t*t + 2 * t*t*t) * p0 + (3 * t*t - 2 * t*t*t) * p1 + (t*t*t - 2 * t*t + t)*q0 + (t*t*t - t*t)*q1;

	return curveValue;
}

vec3 ABSplineInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double t)
{
	vec3 curveValue(0, 0, 0);

	// Hint: Create a recursive helper function N(knots,n,j,t) to calculate BSpline basis function values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = curve interval on knot vector in which to interpolate
	//     t = time value	

	// Step 1: determine the index j

	std::vector<double> knots;
	knots.resize(keys.size() + 2 * 3);
	for (int i = 0; i < keys.size(); i++)
	{
		knots[i + 3] = keys[i].first;
	}

	knots[2] = knots[3] - (knots[4] - knots[3]);
	knots[1] = knots[3] - 2 * (knots[4] - knots[3]);
	knots[0] = knots[3] - 3 * (knots[4] - knots[3]);

	knots[knots.size() - 3] = knots[knots.size() - 4] + (knots[knots.size() - 4] - knots[knots.size() - 5]);
	knots[knots.size() - 2] = knots[knots.size() - 4] + 2 * (knots[knots.size() - 4] - knots[knots.size() - 5]);
	knots[knots.size() - 1] = knots[knots.size() - 4] + 3 * (knots[knots.size() - 4] - knots[knots.size() - 5]);

	int j = segment + 3;

	// Step 2: compute the n nonzero Bspline Basis functions N given j

	t = keys[segment].first + t*(keys[segment + 1].first - keys[segment].first);

	double N0 = N(knots, 3, j - 3, t);
	double N1 = N(knots, 3, j - 2, t);
	double N2 = N(knots, 3, j - 1, t);
	double N3 = N(knots, 3, j, t);

	// Step 3: get the corresponding control points from the ctrlPoints vector

	vec3 b0 = ctrlPoints[j - 3];
	vec3 b1 = ctrlPoints[j - 2];
	vec3 b2 = ctrlPoints[j - 1];
	vec3 b3 = ctrlPoints[j];

	// Step 4: compute the Bspline curveValue at time t

	curveValue = b0 * N0 + b1 * N1 + b2 * N2 + b3 * N3;

	return curveValue;
}

double ABSplineInterpolatorVec3::N(const std::vector<double> &knots,
	int n,
	int j,
	double t)
{
	if (n == 0)
	{
		if (t >= knots[j] && t < knots[j + 1])
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return (t - knots[j]) / (knots[j + n] - knots[j]) * N(knots, n - 1, j, t) + (knots[j + n + 1] - t) / (knots[j + n + 1] - knots[j + 1]) * N(knots, n - 1, j + 1, t);
	}
}

void ACubicInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys,
	std::vector<vec3>& ctrlPoints,
	vec3& startPoint, vec3& endPoint)
{
	ctrlPoints.clear();
	if (keys.size() <= 1) return;

	for (int i = 1; i < keys.size(); i++)
	{
		vec3 b0, b1, b2, b3;

		// TODO: compute b0, b1, b2, b3

		//using phantom points
		b0 = keys[i - 1].second;
		b1 = i - 2 >= 0 ? keys[i - 1].second + (keys[i].second - keys[i - 2].second) / 6.0 : keys[i - 1].second + (keys[i].second - startPoint) / 6.0;
		b2 = i + 1 <= keys.size() - 1 ? keys[i].second - (keys[i + 1].second - keys[i - 1].second) / 6.0 : keys[i].second - (endPoint - keys[i - 1].second) / 6.0;
		b3 = keys[i].second;

		ctrlPoints.push_back(b0);
		ctrlPoints.push_back(b1);
		ctrlPoints.push_back(b2);
		ctrlPoints.push_back(b3);
	}
}

void AHermiteInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys,
	std::vector<vec3>& ctrlPoints,
	vec3& startPoint, vec3& endPoint)
{
	ctrlPoints.clear();
	if (keys.size() <= 1) return;

	int numKeys = keys.size();


	// TODO: 
	// For each key point pi, compute the corresonding value of the slope pi_prime.
	// Hints: Using Eigen::MatrixXd for a matrix data structures, 
	// this can be accomplished by solving the system of equations AC=D for C.
	// Don't forget to save the values computed for C in ctrlPoints
	// For clamped endpoint conditions, set 1st derivative at first and last points (p0 and pm) to s0 and s1, respectively
	// For natural endpoints, set 2nd derivative at first and last points (p0 and pm) equal to 0

	// Step 1: Initialize A
	// Step 2: Initialize D
	// Step 3: Solve AC=D for C
	// Step 4: Save control points in ctrlPoints


	Eigen::MatrixXd A(numKeys, numKeys);
	A.setZero();
	A(0, 0) = 2;
	A(0, 1) = 1;
	for (int i = 1; i <= numKeys - 2; i++)
	{
		A(i, i - 1) = 1;
		A(i, i) = 4;
		A(i, i + 1) = 1;
	}
	A(numKeys - 1, numKeys - 2) = 1;
	A(numKeys - 1, numKeys - 1) = 2;

	//natural spline
	Eigen::MatrixXd D(numKeys, 3);
	D(0, 0) = 3 * (keys[1].second[0] - keys[0].second[0]);
	D(0, 1) = 3 * (keys[1].second[1] - keys[0].second[1]);
	D(0, 2) = 3 * (keys[1].second[2] - keys[0].second[2]);
	for (int i = 1; i <= numKeys - 2; i++)
	{
		D(i, 0) = 3 * (keys[i + 1].second[0] - keys[i - 1].second[0]);
		D(i, 1) = 3 * (keys[i + 1].second[1] - keys[i - 1].second[1]);
		D(i, 2) = 3 * (keys[i + 1].second[2] - keys[i - 1].second[2]);
	}
	D(numKeys - 1, 0) = 3 * (keys[numKeys - 1].second[0] - keys[numKeys - 2].second[0]);
	D(numKeys - 1, 1) = 3 * (keys[numKeys - 1].second[1] - keys[numKeys - 2].second[1]);
	D(numKeys - 1, 2) = 3 * (keys[numKeys - 1].second[2] - keys[numKeys - 2].second[2]);

	Eigen::MatrixXd C(numKeys, 3);

	C = A.inverse()*D;

	for (int i = 0; i < numKeys; i++)
	{
		vec3 b(C(i, 0), C(i, 1), C(i, 2));
		ctrlPoints.push_back(b);
	}

}

void ABSplineInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys,
	std::vector<vec3>& ctrlPoints,
	vec3& startPt, vec3& endPt)
{
	ctrlPoints.clear();
	if (keys.size() <= 1) return;

	// TODO: c
	// Hints: 
	// 1. use Eigen::MatrixXd to calculate the control points by solving the system of equations AC=D for C

	// 2. Create a recursive helper function dN(knots,n,t,l) to calculate derivative BSpline values at t, where
	//     knots = knot array
	//	   n = degree of the spline curves (n =3 for cubic)
	//     j = interval on knot vector in which to interpolate
	//     t = time value
	//     l = derivative (l = 1 => 1st derivative)

	// Step 1: Calculate knot vector using a uniform BSpline
	//         (assune knots are evenly spaced 1 apart and the start knot is at time = 0.0)

	std::vector<double> knots;
	knots.resize(keys.size() + 2 * 3);
	for (int i = 0; i < keys.size(); i++)
	{
		knots[i + 3] = keys[i].first;
	}

	knots[2] = knots[3] - (knots[4] - knots[3]);
	knots[1] = knots[3] - 2 * (knots[4] - knots[3]);
	knots[0] = knots[3] - 3 * (knots[4] - knots[3]);

	knots[knots.size() - 3] = knots[knots.size() - 4] + (knots[knots.size() - 4] - knots[knots.size() - 5]);
	knots[knots.size() - 2] = knots[knots.size() - 4] + 2 * (knots[knots.size() - 4] - knots[knots.size() - 5]);
	knots[knots.size() - 1] = knots[knots.size() - 4] + 3 * (knots[knots.size() - 4] - knots[knots.size() - 5]);

	// Step 2: Calculate A matrix  for a natural BSpline
	//         (Set 2nd derivative at t0 and tm to zero, where tm is the last point knot; m = #segments)

	Eigen::MatrixXd A(keys.size() + 2, keys.size() + 2);
	A.setZero();

	A(0, 0) = dN(knots, 3, 0, keys[0].first, 2);
	A(0, 1) = dN(knots, 3, 1, keys[0].first, 2);
	A(0, 2) = dN(knots, 3, 2, keys[0].first, 2);
	A(0, 3) = dN(knots, 3, 3, keys[0].first, 2);
	for (int i = 1; i < keys.size(); i++)
	{
		A(i, i - 1) = N(knots, 3, i - 1, keys[i - 1].first);
		A(i, i) = N(knots, 3, i, keys[i - 1].first);
		A(i, i + 1) = N(knots, 3, i + 1, keys[i - 1].first);
		A(i, i + 2) = N(knots, 3, i + 2, keys[i - 1].first);
	}
	A(keys.size(), keys.size() - 2) = N(knots, 3, keys.size() - 2, keys[keys.size() - 1].first);
	A(keys.size(), keys.size() - 1) = N(knots, 3, keys.size() - 1, keys[keys.size() - 1].first);
	A(keys.size(), keys.size()) = N(knots, 3, keys.size(), keys[keys.size() - 1].first);
	A(keys.size(), keys.size() + 1) = N(knots, 3, keys.size() + 1, keys[keys.size() - 1].first);

	A(keys.size() + 1, keys.size() - 2) = dN(knots, 3, keys.size() - 2, keys[keys.size() - 1].first, 2);
	A(keys.size() + 1, keys.size() - 1) = dN(knots, 3, keys.size() - 1, keys[keys.size() - 1].first, 2);
	A(keys.size() + 1, keys.size()) = dN(knots, 3, keys.size(), keys[keys.size() - 1].first, 2);
	A(keys.size() + 1, keys.size() + 1) = dN(knots, 3, keys.size() + 1, keys[keys.size() - 1].first, 2);

	// Step 3: Calculate  D matrix composed of our target points to interpolate

	Eigen::MatrixXd D(keys.size() + 2, 3);

	D(0, 0) = 0;
	D(0, 1) = 0;
	D(0, 2) = 0;
	for (int i = 1; i < keys.size() + 1; i++)
	{
		D(i, 0) = keys[i - 1].second[0];
		D(i, 1) = keys[i - 1].second[1];
		D(i, 2) = keys[i - 1].second[2];
	}
	D(keys.size() + 1, 0) = 0;
	D(keys.size() + 1, 1) = 0;
	D(keys.size() + 1, 2) = 0;

	// Step 4: Solve AC=D for C 

	Eigen::MatrixXd C(keys.size() + 2, 3);
	C = A.inverse() * D;

	// Step 5: save control points in ctrlPoints
	for (int i = 0; i < keys.size() + 2; i++)
	{
		vec3 b = vec3(C(i, 0), C(i, 1), C(i, 2));
		ctrlPoints.push_back(b);
	}

	//std::cout << A << std::endl;
	//std::cout << D << std::endl;
	//std::cout << C << std::endl;
}

double ABSplineInterpolatorVec3::dN(const std::vector<double> &knots,
	int n,
	int j,
	double t,
	int l)
{
	if (l == 0)
	{
		return N(knots, n, j, t);
	}
	else
	{
		return n*(1 / (knots[j + n] - knots[j])*dN(knots, n - 1, j, t, l - 1) - 1 / (knots[j + n + 1] - knots[j + 1]) * dN(knots, n - 1, j + 1, t, l - 1));
	}
}