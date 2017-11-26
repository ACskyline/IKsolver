#ifndef aRotation_H_
#define aRotation_H_

#include <iostream>
#include "aVector.h"

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192313216916398
#endif

const double Rad2Deg = (180.0f / M_PI);			// Rad to Degree
const double Deg2Rad = (M_PI / 180.0f);			// Degree to Rad

class quat;
class mat3
{
protected:

    vec3 mM[3];

public:
	enum RotOrder { ZYX, XYZ, YZX, XZY, YXZ, ZXY };

    // Constructors
    mat3();
    mat3(const vec3& v0, const vec3& v1, const vec3& v2);
    mat3(double d);
    mat3(const mat3& m);

    // Static functions
	void Zero(); 
	void Identity();

	static mat3 Rotation3D(const vec3& axis, double angleRad);
	static mat3 Rotation3D(const int Axis, double angleRad);

    static mat3 FromToRotation(const vec3& fromDir, const vec3& toDir);
	bool ToEulerAngles(RotOrder order, vec3& anglesRad) const;
	mat3 FromEulerAngles(RotOrder order, const vec3& anglesRad);

    // Conversion with Quaternion
    quat ToQuaternion() const;
    void FromQuaternion(const quat& q);
    void ToAxisAngle(vec3& axis, double& angleRad) const;
    void FromAxisAngle(const vec3& axis, double angleRad);

    // Assignment operators
    mat3& operator = ( const mat3& m );	    // assignment of a mat3
    mat3& operator += ( const mat3& m );	    // incrementation by a mat3
    mat3& operator -= ( const mat3& m );	    // decrementation by a mat3
    mat3& operator *= ( double d );	    // multiplication by a constant
    mat3& operator /= ( double d );	    // division by a constant
    vec3& operator [] ( int i);					// indexing
    const vec3& operator [] ( int i) const;		// read-only indexing

    // special functions
    mat3 Transpose() const;								// transpose
    mat3 Inverse() const;								// inverse
    void WriteToGLMatrix(float* m) const;							// turn rotational data into 4x4 opengl matrix with zero translation
    void ReadFromGLMatrix(float* m);						// read rotational data from 4x4 opengl matrix
    bool Reorthogonalize();								// Gram-Schmidt orthogonalization
    vec3 GetRow(unsigned int axis) const;	// get a particular row
    vec3 GetCol(unsigned int axis) const;	// get a particular col
    void SetRow(unsigned int axis, const vec3& rowVec);	// set a particular row
    void SetCol(unsigned int axis, const vec3& colVec);	// set a particular col
    vec3 GetYawPitchRoll(unsigned int leftAxis, unsigned int upAixs, unsigned int frontAxis) const;

    // friends
     friend mat3 operator - (const mat3& a);						// -m1
     friend mat3 operator + (const mat3& a, const mat3& b);	    // m1 + m2
     friend mat3 operator - (const mat3& a, const mat3& b);	    // m1 - m2
     friend mat3 operator * (const mat3& a, const mat3& b);		// m1 * m2
     friend mat3 operator * (const mat3& a, double d);	    // m1 * 3.0
     friend mat3 operator * (double d, const mat3& a);	    // 3.0 * m1
     friend mat3 operator / (const mat3& a, double d);	    // m1 / 3.0
     friend int operator == (const mat3& a, const mat3& b);	    // m1 == m2 ?
     friend int operator != (const mat3& a, const mat3& b);	    // m1 != m2 ?
     friend void Swap(mat3& a, mat3& b);			    // swap m1 & m2

     friend std::istream& operator >> (std::istream& s, mat3& v);
     friend std::ostream& operator << (std::ostream& s, const mat3& v);

    // necessary friend declarations
    friend vec3 operator * (const mat3& a, const vec3& v);	    // linear transform
    friend  mat3 operator * (const mat3& a, const mat3& b);	// matrix 3 product
};

const mat3 identity3D(axisX, axisY, axisZ);
const mat3 zero3D(vec3Zero, vec3Zero, vec3Zero);

class  quat
{
protected:

    double mQ[4];

public:

    // Constructors
    quat();
    quat(double w, double x, double y, double z);
    quat(const quat& q);

    // Static functions
    static double Dot(const quat& q0, const quat& q1);
    static double Distance(const quat& q0, const quat& q1); // returns angle between in radians
    static quat Exp(const quat& q);
    static quat Log(const quat& q);
    static quat UnitInverse(const quat& q);

   // interpolation functions
	static quat SDouble(const quat& a, const quat& b);
	static quat SBisect(const quat& a, const quat& b);
	static quat Slerp(const quat& q0, const quat& q1, double u);
	static void ScubicControlPts(const quat& q_1, const quat& q0, const quat& q1, const quat& q2, quat& b1, quat& b2);
	static quat Scubic(const quat& q0, const quat& b1, const quat& b2, const quat& q1, double u);
	static quat Intermediate(const quat& q0, const quat& q1, const quat& q2);
	static quat Squad(const quat& q0, const quat& a, const quat& b, const quat& q1, double u);
   


    // Conversion functions
    void ToAxisAngle (vec3& axis, double& angleRad) const;
    void FromAxisAngle (const vec3& axis, double angleRad);
	static quat ProjectToAxis(const quat& q, vec3& axis);

    vec3 ToExpMap() const;
    void FromExpMap(const vec3& expmap);

    mat3 ToRotation () const;
    void FromRotation (const mat3& rot);

    // Assignment operators
    quat& operator = (const quat& q);	// assignment of a quaternion
    quat& operator += (const quat& q);	// summation with a quaternion
    quat& operator -= (const quat& q);	// subtraction with a quaternion
    quat& operator *= (const quat& q);	// multiplication by a quaternion
    quat& operator *= (double d);		// multiplication by a scalar
    quat& operator /= (double d);		// division by a scalar


    // Indexing
    double& W();
    double W() const;
    double& X();
    double X() const;
    double& Y();
    double Y() const;
    double& Z();
    double Z() const;
    double& operator[](int i); // carefull using these, W is last component!
    double operator[](int i) const;

    // Friends
     friend quat operator - (const quat& q);							// -q
     friend quat operator + (const quat& q0, const quat& q1);	    // q0 + q1
     friend quat operator - (const quat& q0, const quat& q1);	// q0 - q1
     friend quat operator * (const quat& q, double d);			// q * 3.0
     friend quat operator * (double d, const quat& q);			// 3.0 * v
     friend quat operator * (const quat& q0, const quat& q1);  // q0 * q1
     friend quat operator / (const quat& q, double d);			// q / 3.0
     friend bool operator == (const quat& q0, const quat& q1);		// q0 == q1 ?
     friend bool operator != (const quat& q0, const quat& q1);		// q0 != q1 ?

     friend std::istream& operator >> (std::istream& s, quat& v);
     friend std::ostream& operator << (std::ostream& s, const quat& v);

    // Special functions
    double Length() const;
    double SqrLength() const;
    quat& Normalize();
    quat Conjugate() const;
    quat Inverse() const;
    void Zero();

    friend mat3;
};


#endif

