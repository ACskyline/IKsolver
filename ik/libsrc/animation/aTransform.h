#ifndef aTransform_H_
#define aTransform_H_

#include "aRotation.h"
#include "aVector.h"

// class for implementing the equivalent of homogeneous transformations
class ATransform
{
public:
    ATransform();
    ATransform(const mat3& Rrot, const vec3& dtrans);
    ATransform(const ATransform& transform);
    ATransform& operator = (const ATransform& source); 

    ATransform Inverse() const;
    void WriteToGLMatrix(float* m);						// turn rotational data into 4x4 opengl matrix 
    void ReadFromGLMatrix(float* m);						// read rotational data from 4x4 opengl matrix

    vec3 RotTrans(const vec3& vecToTransform) const;    // transforms input vector using both rotation and translation components
	vec3 Rotate(const vec3& vecToTransform) const;      // transforms input vector using only rotation component
	vec3 Translate(const vec3& vecToTransform) const;   // transforms input vector using only translation component

    friend ATransform operator * (const ATransform& H1, const ATransform& H2);		// H1 * H2
    friend vec3 operator * (const ATransform& A, const vec3& v);	    // linear transform, assumes v is a vector
    friend std::ostream& operator << (std::ostream& s, const ATransform& v);

public:
    vec3 m_translation;   // equivalent to homogeneous transformation component m_d
    mat3 m_rotation;      // equivalent to homogeneous transformation component m_R
};

#endif

