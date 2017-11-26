#include "aTransform.h"
#pragma warning(disable : 4244)

ATransform::ATransform() : m_rotation(identity3D), m_translation(vec3Zero)
{
}

ATransform::ATransform(const mat3& rot, const vec3& offset) : m_rotation(rot), m_translation(offset)
{
}

ATransform::ATransform(const ATransform& m)
{
    *this = m;
}

// Assignment operators
ATransform& ATransform::operator = (const ATransform& orig)
{
    if (&orig == this)
    {
        return *this;
    }
    m_rotation = orig.m_rotation;
    m_translation = orig.m_translation;
    return *this;
}


ATransform ATransform::Inverse() const
{
	ATransform result;

	// TODO: compute the inverse of a transform given the current rotation and translation components

	result.m_rotation = m_rotation.Inverse();
	result.m_translation = -m_translation;
 

	return result;
}


vec3 ATransform::RotTrans(const vec3& vecToTransform) const
{
	vec3 result(0.0);

	// TODO: Transform the input vector based on this transform's rotation and translation components


	result = Rotate(Translate(vecToTransform));
 

	return result;

}

vec3 ATransform::Rotate(const vec3& vecToTransform) const
{
	vec3 result(0.0);

	// TODO: Transform the input direction based on this transform's rotation component


	result = m_rotation * vecToTransform;
 

	return result;
}

vec3 ATransform::Translate(const vec3& vecToTransform) const
{
	vec3 result(0.0);

	// TODO: Transform the input vector based on this transform's translation component


	result = vecToTransform + m_translation;
 

	return result;

}

ATransform operator * (const ATransform& H1, const ATransform& H2)
{
	ATransform result;

	// TODO: implement the equivalent of multiplying  H1 and H2 transformation matrices and return the result

	result.m_rotation = H1.m_rotation * H2.m_rotation;
	result.m_translation = H1.m_translation + H1.m_rotation*H2.m_translation;

 

	return result;
}

vec3 operator * (const ATransform& A, const vec3& v)
{
	return A.RotTrans(v);
}

void ATransform::WriteToGLMatrix(float* m)
{
	m[0] = m_rotation[0][0]; m[4] = m_rotation[0][1]; m[8] = m_rotation[0][2];  m[12] = m_translation[0];
	m[1] = m_rotation[1][0]; m[5] = m_rotation[1][1]; m[9] = m_rotation[1][2];  m[13] = m_translation[1];
	m[2] = m_rotation[2][0]; m[6] = m_rotation[2][1]; m[10] = m_rotation[2][2]; m[14] = m_translation[2];
	m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
}

void ATransform::ReadFromGLMatrix(float* m)
{
	m_rotation[0][0] = m[0]; m_rotation[0][1] = m[4]; m_rotation[0][2] = m[8];  m_translation[0] = m[12];
	m_rotation[1][0] = m[1]; m_rotation[1][1] = m[5]; m_rotation[1][2] = m[9];  m_translation[1] = m[13];
	m_rotation[2][0] = m[2]; m_rotation[2][1] = m[6]; m_rotation[2][2] = m[10]; m_translation[2] = m[14];
}


std::ostream& operator << (std::ostream& s, const ATransform& t)
{
    vec3 anglesRad;
    t.m_rotation.ToEulerAngles(mat3::ZXY, anglesRad);
    s << "R: " << anglesRad << " T: " << t.m_translation << " ";
    return s;
}





