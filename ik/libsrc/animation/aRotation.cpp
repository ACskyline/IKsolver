#include "aRotation.h"
#include <algorithm>

#define IS_ZERO(x) ((x)>-EPSILON && (x)<EPSILON)
#define SGN(x) (x>=0?1.0:-1.0)

#ifndef EPSILON
#define EPSILON 0.001f
#endif

enum { VX, VY, VZ, VW };
#pragma warning(disable : 4244)

// CONSTRUCTORS
mat3::mat3() 
{
    mM[0] = vec3(0.0f,0.0f,0.0f);
    mM[1] = mM[2] = mM[0];
}

mat3::mat3(const vec3& v0, const vec3& v1, const vec3& v2)
{ 
    mM[0] = v0; mM[1] = v1; mM[2] = v2; 
}

mat3::mat3(double d)
{ 
    mM[0] = mM[1] = mM[2] = vec3(d); 
}

mat3::mat3(const mat3& m)
{ 
    mM[0] = m.mM[0]; mM[1] = m.mM[1]; mM[2] = m.mM[2]; 
}


 void mat3::Identity()
{
	 vec3 v0; // constructor intializes all values to zero
	 mM[0] = v0; mM[1] = v0; mM[2] = v0;
	 for (int i = 0; i < 3; i++)
		 mM[i][i] = 1.0;
}
 void mat3::Zero()
 {
	 vec3 v0; // constructor intializes all values to zero
	 mM[0] = v0; mM[1] = v0; mM[2] = v0;
 }

mat3 mat3::Rotation3D(const vec3& axis, double angleRad)
{
	double c = cos(angleRad), s = sin(angleRad), t = 1.0f - c;
	vec3 Axis = axis;
	Axis.Normalize();
	return mat3(vec3(t * Axis[VX] * Axis[VX] + c,
		t * Axis[VX] * Axis[VY] - s * Axis[VZ],
		t * Axis[VX] * Axis[VZ] + s * Axis[VY]),
		vec3(t * Axis[VX] * Axis[VY] + s * Axis[VZ],
		t * Axis[VY] * Axis[VY] + c,
		t * Axis[VY] * Axis[VZ] - s * Axis[VX]),
		vec3(t * Axis[VX] * Axis[VZ] - s * Axis[VY],
		t * Axis[VY] * Axis[VZ] + s * Axis[VX],
		t * Axis[VZ] * Axis[VZ] + c)
		);
}

mat3 mat3::Rotation3D(const int Axis, double angleRad)
{
	mat3 m;
	switch (Axis)
	{
	case VX: m = Rotation3D(axisX, angleRad);
		break;
	case VY: m = Rotation3D(axisY, angleRad);
		break;
	case VZ: m = Rotation3D(axisZ, angleRad);
		break;
	}
	return m;
}

bool mat3::ToEulerAngles(RotOrder order, vec3& angleRad) const
{
	bool result;
	switch (order)
	{
		case ZYX:
			angleRad[VY] = -asin(mM[2][0]);
			if (angleRad[VY] > -M_PI_2 + EPSILON)
			{
				if (angleRad[VY] < M_PI_2 - EPSILON)
				{
					angleRad[VZ] = atan2(mM[1][0], mM[0][0]);
					angleRad[VX] = atan2(mM[2][1], mM[2][2]);
					result = true;
				}
				else
				{
					// WARNING.  Not a unique solution.
					angleRad[VX] = 0.0f;
					angleRad[VZ] = atan2(-mM[0][1], mM[0][2]);
					result = false;
				}
			}
			else
			{
				// WARNING.  Not a unique solution.
				angleRad[VX] = 0.0f;
				angleRad[VZ] = atan2(mM[0][1], mM[0][2]);
				result = false;
			}
			break;
		case XYZ:
			angleRad[VY] = asin(mM[0][2]);
			if (angleRad[VY] > -M_PI_2 + EPSILON)
			{
				if (angleRad[VY] < M_PI_2 - EPSILON)
				{
					angleRad[VX] = atan2(-mM[1][2], mM[2][2]);
					angleRad[VZ] = atan2(-mM[0][1], mM[0][0]);
					result = true;
				}
				else
				{
					// WARNING.  Not a unique solution.
					angleRad[VZ] = 0.0f;
					angleRad[VX] = atan2(mM[1][0], mM[1][1]);
					result = false;
				}
			}
			else
			{
				// WARNING.  Not a unique solution.
				angleRad[VZ] = 0.0f;  // any angle works
				angleRad[VX] = -atan2(mM[1][0], mM[1][1]);
				result = false;
			}
			break;
		case YZX:
			angleRad[VZ] = asin(mM[1][0]);
			if (angleRad[VZ] > -M_PI_2 + EPSILON)
			{
				if (angleRad[VZ] < M_PI_2 - EPSILON)
				{
					angleRad[VY] = atan2(-mM[2][0], mM[0][0]);
					angleRad[VX] = atan2(-mM[1][2], mM[1][1]);
					result = true;
				}
				else
				{
					// WARNING.  Not a unique solution.
					angleRad[VX] = 0.0f;
					angleRad[VY] = atan2(mM[2][1], mM[2][2]);
					result = false;
				}
			}
			else
			{
				// WARNING.  Not a unique solution.
				angleRad[VX] = 0.0f;
				angleRad[VY] = -atan2(mM[2][1], mM[2][2]);
				result = false;
			}

			break;
		case XZY:
			angleRad[VZ] = asin(-mM[0][1]);
			if (angleRad[VZ] > -M_PI_2 + EPSILON)
			{
				if (angleRad[VZ] < M_PI_2 - EPSILON)
				{
					angleRad[VX] = atan2(mM[2][1], mM[1][1]);
					angleRad[VY] = atan2(mM[0][2], mM[0][0]);
					result = true;
				}
				else
				{
					// WARNING.  Not a unique solution.
					angleRad[VY] = 0.0f;  // any angle works
					angleRad[VX] = atan2(mM[2][0], mM[2][2]);
					result = false;
				}
			}
			else
			{
				// WARNING.  Not a unique solution.
				angleRad[VY] = 0.0f;
				angleRad[VX] = -atan2(mM[2][0], mM[2][2]);
				result = false;
			}
			break;
		case ZXY:
			angleRad[VX] = asin(mM[2][1]);
			if (angleRad[VX] > -M_PI_2 + EPSILON)
			{
				if (angleRad[VX] < M_PI_2 - EPSILON)
				{
					angleRad[VZ] = atan2(-mM[0][1], mM[1][1]);
					angleRad[VY] = atan2(-mM[2][0], mM[2][2]);
					result = true;
				}
				else
				{
					// WARNING.  Not a unique solution.
					angleRad[VY] = 0.0f;
					angleRad[VZ] = atan2(mM[0][2], mM[0][0]);
					result = false;
				}
			}
			else
			{
				// WARNING.  Not a unique solution.
				angleRad[VY] = 0.0f;
				angleRad[VZ] = -atan2(mM[0][2], mM[0][0]);
				result = false;
			}

			break;

		case YXZ:
			//TODO: student implementation for computing Euler angles from a rotation matrix with an YXZ order of rotation goes here
			angleRad[VX] = asin(-mM[1][2]);
			if (angleRad[VX] > -M_PI_2 + EPSILON)
			{
				if (angleRad[VX] < M_PI_2 - EPSILON)
				{
					angleRad[VZ] = atan2(mM[1][0], mM[1][1]);
					angleRad[VY] = atan2(mM[0][2], mM[2][2]);
					result = true;
				}
				else
				{
					// WARNING.  Not a unique solution.
					angleRad[VZ] = 0.0f; 
					angleRad[VY] = acos(mM[0][0]);
					result = false;
				}
			}
			else
			{
				// WARNING.  Not a unique solution.
				angleRad[VZ] = 0.0f;
				angleRad[VY] = acos(mM[0][0]);
				result = false;
			}

			break;
	}
	return result;
}


mat3 mat3::FromEulerAngles(RotOrder order, const vec3& anglesRad)
{
	mat3 m;
	switch (order)
	{
	case ZYX:
		m =   mat3::Rotation3D(axisZ, anglesRad[VZ])
			* mat3::Rotation3D(axisY, anglesRad[VY])
			* mat3::Rotation3D(axisX, anglesRad[VX]);

		break;
	case XYZ:
		m =   mat3::Rotation3D(axisX, anglesRad[VX])
			* mat3::Rotation3D(axisY, anglesRad[VY])
			* mat3::Rotation3D(axisZ, anglesRad[VZ]);

		break;
	case YZX:
		m =   mat3::Rotation3D(axisY, anglesRad[VY])
			* mat3::Rotation3D(axisZ, anglesRad[VZ])
			* mat3::Rotation3D(axisX, anglesRad[VX]);

		break;
	case XZY:
		m =   mat3::Rotation3D(axisX, anglesRad[VX])
			* mat3::Rotation3D(axisZ, anglesRad[VZ])
			* mat3::Rotation3D(axisY, anglesRad[VY]);
		break;
	case ZXY:
		m =   mat3::Rotation3D(axisZ, anglesRad[VZ])
			* mat3::Rotation3D(axisX, anglesRad[VX])
			* mat3::Rotation3D(axisY, anglesRad[VY]);
		break;

	case YXZ:
		//TODO: student implementation for computing rotation matrix for YXZ order of rotation goes here
		m =   mat3::Rotation3D(axisY, anglesRad[VY])
			* mat3::Rotation3D(axisX, anglesRad[VX])
			* mat3::Rotation3D(axisZ, anglesRad[VZ]);
		break;


	}
	*this = m;
	return m;
}

bool mat3::Reorthogonalize()
{
    // Factor M = QR where Q is orthogonal and R is upper triangular.
    // Algorithm uses Gram-Schmidt orthogonalization (the QR algorithm).
    //
    // If M = [ m0 | m1 | m2 ] and Q = [ q0 | q1 | q2 ], then
    //
    //   q0 = m0/|m0|
    //   q1 = (m1-(q0*m1)q0)/|m1-(q0*m1)q0|
    //   q2 = (m2-(q0*m2)q0-(q1*m2)q1)/|m2-(q0*m2)q0-(q1*m2)q1|
    //
    // where |V| indicates length of vector V and A*B indicates dot
    // product of vectors A and B.  The matrix R has entries
    //
    //   r00 = q0*m0  r01 = q0*m1  r02 = q0*m2
    //   r10 = 0      r11 = q1*m1  r12 = q1*m2
    //   r20 = 0      r21 = 0      r22 = q2*m2
    //
    // The reorthogonalization replaces current matrix by computed Q.

    const double fEpsilon = 1e-05f;

    // unitize column 0
    double fLength = sqrt(mM[0][0] * mM[0][0] + mM[1][0] * mM[1][0] + mM[2][0] * mM[2][0]);
    if ( fLength < fEpsilon )
        return false;
    double fInvLength = 1.0f / fLength;
    mM[0][0] *= fInvLength;
    mM[1][0] *= fInvLength;
    mM[2][0] *= fInvLength;

    // project out column 0 from column 1
    double fDot = mM[0][0] * mM[0][1] + mM[1][0] * mM[1][1] + mM[2][0] * mM[2][1];
    mM[0][1] -= fDot * mM[0][0];
    mM[1][1] -= fDot * mM[1][0];
    mM[2][1] -= fDot * mM[2][0];

    // unitize column 1
    fLength = sqrt(mM[0][1] * mM[0][1] + mM[1][1] * mM[1][1] + mM[2][1] * mM[2][1]);
    if ( fLength < fEpsilon )
        return false;
    fInvLength = 1.0f/fLength;
    mM[0][1] *= fInvLength;
    mM[1][1] *= fInvLength;
    mM[2][1] *= fInvLength;

    // project out column 0 from column 2
    fDot = mM[0][0] * mM[0][2] + mM[1][0] * mM[1][2] + mM[2][0] * mM[2][2];
    mM[0][2] -= fDot * mM[0][0];
    mM[1][2] -= fDot * mM[1][0];
    mM[2][2] -= fDot * mM[2][0];

    // project out column 1 from column 2
    fDot = mM[0][1] * mM[0][2] + mM[1][1] * mM[1][2] + mM[2][1] * mM[2][2];
    mM[0][2] -= fDot * mM[0][1];
    mM[1][2] -= fDot * mM[1][1];
    mM[2][2] -= fDot * mM[2][1];

    // unitize column 2
    fLength = sqrt(mM[0][2] * mM[0][2] + mM[1][2] * mM[1][2] + mM[2][2] * mM[2][2]);
    if ( fLength < fEpsilon )
        return false;
    fInvLength = 1.0f / fLength;
    mM[0][2] *= fInvLength;
    mM[1][2] *= fInvLength;
    mM[2][2] *= fInvLength;

    return true;
}

// Conversion with Quaternion
quat mat3::ToQuaternion() const
{
    quat q;
    q.FromRotation(*this);
    return q;
}

void mat3::FromQuaternion(const quat& q)
{
    (*this) = q.ToRotation();
}


void mat3::ToAxisAngle(vec3& axis, double& angleRad) const
{
    // Let (x,y,z) be the unit-length axis and let A be an angle of rotation.
    // The rotation matrix is R = I + sin(A)*P + (1-cos(A))*P^2 where
    // I is the identity and
    //
    //       +-        -+
    //   P = |  0 +z -y |
    //       | -z  0 +x |
    //       | +y -x  0 |
    //       +-        -+
    //
    // Some algebra will show that
    //
    //   cos(A) = (trace(R)-1)/2  and  R - R^t = 2*sin(A)*P

    double fTrace = mM[0][0] + mM[1][1] + mM[2][2];
    angleRad = acos( 0.5f * (fTrace - 1.0f));

    axis[VX] = mM[1][2] - mM[2][1];
    axis[VY] = mM[2][0] - mM[0][2];
    axis[VZ] = mM[0][1] - mM[1][0];
    double fLength = axis.Length();
    const double fEpsilon = 1e-06f;
    if ( fLength > fEpsilon )
    {
        double fInvLength = 1.0f / fLength;
        axis *= -fInvLength;
    }
    else  // angle is 0 or pi
    {
        if ( angleRad > 1.0f )  // any number strictly between 0 and pi works
        {
            // angle must be pi
            axis[VX] = sqrt(0.5f * (1.0f + mM[0][0]));
            axis[VY] = sqrt(0.5f * (1.0f + mM[1][1]));
            axis[VZ] = sqrt(0.5f * (1.0f + mM[2][2]));

            // determine signs of axis components
            double tx, ty, tz;
            tx = mM[0][0] * axis[VX] + mM[0][1] * axis[VY] + mM[0][2] * axis[VZ] - axis[VX];
            ty = mM[1][0] * axis[VX] + mM[1][1] * axis[VY] + mM[1][2] * axis[VZ] - axis[VY];
            tz = mM[2][0] * axis[VX] + mM[2][1] * axis[VY] + mM[2][2] * axis[VZ] - axis[VZ];
            fLength = tx * tx + ty * ty + tz * tz;
            if ( fLength < fEpsilon )
            {
               axis = -axis;
               return;
            }

            axis[VZ] = -axis[VZ];
            tx = mM[0][0] * axis[VX] + mM[0][1] * axis[VY] + mM[0][2] * axis[VZ] - axis[VX];
            ty = mM[1][0] * axis[VX] + mM[1][1] * axis[VY] + mM[1][2] * axis[VZ] - axis[VY];
            tz = mM[2][0] * axis[VX] + mM[2][1] * axis[VY] + mM[2][2] * axis[VZ] - axis[VZ];
            fLength = tx * tx + ty * ty + tz * tz;
            if ( fLength < fEpsilon )
            {
               axis = -axis;
               return;
            }

            axis[VY] = -axis[VY];
            tx = mM[0][0] * axis[VX] + mM[0][1] * axis[VY] + mM[0][2] * axis[VZ] - axis[VX];
            ty = mM[1][0] * axis[VX] + mM[1][1] * axis[VY] + mM[1][2] * axis[VZ] - axis[VY];
            tz = mM[2][0] * axis[VX] + mM[2][1] * axis[VY] + mM[2][2] * axis[VZ] - axis[VZ];
            fLength = tx * tx + ty * ty + tz * tz;
            if ( fLength < fEpsilon )
            {
               axis = -axis;
               return;
            }
        }
        else
        {
            // Angle is zero, matrix is the identity, no unique axis, so
            // return (0,1,0) for as good a guess as any.
            angleRad = 0.0f;
            axis[VX] = 0.0f;
            axis[VY] = 1.0f;
            axis[VZ] = 0.0f;
        }
    }
}

mat3 mat3::FromToRotation(const vec3& fromDir, const vec3& toDir)
{
    vec3 dir1 = fromDir;
    vec3 dir2 = toDir;
    dir1.Normalize();
    dir2.Normalize();
    vec3 axis = dir1.Cross(dir2);
    double angle = acos(dir1 * dir2);
    mat3 mat;
    mat.FromAxisAngle(axis, angle);
    vec3 tmp1 = mat * dir1;
    //std::cout << "CHECK " << tmp1 << dir2 << std::endl; 
    return mat;
}

mat3 mat3::Inverse() const    // Gauss-Jordan elimination with partial pivoting
{
    mat3 a(*this),        // As a evolves from original mat into identity
    b(identity3D);   // b evolves from identity into inverse(a)
    int     i, j, i1;

    // Loop over cols of a from left to right, eliminating above and below diag
    for (j=0; j<3; j++) {   // Find largest pivot in column j among rows j..2
        i1 = j;    	    // Row with largest pivot candidate
        for (i=j+1; i<3; i++)
            if (fabs(a.mM[i].n[j]) > fabs(a.mM[i1].n[j]))
            	i1 = i;

        // Swap rows i1 and j in a and b to put pivot on diagonal
        Swap(a.mM[i1], a.mM[j]);
        Swap(b.mM[i1], b.mM[j]);

        // Scale row j to have a unit diagonal
        if (a.mM[j].n[j] == 0.)
        {
            std::cout << "mat3::inverse: singular matrix; can't invert\n";
            return b;
        }
        b.mM[j] /= a.mM[j].n[j];
        a.mM[j] /= a.mM[j].n[j];

        // Eliminate off-diagonal elements in col j of a, doing identical ops to b
        for (i=0; i<3; i++)
            if (i!=j) 
            {
            	b.mM[i] -= a.mM[i].n[j]*b.mM[j];
            	a.mM[i] -= a.mM[i].n[j]*a.mM[j];
            }
    }
    return b;
}

void mat3::FromAxisAngle(const vec3& axis, double angleRad)
{
    *this = Rotation3D(axis, angleRad);
}


// ASSIGNMENT OPERATORS

mat3& mat3::operator = ( const mat3& m )
{ 
    mM[0] = m.mM[0]; mM[1] = m.mM[1]; mM[2] = m.mM[2]; 
    return *this; 
}

mat3& mat3::operator += ( const mat3& m )
{ 
    mM[0] += m.mM[0]; mM[1] += m.mM[1]; mM[2] += m.mM[2]; 
    return *this; 
}

mat3& mat3::operator -= ( const mat3& m )
{ 
    mM[0] -= m.mM[0]; mM[1] -= m.mM[1]; mM[2] -= m.mM[2]; 
    return *this; 
}

mat3& mat3::operator *= ( double d )
{ 
    mM[0] *= d; mM[1] *= d; mM[2] *= d; 
    return *this; 
}

mat3& mat3::operator /= ( double d )
{ 
    mM[0] /= d; mM[1] /= d; mM[2] /= d; 
    return *this; 
}

vec3& mat3::operator [] ( int i) 
{
    assert(! (i < VX || i > VZ));
    return mM[i];
}

const vec3& mat3::operator [] ( int i) const 
{
    assert(!(i < VX || i > VZ));
    return mM[i];
}

// SPECIAL FUNCTIONS

mat3 mat3::Transpose() const 
{
    return mat3(vec3(mM[0][0], mM[1][0], mM[2][0]),
        vec3(mM[0][1], mM[1][1], mM[2][1]),
        vec3(mM[0][2], mM[1][2], mM[2][2]));
}


void mat3::WriteToGLMatrix(float* m) const
{
    m[0] = mM[0][0]; m[4] = mM[0][1]; m[8] = mM[0][2];  m[12] = 0.0f;
    m[1] = mM[1][0]; m[5] = mM[1][1]; m[9] = mM[1][2];  m[13] = 0.0f;
    m[2] = mM[2][0]; m[6] = mM[2][1]; m[10] = mM[2][2]; m[14] = 0.0f;
    m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;
}

void mat3::ReadFromGLMatrix(float* m)
{
    mM[0][0] = m[0]; mM[0][1] = m[4]; mM[0][2] = m[8];
    mM[1][0] = m[1]; mM[1][1] = m[5]; mM[1][2] = m[9];
    mM[2][0] = m[2]; mM[2][1] = m[6]; mM[2][2] = m[10];
}

vec3 mat3::GetRow(unsigned int axis) const
{
    vec3 rowVec = mM[axis];
    return rowVec;
}

vec3 mat3::GetCol(unsigned int axis) const
{
    vec3 colVec;
    colVec[0] = mM[0][axis]; colVec[1] = mM[1][axis]; colVec[2] = mM[2][axis];
    return colVec;
}

void mat3::SetRow(unsigned int axis, const vec3& rowVec)
{
    mM[axis] = rowVec;
}

void mat3::SetCol(unsigned int axis, const vec3& colVec)
{
    mM[0][axis] = colVec[0]; mM[1][axis] = colVec[1]; mM[2][axis] = colVec[2];
}

vec3 mat3::GetYawPitchRoll(unsigned int leftAxis, unsigned int upAxis, unsigned int frontAxis) const
{
    // Assume world coordinates: Y up, X left, Z front.

    vec3 leftVect, upVect, frontVect, dVect, angles, frontVect2, leftVect2;
    double t, value, x, y;
    leftVect = GetCol(leftAxis);
    upVect = GetCol(upAxis);
    frontVect = GetCol(frontAxis);

    // Compute yaw angle
    if (frontVect[VY] >= 0.0f && upVect[VY] >= 0.0f)
    {
        frontVect2 = frontVect;
        dVect = -upVect - frontVect2;
    }else if (frontVect[VY] < 0.0f && upVect[VY] < 0.0f)
    {
        frontVect2 = -frontVect;
        dVect = upVect - frontVect2;
    }else if (frontVect[VY] >= 0.0f && upVect[VY] < 0.0f)
    {
        frontVect2 = -frontVect;
        dVect = -upVect - frontVect2;

    }else if (frontVect[VY] < 0.0f && upVect[VY] >= 0.0f)
    {
        frontVect2 = frontVect;
        dVect = upVect - frontVect2;
    }
    t = -frontVect2[VY] / dVect[VY];
    x = frontVect2[VZ] + t * dVect[VZ];
    y = frontVect2[VX] + t * dVect[VX];
    angles[0] = atan2(y, x);
    frontVect2 = vec3(y, 0.0f, x);
    frontVect2.Normalize();
    leftVect2 = vec3(0.0f, 1.0f, 0.0f);
    leftVect2 = leftVect2.Cross(frontVect2);

    // Compute pitch angle
    double v = acos(frontVect * frontVect2);
    if (frontVect[VY] >= 0.0f)
    {
        value = -v;
    }else
    {
        value = v;
    }
    angles[1] = value;

    // Compute roll angle
    v = acos(leftVect * leftVect2);
    if (leftVect[VY] >= 0.0f)
    {
        value = -v;
    }else
    {
        value = v;
    }
    angles[2] = value;

    return angles;
}

//OpenGL transformation matrix
//    Rx =
//    |1       0        0    Tx|
//    |0  cos(a)  -sin(a)    Ty|
//    |0  sin(a)   cos(a)    Tz|
//    |0       0        0    1 |
//
//    Ry =
//    | cos(a)  0  sin(a)    Tx|
//    |      0  1       0    Ty|
//    |-sin(a)  0  cos(a)    Tz|
//    |      0  0       0    1 |
//
//    Rz = 
//    |cos(a)  -sin(a)  0   Tx|
//    |sin(a)   cos(a)  0   Ty|
//    |     0        0  1   Tz|
//    |     0        0  0   1 |
//
// However, when they are stored in OpenGL matrix, they are stored column major
// OpenGL convention
// m[0] = R[0][0]; m[4] = R[0][1]; m[8]  = R[0][2]; m[12] = Tx;
// m[1] = R[1][0]; m[5] = R[1][1]; m[9]  = R[1][2]; m[13] = Ty;
// m[2] = R[2][0]; m[6] = R[2][1]; m[10] = R[2][2]; m[14] = Tz;
// m[3] = 0.0f;    m[7] = 0.0f;    m[11] = 0.0f;    m[15] = 1.0f;

// FRIENDS

mat3 operator - (const mat3& a)
{ 
    return mat3(-a.mM[0], -a.mM[1], -a.mM[2]); 
}

mat3 operator + (const mat3& a, const mat3& b)
{ 
    return mat3(a.mM[0] + b.mM[0], a.mM[1] + b.mM[1], a.mM[2] + b.mM[2]); 
}

mat3 operator - (const mat3& a, const mat3& b)
{ 
    return mat3(a.mM[0] - b.mM[0], a.mM[1] - b.mM[1], a.mM[2] - b.mM[2]); 
}

mat3 operator * (const mat3& a, const mat3& b)
{
#define ROWCOL(i, j) \
    a.mM[i].n[0]*b.mM[0][j] + a.mM[i].n[1]*b.mM[1][j] + a.mM[i].n[2]*b.mM[2][j]
    return mat3(vec3(ROWCOL(0,0), ROWCOL(0,1), ROWCOL(0,2)),
        vec3(ROWCOL(1,0), ROWCOL(1,1), ROWCOL(1,2)),
        vec3(ROWCOL(2,0), ROWCOL(2,1), ROWCOL(2,2)));
#undef ROWCOL // (i, j)
}

mat3 operator * (const mat3& a, double d)
{ 
    return mat3(a.mM[0] * d, a.mM[1] * d, a.mM[2] * d); 
}

mat3 operator * (double d, const mat3& a)
{ 
    return a*d; 
}

mat3 operator / (const mat3& a, double d)
{ 
    return mat3(a.mM[0] / d, a.mM[1] / d, a.mM[2] / d); 
}

int operator == (const mat3& a, const mat3& b)
{ 
    return (a.mM[0] == b.mM[0]) && (a.mM[1] == b.mM[1]) && (a.mM[2] == b.mM[2]); 
}

int operator != (const mat3& a, const mat3& b)
{ 
    return !(a == b); 
}

void Swap(mat3& a, mat3& b)
{ 
    mat3 tmp(a); a = b; b = tmp; 
}

std::istream& operator >> (std::istream& s, mat3& m)
{
    double value;
    for (unsigned int i = 0; i < 3; i++)
        for (unsigned int j = 0; j < 3; j++)
        {
            s >> value;
            m[i][j] = value;
        }
    return s;
}

std::ostream& operator << (std::ostream& s, const mat3& m)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        for (unsigned int j = 0; j < 2; j++)
        {
            s << (float) m[i][j] << " ";
        }
        s << (float) m[i][2] << std::endl;
    }
    return s;
}


 vec3 operator * (const mat3& a, const vec3& v)
{
#define ROWCOL(i) a.mM[i].n[0]*v.n[VX] + a.mM[i].n[1]*v.n[VY] \
    + a.mM[i].n[2]*v.n[VZ]
    return vec3(ROWCOL(0), ROWCOL(1), ROWCOL(2));
#undef ROWCOL // (i)
}

 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // quat class member funtions

quat::quat()
{
    mQ[VW] = 0; mQ[VX] = 0; mQ[VY] = 0; mQ[VZ] = 0;
}

quat::quat(double w, double x, double y, double z)
{
    mQ[VW] = w; mQ[VX] = x; mQ[VY] = y; mQ[VZ] = z;
}

quat::quat(const quat& q)
{
    mQ[VW] = q.mQ[VW]; mQ[VX] = q.mQ[VX]; mQ[VY] = q.mQ[VY]; mQ[VZ] = q.mQ[VZ];
}

// Static functions

double quat::Distance(const quat& q1, const quat& q2) // returns angle between in radians
{
    double inner_product = quat::Dot(q1, q2);
    if (inner_product < 0.0)
    {
        inner_product = quat::Dot(q1, -q2);
    }

    double tmp = std::min<double>(1.0, std::max<double>(-1.0,2*inner_product*inner_product - 1));
    double theta = acos(tmp);
    return theta;
}

double quat::Dot(const quat& q0, const quat& q1)
{
    return q0.mQ[VW] * q1.mQ[VW] + q0.mQ[VX] * q1.mQ[VX] + q0.mQ[VY] * q1.mQ[VY] + q0.mQ[VZ] * q1.mQ[VZ];
}

quat quat::UnitInverse(const quat& q)
{
    return quat(q.mQ[VW], -q.mQ[VX], -q.mQ[VY], -q.mQ[VZ]);
}

// Assignment operators
quat& quat::operator = (const quat& q)
{
    mQ[VW] = q.mQ[VW]; mQ[VX] = q.mQ[VX]; mQ[VY] = q.mQ[VY]; mQ[VZ] = q.mQ[VZ];
    return *this;
}

quat& quat::operator += (const quat& q)
{
    mQ[VW] += q.mQ[VW]; mQ[VX] += q.mQ[VX]; mQ[VY] += q.mQ[VY]; mQ[VZ] += q.mQ[VZ];
    return *this;
}

quat& quat::operator -= (const quat& q)
{
    mQ[VW] -= q.mQ[VW]; mQ[VX] -= q.mQ[VX]; mQ[VY] -= q.mQ[VY]; mQ[VZ] -= q.mQ[VZ];
    return *this;
}

quat& quat::operator *= (const quat& q)
{
    *this = quat(mQ[VW] * q.mQ[VW] - mQ[VX] * q.mQ[VX] - mQ[VY] * q.mQ[VY] - mQ[VZ] * q.mQ[VZ],
        mQ[VW] * q.mQ[VX] + mQ[VX] * q.mQ[VW] + mQ[VY] * q.mQ[VZ] - mQ[VZ] * q.mQ[VY],
        mQ[VW] * q.mQ[VY] + mQ[VY] * q.mQ[VW] + mQ[VZ] * q.mQ[VX] - mQ[VX] * q.mQ[VZ],
        mQ[VW] * q.mQ[VZ] + mQ[VZ] * q.mQ[VW] + mQ[VX] * q.mQ[VY] - mQ[VY] * q.mQ[VX]);
    return *this;
}

quat& quat::operator *= (double d)
{
    mQ[VW] *= d; mQ[VX] *= d;    mQ[VY] *= d; mQ[VZ] *= d;
    return *this;
}

quat& quat::operator /= (double d)
{
    mQ[VW] /= d; mQ[VX] /= d;    mQ[VY] /= d; mQ[VZ] /= d;
    return *this;
}

// Indexing
double& quat::operator [](int i)
{
    return mQ[i];
}

double quat::operator [](int i) const
{
    return mQ[i];
}

double& quat::W()
{
    return mQ[VW];
}

double quat::W() const
{
    return mQ[VW];
}

double& quat::X()
{
    return mQ[VX];
}

double quat::X() const
{
    return mQ[VX];
}

double& quat::Y()
{
    return mQ[VY];
}

double quat::Y() const
{
    return mQ[VY];
}

double& quat::Z()
{
    return mQ[VZ];
}

double quat::Z() const
{
    return mQ[VZ];
}

// Friends

quat operator - (const quat& q)
{
    return quat(-q.mQ[VW], -q.mQ[VX], -q.mQ[VY], -q.mQ[VZ]); 
}

quat operator + (const quat& q0, const quat& q1)
{
    return quat(q0.mQ[VW] + q1.mQ[VW], q0.mQ[VX] + q1.mQ[VX], q0.mQ[VY] + q1.mQ[VY], q0.mQ[VZ] + q1.mQ[VZ]);
}

quat operator - (const quat& q0, const quat& q1)
{
    return quat(q0.mQ[VW] - q1.mQ[VW], q0.mQ[VX] - q1.mQ[VX], q0.mQ[VY] - q1.mQ[VY], q0.mQ[VZ] - q1.mQ[VZ]);
}

quat operator * (const quat& q, double d)
{
    return quat(q.mQ[VW] * d, q.mQ[VX] * d, q.mQ[VY] * d, q.mQ[VZ] * d);
}

quat operator * (double d, const quat& q)
{
    return quat(q.mQ[VW] * d, q.mQ[VX] * d, q.mQ[VY] * d, q.mQ[VZ] * d);
}

quat operator * (const quat& q0, const quat& q1)
{
    return quat(q0.mQ[VW] * q1.mQ[VW] - q0.mQ[VX] * q1.mQ[VX] - q0.mQ[VY] * q1.mQ[VY] - q0.mQ[VZ] * q1.mQ[VZ],
        q0.mQ[VW] * q1.mQ[VX] + q0.mQ[VX] * q1.mQ[VW] + q0.mQ[VY] * q1.mQ[VZ] - q0.mQ[VZ] * q1.mQ[VY],
        q0.mQ[VW] * q1.mQ[VY] + q0.mQ[VY] * q1.mQ[VW] + q0.mQ[VZ] * q1.mQ[VX] - q0.mQ[VX] * q1.mQ[VZ],
        q0.mQ[VW] * q1.mQ[VZ] + q0.mQ[VZ] * q1.mQ[VW] + q0.mQ[VX] * q1.mQ[VY] - q0.mQ[VY] * q1.mQ[VX]);
}

quat operator / (const quat& q, double d)
{
    return quat(q.mQ[VW] / d, q.mQ[VX] / d, q.mQ[VY] / d, q.mQ[VZ] / d);
}

bool operator == (const quat& q0, const quat& q1)
{
    return (q0.mQ[VW] == q1.mQ[VW]) && (q0.mQ[VX] == q1.mQ[VX]) && (q0.mQ[VY] == q1.mQ[VY]) && (q0.mQ[VZ] == q1.mQ[VZ]);
}

bool operator != (const quat& q0, const quat& q1)
{
    return !(q0 == q1); 
}

// special functions

double quat::SqrLength() const
{
    return mQ[VW] * mQ[VW] + mQ[VX] * mQ[VX] + mQ[VY] * mQ[VY] + mQ[VZ] * mQ[VZ];
}

double quat::Length() const
{
    double l = SqrLength();
    if (l > EPSILON)
        return sqrt(SqrLength());
    else 
        return 0;
}

quat& quat::Normalize()
{
    double l = Length();
    if (l < EPSILON || abs(l) > 1e6)
    {
        FromAxisAngle(vec3(0.0f, 1.0f, 0.0f), 0.0f);
    }else
    {
        *this /= l;
    }

    return *this; 
}


quat quat::Conjugate() const
{
    return quat(mQ[VW], -mQ[VX], -mQ[VY], -mQ[VZ]);
}

quat quat::Inverse() const
{
    return Conjugate() / SqrLength();
}

quat quat::Exp(const quat& q)
{
    // q = A*(x*i+y*j+z*k) where (x,y,z) is unit length
    // exp(q) = cos(A)+sin(A)*(x*i+y*j+z*k)
    double angle = sqrt(q.mQ[VX] * q.mQ[VX] + q.mQ[VY] * q.mQ[VY] + q.mQ[VZ] * q.mQ[VZ]);
    double sn, cs;
    sn = sin(angle);
    cs = cos(angle);

    // When A is near zero, sin(A)/A is approximately 1.  Use
    // exp(q) = cos(A)+A*(x*i+y*j+z*k)
    double coeff = ( abs(sn) < EPSILON ? 1.0f : sn/angle );

    quat result(cs, coeff * q.mQ[VX], coeff * q.mQ[VY], coeff * q.mQ[VZ]);

    return result.Normalize();
}

quat quat::Log(const quat& q)
{
    // q = cos(A)+sin(A)*(x*i+y*j+z*k) where (x,y,z) is unit length
    // log(q) = A*(x*i+y*j+z*k)
    
    double angle = acos(q.mQ[VW]);
    double sn = sin(angle);

    // When A is near zero, A/sin(A) is approximately 1.  Use
    // log(q) = sin(A)*(x*i+y*j+z*k)
    double coeff = ( abs(sn) < EPSILON ? 1.0f : angle/sn );

    return quat(0.0f, coeff * q.mQ[VX], coeff * q.mQ[VY], coeff * q.mQ[VZ]);
}

void quat::Zero()
{
    mQ[VW] = mQ[VX] = mQ[VY] = mQ[VZ] = 0.0f;
}

#define Q_EST(a, b, c) 0.25*(1 + a + b + c)
#define Q_MAX(a, b, c, d) std::max(a, std::max(b, std::max(c, d))) 

void quat::FromRotation (const mat3& rot)
{
	mQ[VW] = 0.0; mQ[VX] = 1.0; mQ[VY] = 0.0;  mQ[VZ] = 0.0;

	//TODO: student implementation for converting from rotation matrix to quat goes here
	double trace = rot[0][0] + rot[1][1] + rot[2][2];
	if (trace > 0)
	{
		mQ[VW] = sqrt(1 + rot[0][0] + rot[1][1] + rot[2][2]) / 2.0;
		mQ[VX] = (rot[2][1] - rot[1][2]) / (4.0*mQ[VW]);
		mQ[VY] = (rot[0][2] - rot[2][0]) / (4.0*mQ[VW]);
		mQ[VZ] = (rot[1][0] - rot[0][1]) / (4.0*mQ[VW]);
	}
	else if (rot[0][0] > rot[1][1] && rot[0][0] > rot[2][2])
	{
		mQ[VX] = sqrt(1 + rot[0][0] - rot[1][1] - rot[2][2]) / 2.0;
		mQ[VW] = (rot[2][1] - rot[1][2]) / (4.0*mQ[VX]);
		mQ[VY] = (rot[0][1] + rot[1][0]) / (4.0*mQ[VX]);
		mQ[VZ] = (rot[0][2] + rot[2][0]) / (4.0*mQ[VX]);
	}
	else if (rot[1][1] > rot[2][2])
	{
		mQ[VY] = sqrt(1 + rot[1][1] - rot[0][0] - rot[2][2]) / 2.0;
		mQ[VW] = (rot[0][2] - rot[2][0]) / (4.0*mQ[VY]);
		mQ[VX] = (rot[0][1] + rot[1][0]) / (4.0*mQ[VY]);
		mQ[VZ] = (rot[1][2] + rot[2][1]) / (4.0*mQ[VY]);
	}
	else
	{
		mQ[VZ] = sqrt(1 + rot[2][2] - rot[0][0] - rot[1][1]) / 2.0;
		mQ[VW] = (rot[1][0] - rot[0][1]) / (4.0*mQ[VZ]);
		mQ[VX] = (rot[0][2] + rot[2][0]) / (4.0*mQ[VZ]);
		mQ[VY] = (rot[1][2] + rot[2][1]) / (4.0*mQ[VZ]);
	}

   Normalize();
}

quat quat::Slerp(const quat& q0, const quat& q1, double u)
{
	quat q = q0;
	//TODO: student implemetation of Slerp goes here

	double omega = acos(Dot(q0, q1));

	q = sin((1.0 - u)*omega) / sin(omega)* q0 + sin(u*omega)/ sin(omega) * q1;

	return q.Normalize();
}
quat quat::SDouble(const quat& a, const quat& b)
{
	quat q = a;
	//TODO: student implementation ofSDouble goes here

	q = 2 * quat::Dot(a, b) * b - a;

	return q.Normalize();
}

quat quat::SBisect(const quat& a, const quat& b)
{
	quat q = a;
	//TODO: student implementation of SBisect goes here
	q = (a + b);

	return q.Normalize();
}


quat quat::Scubic(const quat& b0, const quat& b1, const quat& b2, const quat& b3, double u)
{
	quat result = b0;
	quat b01, b11, b21, b02, b12, b03;
	// TODO: Return the result of Scubic based on the cubic quaternion curve control points b0, b1, b2 and b3

	b01 = Slerp(b0, b1, u);
	b11 = Slerp(b1, b2, u);
	b21 = Slerp(b2, b3, u);

	b02 = Slerp(b01, b11, u);
	b12 = Slerp(b11, b21, u);

	b03 = Slerp(b02, b12, u);

	result = b03;

	return result.Normalize(); // result should be of unit length
}


quat quat::Intermediate(const quat& q0, const quat& _q1, const quat& _q2)
{
	// assert:  q0, q1, q2 are unit quaternion
	quat q1 = _q1;
	quat q2 = _q2;

	if (quat::Dot(q0, q1) < 0) q1 = -q1;
	if (quat::Dot(q0, q2) < 0) q2 = -q2;

	quat inv = UnitInverse(q1);
	quat exp = Exp(-0.25f * (Log(inv * q0) + Log(inv * q2)));
	return q1 * exp;
}

quat quat::Squad(const quat& q0, const quat& a, const quat& b, const quat& q1, double t)
{
    return Slerp(Slerp(q0, q1, t), Slerp(a, b, t), 2.0f * t * (1.0f - t));
}

vec3 quat::ToExpMap() const
{
    vec3 axis; double angle;
    ToAxisAngle(axis, angle);

    vec3 expmap(0,0,0);
    if (fabs(angle) > 0.000001)
    {
        double factor = angle / sin(0.5*angle);
        expmap = vec3(factor * X(), factor * Y(), factor * Z());
    }
    return expmap;
}

void quat::FromExpMap(const vec3& expmap)
{
    double theta = expmap.Length();
    double scale = 0.0;
    if (fabs(theta) < 0.032)
    {
        scale = 0.5 + (theta*theta)*0.021;
    }
    else
    {
        scale = sin(0.5*theta)/theta;
    }
    mQ[VW] = cos(0.5*theta);
    mQ[VX] = expmap[0]*scale;
    mQ[VY] = expmap[1]*scale;
    mQ[VZ] = expmap[2]*scale;
    Normalize(); 
}

quat quat::ProjectToAxis(const quat& q, vec3& axis)
{
    axis.Normalize();
    vec3 qv = vec3(q.X(), q.Y(), q.Z());
    double angle = acos(q.W());
    double sn = sin(angle);
    vec3 qaxis = qv / sn;
    qaxis.Normalize();
    angle = qaxis * axis;
    double halfTheta;
    if (angle < EPSILON)
    {
        halfTheta = 0.0f;
    }else
    {
        double s = axis * qv;
        double c = q.W();
        halfTheta = atan2(s, c);
    }    
    double cn = cos(halfTheta);
    sn = sin(halfTheta);
    return quat(cn, sn * axis[VX], sn * axis[VY], sn * axis[VZ]); 
}

// Conversion functions
void quat::ToAxisAngle (vec3& axis, double& angleRad) const
{
	//TODO: student implementation for converting quaternion to axis/angle representation goes here

	double half_theta = acos(mQ[VW]);
	angleRad = 2*half_theta;
	//axis = vec3(mQ[VX], mQ[VY], mQ[VZ]) / sin(half_theta);
	//normalize is faster than sin
	axis = vec3(mQ[VX], mQ[VY], mQ[VZ]);
	axis.Normalize();
}

void quat::FromAxisAngle (const vec3& axis, double angleRad)
{
	//TODO: student implementation for converting from axis/angle to quaternion goes here

	//assuming the axis is not an unit vector
	vec3 unit_axis = axis;
	unit_axis.Normalize();

	double half_theta = angleRad / 2;
	double sin_half_theta = sin(half_theta);
	mQ[VW] = cos(half_theta);
	mQ[VX] = sin_half_theta*unit_axis[VX];
	mQ[VY] = sin_half_theta*unit_axis[VY];
	mQ[VZ] = sin_half_theta*unit_axis[VZ];
}

mat3 quat::ToRotation () const
{
	mat3 m;
	m.Identity();
	//TODO: student implementation for converting quaternion to rotation matrix goes here

	double x = mQ[VX];
	double y = mQ[VY];
	double z = mQ[VZ];
	double w = mQ[VW];

	m[0][0] = 1 - 2 * y*y - 2 * z*z;
	m[0][1] = 2 * x*y - 2 * w*z;
	m[0][2] = 2 * x*z + 2 * w*y;
	m[1][0] = 2 * x*y + 2 * w*z;
	m[1][1] = 1 - 2 * x*x - 2 * z*z;
	m[1][2] = 2 * y*z - 2 * w*x;
	m[2][0] = 2 * x*z - 2 * w*y;
	m[2][1] = 2 * y*z + 2 * w*x;
	m[2][2] = 1 - 2 * x*x - 2 * y*y;

    return m;
}

std::istream& operator >> (std::istream& s, quat& q)
{
    double x, y, z, w;
    s >> w >> x >> y >> z;
    q[VX] = x;
    q[VY] = y;
    q[VZ] = z;
    q[VW] = w;
    return s;
}

std::ostream& operator << (std::ostream& s, const quat& q)
{
    s << (float) q[VW] << " " << (float) q[VX] << " " << (float) q[VY] << " " << (float) q[VZ];
    return s;
}

double Lerp(double q0, double q1, double t)
{
    return q0*(1 - t) + q1*(t);
}
