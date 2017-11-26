// Rocket.cpp: implementation of the ARocket class.
//
//////////////////////////////////////////////////////////////////////

#include "aRocket.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>

#ifndef RAD
#define PI 3.14159265358979f
#define RAD (PI / 180.0f)
#endif
#ifndef GRAVITY
#define GRAVITY 9.8f
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ARocket::ARocket(float* color): ASpark()
{
	m_explosionCount = -1;
	for (int i = 0; i < 3; i++)
		m_color[i] = color[i];
	m_mode = FLYING;
	m_Vexplode = 10.0;
	
	m_mass = 50.0;
}

ARocket::~ARocket()
{

}

void ARocket::display()
{
	if (m_explosionCount < 0)
	{
		float alpha = 1.0;
		float scale = 6.0;

		glPushMatrix();
		double theta = atan2(m_state[4], m_state[3]) * 180.0 / PI;
		glColor4f(m_color[0], m_color[1], m_color[2], alpha);
		glTranslatef(m_state[0], m_state[1], m_state[2]);
		glScalef(scale, scale, scale);
		glRotatef(90+theta, 0, 0, 1);
		glRotatef(90, 1, 0, 0);
		glutSolidCone(1.0, 3, 10, 10);
		glPopMatrix();	
	}
}

void ARocket::update(float deltaT, int extForceMode)
{

	if (m_mode == EXPLOSION && m_explosionCount > 0)
		m_explosionCount--;

	
	if ( !(extForceMode & EXT_ROCKETFORCES_ACTIVE))
		extForceMode = 0;
	computeForces(extForceMode);
	updateState(deltaT, EULER);

	// update the orientation
	float velMag = sqrt(m_state[3] * m_state[3] + m_state[4] * m_state[4]);
	if (velMag > 0)
	{
		float cosTheta = m_state[3] / velMag;
		float sinTheta = m_state[4] / velMag;
		m_angle = atan2(sinTheta, cosTheta);
	}

	float Vmin = m_Vexplode;
	if (m_state[4] < Vmin && m_mode == FLYING)
	{
		m_mode = EXPLOSION;
		m_explosionCount = TOTALEXPLOSIONS;
	}
	if (m_mode == EXPLOSION && m_explosionCount == 0)
	{
		m_mode = DEAD;
		m_alive = false;
	}


}

void ARocket::computeForces(int extForceMode)
{

	ASpark::computeForces(extForceMode);

}


