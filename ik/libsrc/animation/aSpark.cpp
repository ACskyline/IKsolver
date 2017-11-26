// Spark.cpp: implementation of the ASpark class.
//
//////////////////////////////////////////////////////////////////////

#include "aSpark.h"
#include <math.h>

#ifndef GRAVITY
#define GRAVITY 9.8f
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ASpark::ASpark()
{
	//coefficients of restitution equals 0.25
	m_COR = 0.25f;
	m_mass = 1.0;
}

ASpark::ASpark(float* color): AParticle()
{
	for (int i = 0; i < 3; i++)
		m_color[i] = color[i];
 
	//coefficients of restitution equals 0.25
	m_COR = 0.25f;
}

ASpark::~ASpark()
{

}

//Set attractor position
void ASpark::setAttractor(vec3 position)
{
	m_attractorPos = position;
}

//Set repeller position
void ASpark::setRepeller(vec3 position)
{
	m_repellerPos = position;
}

void ASpark::setWind(vec3 wind)
{
	m_windForce = wind;
}

void ASpark::display()
{
	float fadeTime = 3.0;
	if (m_alive)
	{
		float alpha = 1.0;
		if (m_state[10] < fadeTime)
		{
			alpha = m_state[10] / 10.0f;
		}
		float scale = 1.0;

		glPushMatrix();
		glColor4f(m_color[0], m_color[1], m_color[2], alpha);
		glTranslatef(m_state[0], m_state[1], m_state[2]);
		glScalef(scale, scale, scale);
		glutSolidSphere(1.0, 10, 10);
		glPopMatrix();
	}

}
	


void ASpark::update(float deltaT, int extForceMode)
{
	m_deltaT = deltaT;
	if (m_state[10] <= 0.0)
	{
		m_alive = false;
		return;
	}

	if (!(extForceMode & EXT_SPARKFORCES_ACTIVE))
		extForceMode = 0;
	
	computeForces(extForceMode);
	
	updateState(deltaT, EULER);

	resolveCollisions();
	
	
}


 
void ASpark::computeForces(int extForceMode)
//	computes the forces applied to this spark
{
	// zero out all forces
	m_state[6] = 0.0;
	m_state[7] = 0.0;
	m_state[8] = 0.0;

	// gravity force
	addForce(m_mass*m_gravity);


	// wind force
	if (extForceMode & WIND_ACTIVE)
	{
		//TODO: Add your code here


	

	}

	if (extForceMode & DRAG_ACTIVE)
	{
		//TODO: Add your code here


;
	}


	// attractor force
	if (extForceMode & ATTRACTOR_ACTIVE)
	{
		//TODO: Add your code here


	
	}

	// repeller force
	if (extForceMode & REPELLER_ACTIVE)
	{
		//TODO: Add your code here


	}

	// random force
	if (extForceMode & RANDOM_ACTIVE)
	{
		//TODO: Add your code here



	}

}

void ASpark::resolveCollisions()
// resolves collisions of the spark with the ground
{
	//TODO: Add  code here that reverses the y value of the spark velocity vector when the y position value of the spark is < 0

}
