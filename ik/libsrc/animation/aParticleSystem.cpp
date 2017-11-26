#include "aParticleSystem.h"
#include <Windows.h>
#include "GL/glut.h"
#include <cstring>
#include <math.h>
#include "aVector.h"


#define GRAVITY 9.8f

AParticleSystem::AParticleSystem()
{

	mMaxParticles = 20;
	mParticles.clear();

	m_deltaT = 0.033;
	mLifeSpan = 3.0; 
	mJitterTime = AJitter(-0.5 * mLifeSpan, 0);
	m_spawnTime = (int)(mLifeSpan / (mMaxParticles*m_deltaT));

	mGravity = vec3(0, -9.8, 0);

	mStartColor = vec3(1, 0, 0);
	mEndColor = vec3(0, 0, 0);
	mColorJitter = AJitter(-0.25, 0.25);

	mStartPos = vec3(0, 0, 0);
	mPositionJitter = AJitter(-0.5, 0.5);

	mStartVel = vec3(0, 20.0, 0);  
	mVelocityJitter = AJitter(-2.0, 2.0);

	mStartAlpha = 1.0;
	mEndAlpha = 0.0;

	mStartScale = 0.1;
	mEndScale = 1.0;
	mScaleJitter = AJitter(0, 0.25);
	
	mRoot = NULL;
}

AParticleSystem::~AParticleSystem()
{
	for (unsigned int i = 0; i < mParticles.size(); i++)
	{
		AParticle* pParticle = mParticles[i];
		delete pParticle;
	}

	mParticles.clear();
}

bool AParticleSystem::isAlive()
{
    if (mInfinite) return true; // if mInfinite = true particles never die!

    for (unsigned int i = 0; i < mParticles.size(); i++)
    {
        if (mParticles[i]->isAlive()) 
			return true;
    }
    return false;
}

void AParticleSystem::update(double deltaT)
{
	int forceMode = 0;
	m_deltaT = deltaT;
	static int spawnDelay = 0;

	for (unsigned int i = 0; i < mParticles.size(); i++)
	{
		if (mInfinite && mParticles[i]->isAlive())
			mParticles[i]->update(deltaT, forceMode);
		else  mParticles[i]->initialize(*this);
	}
	if (mParticles.size() < mMaxParticles)  // add new particle
	{
		spawnDelay++;
		if (spawnDelay == m_spawnTime)
		{
			AParticle *particle = new AParticle();
			particle->initialize(*this);

			if (mInfinite) // jitter the LifeSpan in infinite mode
			{
				double newLifeSpan = mLifeSpan; //+ AJitterVal(mJitterTime);
				particle->setLifeSpan(newLifeSpan);
			}
			mParticles.push_back(particle);
			spawnDelay = 0;
		}
	
	}
}

void AParticleSystem::drawOpenGL()
{
    for (unsigned int i = 0; i < mParticles.size(); i++)
    {
		AParticle* particle = mParticles[i];
		if (particle->isAlive())
		{
			// ramp the color and fade out the particle as its life runs out 
			double u = particle->getTimeToLive() / particle->m_lifeSpan;

			double scale = particle->mStartScale * u + particle->mEndScale * (1 - u);
			double alpha = particle->mStartAlpha * u + particle->mEndAlpha * (1 - u);
			vec3 color = particle->mStartColor * u + particle->mEndColor * (1 - u);

			vec3 pos = mParticles[i]->m_Pos;

			glColor4f(color[0], color[1], color[2], alpha);
			glPushMatrix();
			glTranslatef(pos[0], pos[1], pos[2]);
			glScalef(scale, scale, scale);
			glutSolidSphere(1.0, 10, 10);
			glPopMatrix();
		}
    }
}

void AParticleSystem::reset()
{
    mParticles.clear();
	m_spawnTime = (int)(mLifeSpan / (mMaxParticles*m_deltaT));

}

