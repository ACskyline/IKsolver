#ifndef PARTICLESYSTEM_H
#define PARTICLESYSTEM_H

#include <vector>
#include "aJoint.h"
#include "aVector.h"
#include "aJitter.h"
#include "aParticle.h"

class AParticle;
class AParticleSystem 
{
public:
    AParticleSystem();
    virtual ~AParticleSystem();

    virtual void setRoot(AJoint& joint) { mRoot = &joint;  }

    virtual void drawOpenGL(); 

    virtual void reset();
    virtual bool isAlive(); 

	AParticle* getParticle(int index) { return mParticles[index]; }
 
	virtual void update(double deltaT);

protected:
    std::vector<AParticle*> mParticles; // vector containing pointers to particles
    AJoint* mRoot; //  used to attach the particle system to a joint 

public:

    bool mInfinite;                // determines whether the particle system plays once, or continuously respawns particles
    
	unsigned int mMaxParticles;    // max number of particles that can be stored in mParticles
    double mLifeSpan;              // default life span of particles
	double m_deltaT;               // size of simulation timestep
	int m_spawnTime;               // number of time steps before emmitting another particle

    vec3 mStartPos;                // location of particle emitter
    vec3 mStartVel;                // initial velocity of particle leaving emitter
	vec3 mGravity;                 // direction and mag of gravity vector in world coords

	vec3 mStartColor, mEndColor;
	double mStartAlpha, mEndAlpha;
	double mStartScale, mEndScale;

	// the jitter variables are used to add noise to particle parameters and values
	AJitter mJitterTime;
	AJitter mScaleJitter;
    AJitter mColorJitter;
    AJitter mPositionJitter;
    AJitter mVelocityJitter;
};

#endif 
