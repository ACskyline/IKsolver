// aParticle.h: interface for the AParticle class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(PARTICLE_H)
#define PARTICLE_H

#include "aVector.h"
#include <vector>
#include <stdlib.h>
#include <GL/glut.h> 
#include "aParticleSystem.h"
using namespace std;

class AParticleSystem;

// Integration types
enum INTEGRATORTYPE { EULER, RK2, ADAMS, RK4 };

class AParticle
{
public:
        //Constructor
        AParticle();

        //Deconstructor
        virtual ~AParticle();

        //Get the state vector
		vector<float> getState();

        //Set the particle state vector
		void setState(vector<float>& newState);
		void setState(float *newState);

		//Get the particle state vector
		vector<float> getStateDot();

		//Get the state vector dimension  
		int getDim();

		//Set the state vector dimension  
		void setDim(int dim);

		//Get time to live
		float getTimeToLive();

		//Set time to live
		void setLifeSpan(float time);

		//checks if alive
		bool isAlive();

		// sets time to live to lifeSpan
		void setAlive();

		// kills particle and sets time to live to 0
		void kill();

		//Set mass
		void setMass(float mass);

		//Get mass
		float getMass();

		//updates particle state
		virtual void updateState(float deltaT, int integratorType);

		//adds force to particle
		void addForce(vec3 force);

        //Compute forces on this particle
        virtual void computeForces(int mode);

        //given the state computes stateDot based on the dynamics of the particle
		virtual void computeDynamics(vector<float>& state, vector<float>& stateDot, float deltaT);

        // computes one simulation step update 
		virtual void update(float deltaT, int mode);

		// allows the particle system holding the particle to set initial values of particle
		virtual void initialize(AParticleSystem& parent);


//Member variables
        //Dimension of the state vector
        int m_dim;

		/* State vector
		*	State vector:
		*  0 : position x
		*  1 : position y
		*  2 : position z
		*  3 : velocity x
		*  4 : velocity y
		*  5 : velocity z
		*  6 : force x
		*  7 : force y
		*  8 : force z
		*  9 : mass
		*  10 : timeToLive
		*  11 : not defined
		*/
		vector<float> m_state;

        //Derivative of the state vector
		vector<float> m_stateDot;

        //Delta time of one simulation step
        float m_deltaT;

        //Indicate if the particle is alive or dead
        bool m_alive;

		double m_lifeSpan; //time to live
		double m_mass;
		vec3 m_Pos, m_Vel, m_gravity;
		vec3 mColor, mStartColor, mEndColor;
		double mScale, mStartScale, mEndScale;
		double mAlpha, mStartAlpha, mEndAlpha;
};

#endif // !defined(PARTICLE_H)
