// aFireworks.h: interface for the AFireworks class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(FIREWORKS_H)
#define FIREWORKS_H

#include <vector>
#include "aRocket.h"
#include "aSpark.h"
#include <stdlib.h>

using namespace std;


#define MAXSPARKS 50 
#define MAXVELOCITY 20
#define SPARK_LIFESPAN 10
#define ROCKET_LIFESPAN 10


class AFireworks  
{
public:
 	//Constructor
	AFireworks();

	//Deconstructor
	virtual ~AFireworks();

	//Display all the particles
	void display();

	//Calculate total number of particles
	int getNumParticles();

	//One step of the simulation
	void update(float deltaT, int extForceMode);

	//When a rocket reaches its top height, it explodes and calls this function to generate sparks
	void explode(float posx, float posy, float posz, float velx, float vely, float velz, float* color);
	
	//When the space key is pressed, the update functions calls this function to generate a rocket
	void fireRocket(float pos, float* color);	
	
//Member variables:
	//Vector containing pointers to ARocket. Rockets in this vector are either flying or generating sparks (FLYING, EXPLOSION, DEAD)
	vector<ARocket*> rockets;
	
	//Vector containing pointers to ASpark.
	vector<ASpark*> sparks;

	//Delta time for a Euler step.
	float m_deltaT;

	vec3 m_attractorPos;  // location of attractor in world
	vec3 m_repellerPos;   // location of repeller in world
	vec3 m_windForce;

	float m_rocketMass;
	float m_sparkMass;
};

#endif // !defined(FIREWORKS_H)
