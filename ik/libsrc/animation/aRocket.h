// aRocket.h: interface for the ARocket class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(ROCKET_H)
#define ROCKET_H

#include "ASpark.h"

// Rocket modes
enum ROCKETMODE {FLYING, EXPLOSION, DEAD};

// Number of times this rocket will explode before it dies
#define TOTALEXPLOSIONS 3  

#define EXT_ROCKETFORCES_ACTIVE 0x40

class ARocket : public ASpark
{
public:
	//Member functions:
	//Constructor
	ARocket(float* color);

	//Deconstructor
	virtual ~ARocket();

	//Display the Rocket
	void display();

	//Computes one time step of simulation
	virtual void update(float deltaT, int extForceMode);

	//Compute the forces on this rocket
	void computeForces(int extForceMode);

	//Member variables

	//Explosion	count
	int m_explosionCount;

	//Rocket mode, see enum ROCKETMODE {FLYING, EXPLOSION, DEAD};
	int m_mode;

	// elevation angle
	float m_angle;

	// min vertical velocity for rocket to explode
	float m_Vexplode;
};

#endif // !defined(ROCKET_H)
