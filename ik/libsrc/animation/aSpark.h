// aSpark.h: interface for the ASpark class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(SPARK_H)
#define SPARK_H

#include "AParticle.h"

// external force modes
#define WIND_ACTIVE 0x01
#define DRAG_ACTIVE 0x02
#define ATTRACTOR_ACTIVE  0x04
#define REPELLER_ACTIVE 0x08
#define RANDOM_ACTIVE 0x10
#define EXT_SPARKFORCES_ACTIVE 0x20


/* State vector - inherited from AParticle
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

class ASpark : public AParticle  
{
public:
//Member functions:
	//Constructor
	ASpark();
	ASpark(float* color);

	//Deconstructor
	virtual ~ASpark();

	//Set attractor position
	void setAttractor(vec3 position);

	//Set repeller position
	void setRepeller(vec3 position);

	//Set wind force
	void setWind(vec3 wind);

	//Display the spark
	void display();

	//Computes one time step of simulation
	virtual void update(float deltaT, int extForceMode);
	
	//Computes the forces on this spark
	void computeForces(int extForceMode);

	//resolves collisions of spark with ground
	void resolveCollisions();
	

//Member variables
	//Spark color
	float m_color[3];
	
	//Coefficients of restitution
	float m_COR;

	vec3 m_attractorPos;  // location of particle attractor
	vec3 m_repellerPos;   // location of particle repeller
	vec3 m_windForce;     
};

#endif // !defined(SPARK_H)
