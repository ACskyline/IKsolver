#ifndef BehaviorController_H_
#define BehaviorController_H_


#include <map>
#include "aVector.h"
#include "aJoint.h"
#include "aSkeleton.h"
#include "aBehaviors.h" 
#include "aActor.h"

using namespace std;

// Behavior Type is global
enum BehaviorType { SEEK, FLEE, ARRIVAL, DEPARTURE, AVOID, WANDER, ALIGNMENT, SEPARATION, COHESION, FLOCKING, LEADER };

// state vector indices
#define POS  0
#define ORI  1
#define VEL  2
#define AVEL 3
#define _X  0
#define _Y  1
#define _Z  2

class Obstacle;
class Behavior;
class AActor;

class BehaviorController
{
public:
    BehaviorController();
    virtual ~BehaviorController();

	AActor* getActor();
	void setActor(AActor* actor); 
	
	void reset();
    
    void setActive(bool b) { m_Active = b; }  // sets agent BehaviorController to active
    bool isActive()  { return m_Active; }

	//---------------------------

	void setLeader(bool b) { mLeader = b; }   // sets agent to leader
	bool isLeader()  { return mLeader; }

	virtual void display();

    AJoint& getGuide()  { return m_Guide;  }   // the guide determines the agent root position and orientation
	vec3 getPosition()  { return  m_Guide.getLocalTranslation(); return m_Pos0; }  // gets the global position of agent
    vec3 getDesiredVelocity()  { return m_Vdesired; }
    vec3 getVelocity()  { return m_Vel0;  }
	vec3 getOrientation()  { vec3 angle(0.0, m_state[ORI][_Y], 0.0); return angle; }  // gets global orientation of agent

	void setTarget(AJoint& target);  // sets the target for all the behaviors in BehaviorList
	 AJoint* getTarget()  { return m_pBehaviorTarget; }

	void setActiveBehavior(Behavior* behavior);   // sets the active behavior
	Behavior* getActiveBehavior()  { return mpActiveBehavior; }

	BehaviorType getActiveBehaviorType() { return m_BehaviorType; }  // gets active behavior type
	void setActiveBehaviorType(BehaviorType type);  // sets active behavior type
	void createBehaviors(vector<AActor>& agentList, vector<Obstacle>& obstacleList); // adds behaviors to BehaviorList

	// SENSE Phase - sense the state of the world 
	virtual void sense(double deltaT);

	// CONTROL PHASE - given the state of the world agent determines what to do 
	virtual void control(double deltaT);

	// ACT PHASE - control is applied and agent state is updated in act phaser
	virtual void act(double deltaT);

	//given the state computes stateDot based on the agent system dynamics 
	virtual void computeDynamics(vector<vec3>& state, vector<vec3>& controlInput, vector<vec3>& stateDot, double deltaT);
	
	//updates Agent state
	virtual void updateState(float deltaT, int integratorType);



protected:
	typedef std::map<BehaviorType, Behavior*> BehaviorMap;
	
	ASkeleton* m_pSkeleton;
	AActor* m_pActor;

	//enum BehaviorType { SEEK, FLEE, ARRIVAL, DEPARTURE, AVOID, WANDER, ALIGNMENT, SEPARATION, COHESION, FLOCKING, LEADER };
	BehaviorType m_BehaviorType;
	BehaviorMap m_BehaviorList;  // maintains a list of behaviors available to agent

	vector<AActor>* m_AgentList;  // a list of all the other agents in the world
	vector<Obstacle>* m_ObstacleList;          // a list of all the obstacles in the world
	// end new

    Behavior* mpActiveBehavior;  // currently active behavior
    bool m_Active;     // determines whether the agent behavior controller is active or not
	bool mLeader;      // determines whether agent is a leader
    AJoint m_Guide;    // the position and orientation of the guide determines the position and orientation of the agent root joint
	AJoint* m_pBehaviorTarget; // target for behaviors

    // the state is comprised of world postion, body Euler angles, body velocity, and body angular velocity 
	// the world y axis is up and the camera z axis is out of the screen
	// the body y axis is up, the body z axis is forward and the body x axis is to the left
    
	vec3 m_Pos0;       // agent world position,  m_Pos0 = [x 0 z]T for the 2D planar case
	vec3 m_Vel0;       // agent world velocity,  m_Vel0 = [Vx 0 Vz]T for the 2D planar case
	vec3 m_lastVel0;   // world velocity at last time step
	vec3 m_Euler;      // agent body Euler angles, m_Euler = [0 theta 0]T for the 2D planar case
	vec3 m_VelB;       // agent body axes velocity, m_VelB = [Vx 0 Vz]T for the 2D planar case
	vec3 m_AVelB;      // agent body axes angular velocity,  m_AVelB = [0 thetaDot 0]T for the 2D planar case
	
	int m_stateDim = 4; 
	vector<vec3> m_state;
		// m_state[0] = m_Pos0 = [x 0 z]T for the 2D planar case
		// m_state[1] = m_Euler = [ 0 theta 0]T for the 2D planar case 
		// m_state[2] = m_VelB = [ Vx 0 Vz]T for the 2D planar case
		// m_state[3] = m_AVelB =  [ 0 thetaDot 0]T for the 2D planar case 

	int m_stateDotDim = 4;
	vector<vec3> m_stateDot;
		// m_stateDot[0] = m_Vel0 = [ Vx 0 Vz]T for the 2D planar case
		// m_stateDot[1] = m_AVelB = [ 0 thetaDot 0]T for the 2D planar case
		// m_stateDot[2] = body acceleration = [ accelx 0 accelz]T for the 2D planar case
		// m_stateDot[3] = body angular acceleration = = [ 0 thetaDot2 0]T for the 2D planar case

    // Inputs:  forces and torques
	vec3 m_force;
	// m_force[0] = body force in the x direction
	// m_force[1] = body force in the y direction (for the 2D planar case = 0.0)
	// m_force[2] = body force in the z direction

	vec3 m_torque;
	// m_torque[0] = body torque about the x axis (for 2D planar case = 0.0)
	// m_torque[1] = body torque about the y axis 
	// m_torque[2] = body torque about the z axis (for 2D planar case = 0.0)

	int m_controlDim = 2;
	vector<vec3> m_controlInput;
		// m_controlInput[0] = m_force = [force_x 0 force_z]T for the 2D planar case
		// m_controlInput[1] = m_torque = [0 torque_y 0]T for the 2D planar case

    // Control inputs: speed and heading direction
    enum {USPEED, UHEADING};
	vec3 m_Vdesired;   // desired world velocity
	double m_vd;
    double m_thetad;
	double m_lastThetad;

public:
    static double gKNeighborhood;
    static double gAgentRadius;

    static double gMass;
    static double gInertia;
    static double gMaxSpeed;
    static double gMaxAngularSpeed;
    static double gMaxForce;
    static double gMaxTorque;

    //Velocity controller gains: force = m * Kv * (Vd - V)
    static double gVelKv;  // Velocity Kv gain

    //Heading controller gains: torque = I * ( -Kv * thetaDot - Kp * theta + Kp * thetad)
    static double gOriKv;  // Orientation Kv gain
    static double gOriKp;  // Orientation Kp gain

    //Behavior gains. See comments in cpp file for details
    static double KArrival;
    static double KDeparture;
    static double KNoise;
    static double KWander;
    static double KAvoid;
    static double TAvoid;
    static double RNeighborhood;
    static double KSeparation;
    static double KAlignment;
    static double KCohesion;
};

#endif