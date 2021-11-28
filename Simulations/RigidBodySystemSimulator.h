#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// 
	RigidBodySystemSimulator(int maxCountBodies = 1000);
	~RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, float mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity); // linear velocity!

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem;
	//Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Custom Attributes
	struct body_t { // actually just a box!
		Vec3 size; // only for rendering

		Vec3 position; // of the center of mass!
		Vec3 linear_velocity;
		Vec3 force; // accumulated total external force to be applied in next simulation step
		float mass; // we only consider the total mass (constant density throughout the box)

		Quat orientation; // rotation around center of mass aka position
		Vec3 angular_velocity;
		Vec3 angular_momentum;
		Vec3 torque; // accumulated total external torque to be applied in next simulation step
		Vec3 iiit; // inverted initial inertia tensor
		// Note: normaly this would be a 3x3 matrix. But the analytical form only has values on the diagonal, so a vec3 is sufficent!
	};
	typedef struct body_t Body;

	int m_iMaxCountBodies;
	int m_iCountBodies;
	Body* m_Bodies;
	float m_fBounciness;

	};
#endif