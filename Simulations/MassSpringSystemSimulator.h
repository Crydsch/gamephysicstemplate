#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change
#define NO_INTEGRATOR 1337

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator(int maxCountMassPoints = 10000, int maxCountSprings = 10000);
	MassSpringSystemSimulator::~MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	// setMass(..) can be used in conjunction with addMassPoint(..)
	//  all subsequent added mass points will have the before set mass.
	void setMass(float mass);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	int getNumberOfMassPoints();
	// setStiffness(..) can be used in conjunction with addSpring(..)
	//  all subsequent added springs will have the before set stiffness.
	void setStiffness(float stiffness);
	// Note: If initialLength is negative, we use the current masspoint distance as spring length
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfSprings();
	// Sets a global damping factor (aka friction).
	void setDampingFactor(float damping);
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	// This adds a global external force which is applied to every
	//  every masspoint in every timestep.
	// Note: This can be used for gravity, but only if all masspoints have the same mass!
	void applyExternalForce(Vec3 force);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	// Custom Functions
	int addMassPoint(float mass, Vec3 position, Vec3 velocity, bool isFixed);
	// Note: if initialLength is negative, we use the current masspoint distance as spring length
	void addSpring(float stiffness, int masspoint1, int masspoint2, float initialLength);
	// Apply external force to only one specific masspoint
	//void applyExternalForce(int masspoint, Vec3 force);


private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Custom Attributes
	struct mass_point_t {
		float mass;
		Vec3 position;
		Vec3 velocity;
		bool isFixed;

		Vec3 force;
		Vec3 acceleration;

		// used only for midpoint integration
		Vec3 tmp_position;
		Vec3 tmp_velocity;
	};
	typedef struct mass_point_t MassPoint;

	struct spring_t {
		float stiffness;
		int masspoint1;
		int masspoint2;
		float length; // length at rest aka desired length
	};
	typedef struct spring_t Spring;

	float m_fSimTime;
	int m_iMaxCountMassPoints;
	int m_iMaxCountSprings;
	int m_iCountMassPoints;
	int m_iCountSprings;
	MassPoint *m_MassPoints;
	Spring* m_Springs;

	// Custom Functions

};
#endif