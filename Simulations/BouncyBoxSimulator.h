#ifndef BOUNCYBOXSIMULATOR_h
#define BOUNCYBOXSIMULATOR_h
#include "Simulator.h"

#define MAX_BODIES 2000
#define MAX_MASS_POINTS (MAX_BODIES*5+1000)
#define MAX_SPRINGS (MAX_BODIES*4+1000)

#define BODY_MIN_SIZE 0.1
#define BODY_MAX_SIZE 0.5

class BouncyBoxSimulator : public Simulator{
public:
	// Constructor
	// maxCountMassPoints and maxCountSprings MUST include MPs/Springs necessary for each body:
	//  maxCountMassPoints = maxCountBodies * 5 + CUSTOM_MASSPOINTS
	//  maxCountSprings = maxCountBodies * 4 + CUSTOM_SPRINGS
	BouncyBoxSimulator(int maxCountBodies = MAX_BODIES, int maxCountMassPoints = MAX_MASS_POINTS, int maxCountSprings = MAX_SPRINGS);
	~BouncyBoxSimulator();
	
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

	// Custom Functions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, float mass, bool isFixed = false);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity); // linear velocity

	int addMassPoint(float mass, Vec3 position, Vec3 velocity, bool isFixed);
	int getNumberOfMassPoints();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	// Note: if initialLength is negative, we use the current masspoint distance as spring length
	void addSpring(float stiffness, int masspoint1, int masspoint2, float initialLength);
	int getNumberOfSprings();

private:
	// Attributes

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	bool m_renderMPS;
	bool m_renderBodies;
	UINT32 frameCount;

	float m_linearDamping;
	//float m_friction;
	float m_angularDamping;
	float m_springDamping;

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

	// Custom Attributes
	struct body_t { // actually just a box!
		//Vec3 size; // size of each side (in axis direction)
		// replaced by mass point system:
		int squishMP_FixX;
		int squishMP_FixY;
		int squishMP_FixZ;
		//int squishMP_Size; // <=  coords == current_size
		MassPoint *MP_Size;

		Vec3 position; // of the center of mass!
		Vec3 linear_velocity;
		Vec3 force; // accumulated total external force to be applied in next simulation step
		float mass; // we only consider the total mass (constant density throughout the box)
		float invMass; // calc invMass only once since mass is constant
		bool isFixed;

		Quat orientation; // rotation around center of mass aka position
		Vec3 angular_velocity;
		Vec3 angular_momentum;
		Vec3 torque; // accumulated total external torque to be applied in next simulation step
		Vec3 iiit; // inverted initial inertia tensor
		// Note: normaly this would be a 3x3 matrix. But the analytical form only has values on the diagonal, so a Vec3 is sufficent!
	};
	typedef struct body_t Body;

	// Layout: | fixed bodies | free bodies | unused slots |
	//         0             #1            #2             #3
	int m_iCountFixedBodies; // #1
	int m_iCountBodies;  // #2
	int m_iMaxCountBodies; // #3
	Body* m_Bodies;
	float m_fBounciness;

	int* m_narrowPhaseBodies; // Note: always in pairs aka [0]&[1]; [2]&[3]; ...
	int m_iCountNarrowPhaseBodies; // Note: counts individual bodies NOT pairs aka one pair==2; two pair=4; ...

	Vec3 m_constantAcceleration; // can be used for gravity (only applied to center of mass aka has no influence on rotation)
	int m_iSelectedBody;

	void spawnBody();
	void updateBodyInertiaTensor(Body& b);
	bool m_newBodyPrepared;
	Vec3 m_newBody_size;
	Quat m_newBody_orientation;
	float m_newBody_mass;
	int m_iCountBodiesTarget;

	int m_iMaxCountMassPoints;
	int m_iMaxCountSprings;
	int m_iCountMassPoints;
	int m_iCountSprings;
	MassPoint* m_MassPoints;
	Spring* m_Springs;

	float m_inputScale;

	void applyForceOnBody(Body& b, Vec3 loc, Vec3 force);
	void runMassSpringSystem(float timeStep);
	void squishBody(Body& b, const Vec3 xyz);
	void drawBox(const Body& b, const Vec3 color_spec, const Vec3 color_diff);
	Vec3 my_rotate(const Vec3 v, const Quat rot);
	void clampBodySize(Body& b);
	void collisionDetectionBroadPhase();
	inline bool collisionDetectionBroadPhaseSphereCheck(const Vec3 pos1, const Real radius1, const Vec3 pos2, const Real radius2);
	inline bool collisionDetectionBroadPhaseSphereCheck(const Body& b1, const Body& b2);
	inline void addNarrowPhaseCheck(const int idxA, const int idxB);
	};

#endif