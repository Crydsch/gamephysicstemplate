#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

RigidBodySystemSimulator::RigidBodySystemSimulator(int maxCountBodies)
{
	m_iMaxCountBodies = maxCountBodies;
	m_iCountBodies = 0;
	m_Bodies = (Body*)calloc(m_iMaxCountBodies, sizeof(Body)); // default 1000*(7*3*4+4*4+4)=102KB

	m_iTestCase = 0;

	reset();
}

RigidBodySystemSimulator::~RigidBodySystemSimulator()
{
	free(m_Bodies);
}

void RigidBodySystemSimulator::reset()
{
	//std::cerr << "reset()" << std::endl;

	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };

	// Note: No need to actually reset every body
	//  Only the ones in [0,count[ are valid anyway
	m_iCountBodies = 0;
	m_iCountBodiesTarget = 0;

	m_fBounciness = 1.0f;

	m_constantAcceleration = Vec3(0.0f);
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	// List testcases as strings
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

// Note: This is defined in main.cpp (as well as the vs-testcases)
//       We use this to programatically change the active testcase.
extern int g_iTestCase;

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	//std::cerr << "notifyCaseChanged(" << testCase << ")" << std::endl;
	// Called to initialize new TestCase

	reset();

	m_iTestCase = testCase;
	
	RigidBodySystemSimulator* rbss;
	Vec3 position, linear_velocity, angular_velocity, posWorld1, posWorld2, posRel1, posRel2, velWorld1, velWorld2;
	switch (m_iTestCase)
	{
	case 0: /* Demo 1 */
		// A simple one-step test
		// We construct our own Simulator similar to the Visual Studio Testcases

		rbss = new RigidBodySystemSimulator();
		rbss->addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		rbss->setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
		rbss->applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
		rbss->simulateTimestep(2.0f);

		position = rbss->getPositionOfRigidBody(0);
		linear_velocity = rbss->getLinearVelocityOfRigidBody(0);
		angular_velocity = rbss->getAngularVelocityOfRigidBody(0);

		posWorld1 = Vec3(0.3f, 0.5f, 0.25f);
		posWorld2 = Vec3(-0.3f, -0.5f, -0.25f);
		posRel1 = posWorld1 - rbss->getPositionOfRigidBody(0);
		posRel2 = posWorld2 - rbss->getPositionOfRigidBody(0);
		velWorld1 = linear_velocity + cross(angular_velocity, posRel1);
		velWorld2 = linear_velocity + cross(angular_velocity, posRel2);

		std::cout << "\nSimulation State after one step (h=2.0f)\n" <<
			"  Body[0].linear_velocity = " << linear_velocity.toString() << "\n" <<
			"  Body[0].angular_velocity = " << angular_velocity.toString() << "\n" <<
			"  Vec3(0.3f, 0.5f, 0.25f).velocity = " << velWorld1.toString() << "\n" <<
			"  Vec3(-0.3f, -0.5f, -0.25f).velocity = " << velWorld2.toString() << std::endl;
		delete rbss;
		rbss = NULL;

		// Demo 1 is a one shot => once done we switch to another testcase
		g_iTestCase = 1;
		notifyCaseChanged(g_iTestCase);
		break;

	case 1: /* Demo 2 */
		// Simple single body simulation
		// Run and render the simulation

		addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f, 0.6f, 0.5f), 2.0f);
		setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
		applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));

		break;

	case 2: /* Demo 3 */
		// Two-rigid-body collision scene
		// Run and render the simulation

		// first box (0.5,0,0.5) rather flat, static
		addRigidBody(Vec3(0.5f, 0.0f, 0.5f), Vec3(1.0f, 0.3f, 1.0f), 2.0f, true);

		// first box (0,1,0) small, rotated to face a corner down, pushed downwards
		addRigidBody(Vec3(0.0f, 1.0f, 0.0f), Vec3(0.3f, 0.3f, 0.3f), 2.0f);
		setOrientationOf(1, Quat((float)(M_PI) * 0.25f, (float)(M_PI) * 0.25f, (float)(M_PI) * 0.25f));
		applyForceOnBody(1, Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f));

		break;

	case 3: /* Demo 4*/
		// Complex simulation
		// Run and render the simulation

		// add floor
		addRigidBody(Vec3(0.0f, -1.0f, 0.0f), Vec3(10.0f, 0.1f, 10.0f), 1000.0f);

		// enable gravity
		m_constantAcceleration = Vec3(0.0f, -1.0f, 0.0f);

		// let the bodies hit the floooooooor!
		m_iCountBodiesTarget = 20;

		break;

	default:
		std::cerr << "Unknown TestCase selected: " << testCase << std::endl;
		break;
	}
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddSeparator(DUC->g_pTweakBar, "", "");
	TwAddVarRW(DUC->g_pTweakBar, "Bounciness Factor", TW_TYPE_FLOAT, &m_fBounciness, "step=0.1 min=0.0 max=1.0");

	switch (m_iTestCase)
	{
	case 0: // Demo 1: Nothing rendered
		break;
	case 1: // Demo 2
		break;
	case 2: // Demo 3
		break;
	case 3: // Demo 4
		TwAddVarRO(DUC->g_pTweakBar, "Bodies Count", TW_TYPE_INT32, &m_iCountBodies, "");
		TwAddVarRW(DUC->g_pTweakBar, "Bodies Count Target", TW_TYPE_INT32, &m_iCountBodiesTarget, "step=1 min=1");

		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	// We always render all boxes

	const Vec3 turquoise = Vec3(64.0f / 255.0f, 224.0f / 255.0f, 208.0f / 255.0f);
	const Vec3 darkturquoise = Vec3(0.0f, 206.0f / 255.0f, 209.0f / 255.0f);

	const Vec3 lilac = Vec3(0x7D, 0x4F, 0xB0) / 255.0f; // #7D4FB0
	const Vec3 lightlilac = Vec3(0x90, 0x68, 0xBB) / 255.0f; // #9068BB
	//const Vec3 green = Vec3(0x7B, 0xBB, 0x44) / 255.0f; // #7BBB44
	//const Vec3 red = Vec3(0xBB, 0x48, 0x44) / 255.0f; // ##BB4844
	//const Vec3 blue = Vec3(0x44, 0xB7, 0xBB) / 255.0f; // ##44B7BB

	int i = 0;

	if (m_iTestCase == 3) {
		// We draw the floor in a different color
		Body& b = m_Bodies[i];
		Mat4 scale, rot, transl, obj2world;
		scale.initScaling(b.size.x, b.size.y, b.size.z);
		rot = b.orientation.getRotMat();
		transl.initTranslation(b.position.x, b.position.y, b.position.z);
		obj2world = scale * rot * transl;

		DUC->setUpLighting(Vec3(0.0f), 0.4 * lightlilac, 100, 0.6 * lilac);
		DUC->drawRigidBody(obj2world);

		i++;
	}

	// Note: Boxes are drawn as unit-cubes around (0,0,0)
	//       our world matrix has to rotate, scale and translate accordingly.
	for (; i < m_iCountBodies; i++) {
		Body &b = m_Bodies[i];
		Mat4 scale, rot, transl, obj2world;
		scale.initScaling(b.size.x, b.size.y, b.size.z);
		rot = b.orientation.getRotMat();
		transl.initTranslation(b.position.x, b.position.y, b.position.z);
		obj2world = scale * rot * transl;
		
		DUC->setUpLighting(Vec3(0.0f), 0.4 * turquoise, 100, 0.6 * darkturquoise);
		DUC->drawRigidBody(obj2world);
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Attention: Called before simulateTimestep when running through simulator
	//            NOT called when running through tests (but tests do not use external forces ...)

	// => Best not to rely on this function being called or not and just do our own internal calculations for external forces!
}

Mat4 constructIIIT(Vec3 iiit, Quat orientation) {
	// calc CURRENT inverse inertia tensor
	Mat4 i0, rot, rotT;
	// construct I0^-1 matrix
	i0.initScaling(iiit.x, iiit.y, iiit.z);
	// get rotation matrix
	rotT = rot = orientation.getRotMat();
	// transpose rotation matrix
	rotT.transpose();
	// calc inertia tensor
	return rotT * i0 * rot;
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	//Point2D mouseDiff;
	//Vec3 user_force;
	//// find a proper scale!
	//float inputScale = 0.5f;
	
	//std:cerr << "\t pos=" << m_Bodies[1].position << "\t rot=" << m_Bodies[1].orientation << std::endl;
	/*Quat cpy = Quat(m_Bodies[1].orientation);
	cpy.unit();
	std:cerr << "diff=" << (m_Bodies[1].orientation.normSq() - cpy.normSq()) << std::endl;*/

	// check collisions
	// TODO optimize
	for (int i = 0; i < m_iCountBodies; i++) {
		Body& b1 = m_Bodies[i];

		Mat4 scale, rot, transl, b1_obj2world, b2_obj2world;
		scale.initScaling(b1.size.x, b1.size.y, b1.size.z);
		rot = b1.orientation.getRotMat();
		transl.initTranslation(b1.position.x, b1.position.y, b1.position.z);
		b1_obj2world = scale * rot * transl;

		for (int o = i+1; o < m_iCountBodies; o++) {
			Body& b2 = m_Bodies[o];

			scale.initScaling(b2.size.x, b2.size.y, b2.size.z);
			rot = b2.orientation.getRotMat();
			transl.initTranslation(b2.position.x, b2.position.y, b2.position.z);
			b2_obj2world = scale * rot * transl;


			CollisionInfo col = checkCollisionSAT(b1_obj2world, b2_obj2world);

			if (col.isValid) {
				// calc velocities at collision point
				Vec3 b1_relcolpos = col.collisionPointWorld - b1.position;
				Vec3 b1_colvel = b1.linear_velocity + cross(b1.angular_velocity, b1_relcolpos);
				Vec3 b2_relcolpos = col.collisionPointWorld - b2.position;
				Vec3 b2_colvel = b2.linear_velocity + cross(b2.angular_velocity, b2_relcolpos);

				// calc relvative velocity
				Vec3 relcolvel = b1_colvel - b2_colvel;
				
				if (dot(col.normalWorld, relcolvel) < 0) { // actually colliding
					/* calc impulse */

					// calc CURRENT inverse inertia tensor
					Mat4 b1_tensor = constructIIIT(b1.iiit, b1.orientation);
					Mat4 b2_tensor = constructIIIT(b2.iiit, b2.orientation);

					Vec3 b1_rot_change = cross(b1_tensor * (cross(b1_relcolpos, col.normalWorld)), b1_relcolpos);
					Vec3 b2_rot_change = cross(b2_tensor * (cross(b2_relcolpos, col.normalWorld)), b2_relcolpos);

					Real J = (-1.0 * (1.0 + (Real)m_fBounciness) * dot(relcolvel, col.normalWorld)) / ((Real)b1.invMass + (Real)b2.invMass + dot(b1_rot_change + b2_rot_change, col.normalWorld));


					/* apply impuls */
					b1.linear_velocity += (col.normalWorld * J) / b1.mass;
					b2.linear_velocity -= (col.normalWorld * J) / b2.mass;

					b1.angular_momentum += cross(b1_relcolpos, col.normalWorld * J);
					b2.angular_momentum -= cross(b2_relcolpos, col.normalWorld * J);
				}
				// elseif >0 already separating
				// elseif ==0 sliding contact

			}
		}
	}
	

	// !! ONLY ONE LOOP !! =D
	for (int i = 0; i < m_iCountBodies; i++) {
		// external force/torque is already accumulated

		/* update position */
		m_Bodies[i].position += timeStep * m_Bodies[i].linear_velocity;

		/* update linear_velocity */
		Vec3 acceleration = (m_Bodies[i].force / m_Bodies[i].mass) + m_constantAcceleration;
		m_Bodies[i].linear_velocity += timeStep * acceleration;
		// clear forces
		m_Bodies[i].force = Vec3(0.0f);


		/* update angular_momentum */
		m_Bodies[i].angular_momentum += timeStep * m_Bodies[i].torque;
		// clear torque
		m_Bodies[i].torque = Vec3(0.0f);

		/* calc CURRENT inverse inertia tensor */
		Mat4 tensor = constructIIIT(m_Bodies[i].iiit, m_Bodies[i].orientation);
		////  using the old orientation
		//Mat4 i0, rot, rotT, tensor;
		//// construct I0^-1 matrix
		//i0.initScaling(m_Bodies[i].iiit.x, m_Bodies[i].iiit.y, m_Bodies[i].iiit.z);
		//// get rotation matrix
		//rotT = rot = m_Bodies[i].orientation.getRotMat();
		//// transpose rotation matrix
		//rotT.transpose();
		//// calc inertia tensor
		//tensor = rotT * i0 * rot;

		/* update orientation */
		//  using old angular_velocity
		// const Real s1 = 0;
		const Vec3 v1 = m_Bodies[i].angular_velocity;
		const Real s2 = m_Bodies[i].orientation.w;
		const Vec3 v2 = Vec3(m_Bodies[i].orientation.x, m_Bodies[i].orientation.y, m_Bodies[i].orientation.z);

		const Real s3 = -1.0f * (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
		const Vec3 v3 = s2 * v1 + cross(v1, v2);
		
		m_Bodies[i].orientation += Quat(v3.x, v3.y, v3.z, s3) * (0.5 * timeStep);
		m_Bodies[i].orientation.unit(); // renormalize

		/* update angular_velocity */
		//  using angular momentum & inertia tensor
		m_Bodies[i].angular_velocity = tensor * m_Bodies[i].angular_momentum;
	}

}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	//std::cerr << "onClick(" << x << "," << y << ")" << std::endl;
	// Called WHILE mouse is clicked

	m_trackmouse = { x, y };
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	//std::cerr << "onMouse(" << x << "," << y << ")" << std::endl;
	// Called WHILE mouse is NOT clicked

	m_oldtrackmouse = { x, y };
	m_trackmouse = { x, y };
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	// accumulate linear force
	m_Bodies[i].force += force;

	// accumulate torque
	const Vec3 rel_pos = loc - m_Bodies[i].position;
	m_Bodies[i].torque += cross(rel_pos, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, float mass, bool isFixed) {
	// a rigid body has to be in a valid state after being added
	
	// Sanity checks
	if (m_iCountBodies >= m_iMaxCountBodies) {
		std::cerr << "Too many mass points! (MAX = " << m_iMaxCountBodies << ")" << std::endl;
		exit(1);
	}
	if (mass == 0.0f) {
		std::cerr << "Invalid mass! mass == 0" << std::endl;
		exit(1);
	}
	if (mass <= 0.0f) {
		std::cerr << "Negative mass! mass < 0" << std::endl;
		exit(1);
	}
	if (size.x <= 0.0f || size.y <= 0.0f || size.z <= 0.0f) {
		std::cerr << "Invalid size! size <= 0" << std::endl;
		exit(1);
	}

	m_Bodies[m_iCountBodies].size = size;

	m_Bodies[m_iCountBodies].position = position;
	m_Bodies[m_iCountBodies].linear_velocity = Vec3(0.0f);
	m_Bodies[m_iCountBodies].force = Vec3(0.0f);
	m_Bodies[m_iCountBodies].mass = mass;

	if (isFixed) { // tricks for fixed bodies
		m_Bodies[m_iCountBodies].invMass = 0.0f;
		m_Bodies[m_iCountBodies].iiit = Vec3(0.0f);
	}
	else {
		m_Bodies[m_iCountBodies].invMass = 1.0f / mass;

		// Note: We use the analytical moment of inertia
		//       We handle x=width y=height z=depth
		// Ref.: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
		m_Bodies[m_iCountBodies].iiit = Vec3(mass / 12.0f);
		m_Bodies[m_iCountBodies].iiit.x *= (size.y * size.y + size.z * size.z);
		m_Bodies[m_iCountBodies].iiit.y *= (size.x * size.x + size.z * size.z);
		m_Bodies[m_iCountBodies].iiit.z *= (size.x * size.x + size.y * size.y);
		// invert
		m_Bodies[m_iCountBodies].iiit.x = 1 / m_Bodies[m_iCountBodies].iiit.x;
		m_Bodies[m_iCountBodies].iiit.y = 1 / m_Bodies[m_iCountBodies].iiit.y;
		m_Bodies[m_iCountBodies].iiit.z = 1 / m_Bodies[m_iCountBodies].iiit.z;
	}
	
	m_Bodies[m_iCountBodies].orientation = Quat(0.0f, 0.0f, 0.0f);
	m_Bodies[m_iCountBodies].angular_velocity = Vec3(0.0f);
	m_Bodies[m_iCountBodies].angular_momentum = Vec3(0.0f);
	m_Bodies[m_iCountBodies].torque = Vec3(0.0f);

	m_iCountBodies++;
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	m_Bodies[i].orientation = orientation.unit();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	m_Bodies[i].linear_velocity = velocity;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return m_iCountBodies;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	return m_Bodies[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	return m_Bodies[i].linear_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	return m_Bodies[i].angular_velocity;
}
