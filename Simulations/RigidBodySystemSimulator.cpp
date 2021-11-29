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
	m_newBodyPrepared = false;
	m_iSelectedBody = -1;

	m_linearDamping = 0.0f;
	m_angularDamping = 0.0f;

	m_inputScale = 0.01f;
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
		addRigidBody(Vec3(0.5f, 0.0f, 0.5f), Vec3(1.0f, 0.3f, 1.0f), 2.0f);

		// first box (0,1,0) small, rotated to face a corner down, pushed downwards
		addRigidBody(Vec3(0.0f, 1.0f, 0.0f), Vec3(0.3f, 0.3f, 0.3f), 2.0f);
		setOrientationOf(1, Quat((float)(M_PI) * 0.25f, (float)(M_PI) * 0.25f, (float)(M_PI) * 0.25f));
		applyForceOnBody(1, Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f));

		break;

	case 3: /* Demo 4*/
		// Complex simulation
		// Run and render the simulation

		// add floor
		addRigidBody(Vec3(0.0f, -1.0f, 0.0f), Vec3(10.0f, 1.1f, 10.0f), 1337.0f, true);

		// enable gravity
		//m_constantAcceleration = Vec3(0.0f, -0.05f, 0.0f);

		// take some energy out of the system
		m_fBounciness = 0.7f;
		m_linearDamping = 0.01f;
		m_angularDamping = 0.01f;

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
	case 2: // Demo 3
		TwAddVarRW(DUC->g_pTweakBar, "Selected Body", TW_TYPE_INT32, &m_iSelectedBody, "step=1 min=0");
		TwAddVarRW(DUC->g_pTweakBar, "Input Scale", TW_TYPE_FLOAT, &m_inputScale, "step=0.01 min=0.001");
		break;
	case 3: // Demo 4
		TwAddVarRO(DUC->g_pTweakBar, "Bodies Count", TW_TYPE_INT32, &m_iCountBodies, "");
		TwAddVarRW(DUC->g_pTweakBar, "Bodies Count Target", TW_TYPE_INT32, &m_iCountBodiesTarget, "step=1 min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Selected Body", TW_TYPE_INT32, &m_iSelectedBody, "step=1 min=1");

		TwAddVarRW(DUC->g_pTweakBar, "Linear Damping", TW_TYPE_FLOAT, &m_linearDamping, "step=0.01 min=0.0");
		TwAddVarRW(DUC->g_pTweakBar, "Angular Damping", TW_TYPE_FLOAT, &m_angularDamping, "step=0.01 min=0.0");
		TwAddVarRW(DUC->g_pTweakBar, "Input Scale", TW_TYPE_FLOAT, &m_inputScale, "step=0.01 min=0.001");

		//TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &(m_constantAcceleration.y), "step=0.01 min=0.0");
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
	const Vec3 red = Vec3(0xBB, 0x48, 0x44) / 255.0f; // ##BB4844
	const Vec3 lightred = Vec3(0xBD, 0x63, 0x60) / 255.0f; // ##BD6360
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
		
		if (i == m_iSelectedBody) {
			DUC->setUpLighting(Vec3(0.0f), 0.4 * lightred, 100, 0.6 * red);
		}
		else {
			DUC->setUpLighting(Vec3(0.0f), 0.4 * turquoise, 100, 0.6 * darkturquoise);
		}
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
	//i0.value[3][3] = 0.0;
	// get rotation matrix
	rotT = rot = orientation.getRotMat();
	// transpose rotation matrix
	rotT.transpose();
	// calc inertia tensor
	return rotT * i0 * rot; // TODO is this the right way round?  this looks better...
	return rot * i0 * rotT; // TODO is this the right way round?
}

Mat4 constructObj2Wold(Vec3 position, Vec3 size, Quat orientation) {
	Mat4 scale, rot, transl, b1_obj2world, b2_obj2world;
	scale.initScaling(size.x, size.y, size.z);
	rot = orientation.getRotMat();
	transl.initTranslation(position.x, position.y, position.z);
	return scale * rot * transl;
}

std::default_random_engine generator;
std::uniform_real_distribution<float> size_dist(0.1f, 0.4f);
std::uniform_real_distribution<float> rot_dist(0.0f, M_PI * 1.0f);
std::uniform_real_distribution<float> mass_dist(0.5f, 2.0f);

void RigidBodySystemSimulator::spawnBody() {

	if (!m_newBodyPrepared) {
		m_newBody_size = Vec3(size_dist(generator), size_dist(generator), size_dist(generator));
		m_newBody_orientation = Quat(rot_dist(generator), rot_dist(generator), rot_dist(generator));
		m_newBody_mass = mass_dist(generator);
		m_newBodyPrepared = true;
	}

	// try to spawn new body
	const Vec3 spawnPos = Vec3(0.0f, 2.0f, 0.0f);
	
	Mat4 newb_obj2world = constructObj2Wold(spawnPos, m_newBody_size, m_newBody_orientation);

	bool safeToSpawn = true;
	for (int i = 0; i < m_iCountBodies; i++) {
		Body& b = m_Bodies[i];
		Mat4 b1_obj2world = constructObj2Wold(b.position, b.size, b.orientation);

		CollisionInfo col = checkCollisionSAT(b1_obj2world, newb_obj2world);
		if (col.isValid) {
			safeToSpawn = false;
			break;
		}
	}

	if (safeToSpawn) {
		addRigidBody(spawnPos, m_newBody_size, m_newBody_mass);
		setOrientationOf(m_iCountBodies - 1, m_newBody_orientation);
		setVelocityOf(m_iCountBodies - 1, Vec3(0.0f, -0.2f, 0.0f));
		m_newBodyPrepared = false;
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	// user interaction
	Point2D mouseDiff;
	Vec3 user_force;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (m_iSelectedBody >= 0 && m_iSelectedBody < m_iCountBodies
		&& (mouseDiff.x != 0 || mouseDiff.y != 0))
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		user_force = worldViewInv.transformVectorNormal(inputView);
		user_force = user_force * m_inputScale;

		applyForceOnBody(m_iSelectedBody, m_Bodies[m_iSelectedBody].position, user_force);
	}

	if (m_iTestCase == 3) {
		m_iCountBodiesTarget = min(m_iCountBodiesTarget, m_iMaxCountBodies);
		if (m_iCountBodies < m_iCountBodiesTarget) {
			spawnBody(); // try to spawn a body (success only if there is space)
		}
	}

	// !! ONLY ONE LOOP !! =D
	for (int i = 0; i < m_iCountBodies; i++) {
		// external force/torque is already accumulated
		// damping
		//  F_damp(t)=-m_fDamping*v(t)
		m_Bodies[i].force += -m_linearDamping * m_Bodies[i].linear_velocity;
		m_Bodies[i].torque += -m_angularDamping * m_Bodies[i].angular_velocity;

		/* update position */
		m_Bodies[i].position += timeStep * m_Bodies[i].linear_velocity;

		/* update linear_velocity */
		Vec3 acceleration = (m_Bodies[i].force * m_Bodies[i].invMass) + m_constantAcceleration;
		// Note: Using bool<->int magic here to avoid branching
		m_Bodies[i].linear_velocity += (1 - m_Bodies[i].isFixed) * timeStep * acceleration;
		// clear forces
		m_Bodies[i].force = Vec3(0.0f);


		/* update angular_momentum */
		m_Bodies[i].angular_momentum += timeStep * m_Bodies[i].torque;
		// clear torque
		m_Bodies[i].torque = Vec3(0.0f);

		/* calc CURRENT inverse inertia tensor */
		//  using the old orientation
		Mat4 tensor = constructIIIT(m_Bodies[i].iiit, m_Bodies[i].orientation);

		/* update orientation */
		//  using old angular_velocity
		// const Real s1 = 0;
		const Vec3 v1 = m_Bodies[i].angular_velocity;
		const Real s2 = m_Bodies[i].orientation.w;
		const Vec3 v2 = Vec3(m_Bodies[i].orientation.x, m_Bodies[i].orientation.y, m_Bodies[i].orientation.z);

		const Real s3 = -1.0f * (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
		const Vec3 v3 = s2 * v1 + cross(v1, v2);
		
		m_Bodies[i].orientation += Quat(v3.x, v3.y, v3.z, s3) * (0.5 * timeStep);
		m_Bodies[i].orientation = m_Bodies[i].orientation.unit(); // renormalize

		/* update angular_velocity */
		//  using angular momentum & inertia tensor
		m_Bodies[i].angular_velocity = tensor * m_Bodies[i].angular_momentum;
	}


	// check collisions
	for (int i = 0; i < m_iCountBodies; i++) {
		Body& b1 = m_Bodies[i];
		Mat4 b1_obj2world = constructObj2Wold(b1.position, b1.size, b1.orientation);

		for (int o = i + 1; o < m_iCountBodies; o++) {
			Body& b2 = m_Bodies[o];
			Mat4 b2_obj2world = constructObj2Wold(b2.position, b2.size, b2.orientation);

			CollisionInfo col = checkCollisionSAT(b1_obj2world, b2_obj2world);

			if (col.isValid) {
				
				/*if (col.collisionPointWorld.y < 0.0) {
					col.collisionPointWorld.y = 0.0;
				}*/

				// calc velocities at collision point
				Vec3 b1_relcolpos = col.collisionPointWorld - b1.position;
				Vec3 b1_colvel = b1.linear_velocity + cross(b1.angular_velocity, b1_relcolpos);
				Vec3 b2_relcolpos = col.collisionPointWorld - b2.position;
				Vec3 b2_colvel = b2.linear_velocity + cross(b2.angular_velocity, b2_relcolpos);

				// calc relvative velocity
				Vec3 relcolvel = b1_colvel - b2_colvel;

				float _dot = dot(col.normalWorld, relcolvel);
				if (_dot < 0.0f) { // actually colliding
				   /* calc impulse */
					//std::cerr << "depth=" << col.depth << std::endl;

					// calc CURRENT inverse inertia tensor
					Mat4 b1_tensor = constructIIIT(b1.iiit, b1.orientation);
					Mat4 b2_tensor = constructIIIT(b2.iiit, b2.orientation);

					Vec3 b1_rot_change = cross(b1_tensor * (cross(b1_relcolpos, col.normalWorld)), b1_relcolpos);
					Vec3 b2_rot_change = cross(b2_tensor * (cross(b2_relcolpos, col.normalWorld)), b2_relcolpos);

					Real J = (-1.0 * (1.0 + (Real)m_fBounciness) * dot(relcolvel, col.normalWorld)) / ((Real)b1.invMass + (Real)b2.invMass + dot(b1_rot_change + b2_rot_change, col.normalWorld));

					/* apply impuls */
					b1.linear_velocity += (col.normalWorld * J) * (Real)b1.invMass;
					b2.linear_velocity -= (col.normalWorld * J) * (Real)b2.invMass;

					b1.angular_momentum += cross(b1_relcolpos, col.normalWorld * J);
					b2.angular_momentum -= cross(b2_relcolpos, col.normalWorld * J);



				}
				// elseif >0 already separating
				// elseif ==0 sliding contact

				// x = len(depth)
				// x = t*v v=x/t
				// v = t*a a=x/t/t
				// F = m*(a-g)
				// g = (0,-1,0)

				if (m_iTestCase == 3) { // we only do this in our own complex demo
				// push bodies apart to fix ghosting through each other
					const Real accepable_penetration = 0.00f;
					if (col.depth > accepable_penetration) {
						//const float necessary_acceleration = col.depth * norm(m_constantAcceleration) * 0.99f;
						if (b2.isFixed) {
							//applyForceOnBody(i, b1.position, col.normalWorld * necessary_acceleration * (1.0f / b1.invMass));
							//applyForceOnBody(i, b1.position, col.normalWorld* ((Real)col.depth - accepable_penetration));
							b1.position += col.normalWorld * ((Real)col.depth - accepable_penetration);
						}
						else if (b1.isFixed) {
							//applyForceOnBody(o, b2.position, -col.normalWorld * necessary_acceleration * (1.0f / b2.invMass));
							//applyForceOnBody(o, b2.position, -col.normalWorld * ((Real)col.depth - accepable_penetration));
							b2.position -= col.normalWorld * ((Real)col.depth - accepable_penetration);
						}
						else {
							//applyForceOnBody(i, b1.position, col.normalWorld * necessary_acceleration * (1.0f / b1.invMass));
							//applyForceOnBody(o, b2.position, -col.normalWorld * necessary_acceleration * (1.0f / b2.invMass));
							//applyForceOnBody(i, b1.position, col.normalWorld * ((Real)col.depth - accepable_penetration));
							//applyForceOnBody(o, b2.position, -col.normalWorld * ((Real)col.depth - accepable_penetration));
							b1.position += col.normalWorld * (0.5 * ((Real)col.depth - accepable_penetration));
							b2.position -= col.normalWorld * (0.5 * ((Real)col.depth - accepable_penetration));
						}
					}
				}

				

			}
		}
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
	//m_Bodies[m_iCountBodies].mass = mass;

	m_Bodies[m_iCountBodies].isFixed = isFixed;
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
	
	m_Bodies[m_iCountBodies].orientation = Quat(0.0f, 0.0f, 0.0f).unit();
	m_Bodies[m_iCountBodies].angular_velocity = Vec3(0.0f);
	m_Bodies[m_iCountBodies].angular_momentum = Vec3(0.0f);
	m_Bodies[m_iCountBodies].torque = Vec3(0.0f);

	if (m_iSelectedBody == -1) {
		if (m_iTestCase == 3) {
			m_iSelectedBody = 1;
		}
		else {
			m_iSelectedBody = m_iCountBodies;
		}
	}

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
