#include "BouncyBoxSimulator.h"
#include "collisionDetect.h"
#include "DrawingUtilitiesClass.h"

std::default_random_engine generator;
std::uniform_real_distribution<float> size_dist(BODY_MIN_SIZE, BODY_MAX_SIZE);
std::uniform_real_distribution<float> rot_dist(0.0f, M_PI * 1.0f);
std::uniform_real_distribution<float> mass_dist(1.0f, 2.0f);
std::uniform_real_distribution<float> dir_dist(-1.0f, 1.0f);

static int squishMP_FixO = 0;

BouncyBoxSimulator::BouncyBoxSimulator(int maxCountBodies, int maxCountMassPoints, int maxCountSprings)
{
	assert(maxCountMassPoints >= maxCountBodies * 5);
	assert(maxCountSprings >= maxCountBodies * 4);

	unsigned int SEED = time(NULL);
	std::cout << "SEED=0x" << std::hex << SEED << std::dec << std::endl;
	generator.seed(SEED);

	m_iMaxCountBodies = maxCountBodies;
	m_iCountBodies = 0;
	m_Bodies = (Body*)calloc(m_iMaxCountBodies, sizeof(Body));

	m_narrowPhaseBodies = (int*)calloc((size_t)m_iMaxCountBodies * (size_t)m_iMaxCountBodies, sizeof(int));
	m_iCountNarrowPhaseBodies = 0;

	m_iMaxCountMassPoints = maxCountMassPoints;
	m_iCountMassPoints = 0;
	m_MassPoints = (MassPoint*)calloc(m_iMaxCountMassPoints, sizeof(MassPoint));

	m_iMaxCountSprings = maxCountSprings;
	m_iCountSprings = 0;
	m_Springs = (Spring*)calloc(m_iMaxCountSprings, sizeof(Spring));

	m_iTestCase = 0;
	m_renderBodies = true;
	m_renderMPS = false;
	render_floor = true;
	reset();
}

BouncyBoxSimulator::~BouncyBoxSimulator()
{
	free(m_Bodies);
	free(m_narrowPhaseBodies);
	free(m_MassPoints);
	free(m_Springs);
}

Real my_norm(Vec3 v) {
	return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

void BouncyBoxSimulator::reset()
{
	frameCount = 0;
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };

	// Note: No need to actually reset every body
	//  Only the ones in [0,count[ are valid anyway
	m_iCountFixedBodies = 0;
	m_iCountBodies = 0;
	m_iCountBodiesTarget = 0;

	// Note: No need to actually reset every mass point/spring
	//  Only the ones in [0,count[ are valid anyway
	m_iCountMassPoints = 0;
	m_iCountSprings = 0;

	m_fBounciness = 1.0f;

	m_constantAcceleration = Vec3(0.0f);
	m_newBodyPrepared = false;
	m_iSelectedBody = -1;

	m_linearDamping = 0.0f;
	//m_friction = 0.0f;
	m_angularDamping = 0.0f;
	m_springDamping = 0.0f;

	m_inputScale = 0.01f;

	squishMP_FixO = addMassPoint(1.0f, Vec3(0.0f), Vec3(0.0f), true);
}

const char* BouncyBoxSimulator::getTestCasesStr()
{
	// List testcases as strings
	return "Bouncy Boxes, 1 Box, 2 Boxes";
}

// Note: This is defined in main.cpp (as well as the vs-testcases)
//       We use this to programatically change the active testcase.
extern int g_iTestCase;

// Called to initialize new TestCase
void BouncyBoxSimulator::notifyCaseChanged(int testCase)
{
	reset();
	m_iTestCase = testCase;
	
	switch (m_iTestCase)
	{
	case 0: /* Bouncy Boxes */
		// Complex simulation
		// Run and render the simulation
		render_floor = false;

		addRigidBody(Vec3(0.0f, -10.0f, 0.0f), Vec3(20.0f, 1.1f, 20.0f), 1.0f, true); // wall y- (floor)
		addRigidBody(Vec3(0.0f, 10.0f, 0.0f), Vec3(20.0f, 1.1f, 20.0f), 1.0f, true); // wall y+ (ceiling)
		addRigidBody(Vec3(-10.0f, 0.0f, 0.0f), Vec3(1.1f, 20.0f, 20.0f), 1.0f, true); // wall x-
		addRigidBody(Vec3(10.0f, 0.0f, 0.0f), Vec3(1.1f, 20.0f, 20.0f), 1.0f, true); // wall x+
		addRigidBody(Vec3(0.0f, 0.0f, -10.0f), Vec3(20.0f, 20.0f, 1.1f), 1.0f, true); // wall z-
		addRigidBody(Vec3(0.0f, 0.0f, 10.0f), Vec3(20.0f, 20.0f, 1.1f), 1.0f, true); // wall z+

		// take some energy out of the system
		m_fBounciness = 0.5f;
		m_linearDamping = 0.00f;
		m_angularDamping = 0.005f;
		m_springDamping = 0.9f;

		// let the bodies hit the floooooooor!
		m_iCountBodiesTarget = 850;

		break;

	case 1: /* Single squishy box */
		addRigidBody(Vec3(0.0f, 0.0f, 0.0f), Vec3(2.0f, 2.0f, 2.0f), 1.0f, false);
		squishBody(m_Bodies[0], Vec3(1.0f, 0.0f, 0.0f));

		m_springDamping = 1.2f;
		break;

	case 2: /* Two boxes hitting each other */
		// add box 1
		addRigidBody(Vec3(2.0f, 0.0f, 0.0f), Vec3(1.0f, 1.0f, 1.0f), 1.0f, false);
		setVelocityOf(0, Vec3(-1.0f, 0.0f, 0.0f));
		// add box 2
		addRigidBody(Vec3(-2.0f, 0.0f, 0.0f), Vec3(1.0f, 1.0f, 1.0f), 1.0f, false);
		setVelocityOf(1, Vec3(1.0f, 0.0f, 0.0f));

		m_fBounciness = 0.8;
		m_linearDamping = 0.01f;
		m_angularDamping = 0.01f;
		m_springDamping = 0.9f;
		
		break;

	default:
		std::cerr << "Unknown TestCase selected: " << testCase << std::endl;
		break;
	}
}

void BouncyBoxSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Render Bodies", TW_TYPE_BOOLCPP, &m_renderBodies, "");
	TwAddVarRW(DUC->g_pTweakBar, "Render MPS", TW_TYPE_BOOLCPP, &m_renderMPS, "");
	TwAddSeparator(DUC->g_pTweakBar, "", "");
	TwAddVarRW(DUC->g_pTweakBar, "Bounciness Factor", TW_TYPE_FLOAT, &m_fBounciness, "step=0.1 min=0.0 max=1.0");

	TwAddVarRO(DUC->g_pTweakBar, "Bodies Count", TW_TYPE_INT32, &m_iCountBodies, "");
	TwAddVarRW(DUC->g_pTweakBar, "Bodies Count Target", TW_TYPE_INT32, &m_iCountBodiesTarget, "step=1 min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Selected Body", TW_TYPE_INT32, &m_iSelectedBody, "step=1 min=1");

	TwAddVarRW(DUC->g_pTweakBar, "Linear Damping", TW_TYPE_FLOAT, &m_linearDamping, "step=0.01 min=0.0");
	TwAddVarRW(DUC->g_pTweakBar, "Angular Damping", TW_TYPE_FLOAT, &m_angularDamping, "step=0.01 min=0.0");
	TwAddVarRW(DUC->g_pTweakBar, "Spring Damping", TW_TYPE_FLOAT, &m_springDamping, "step=0.05 min=0.0");
	TwAddVarRW(DUC->g_pTweakBar, "Input Scale", TW_TYPE_FLOAT, &m_inputScale, "step=0.01 min=0.001");

	//TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_DOUBLE, &(m_constantAcceleration.y), "step=0.10");
}

void BouncyBoxSimulator::drawBox(const Body &b, const Vec3 color_spec, const Vec3 color_diff) {
	Mat4 scale, rot, transl, obj2world;
	Vec3 curr_size = b.MP_Size->position;
	scale.initScaling(curr_size.x, curr_size.y, curr_size.z);
	rot = b.orientation.getRotMat();
	transl.initTranslation(b.position.x, b.position.y, b.position.z);
	obj2world = scale * rot * transl;

	DUC->setUpLighting(Vec3(0.0f), 0.4 * color_spec, 100, 0.6 * color_diff);
	DUC->drawRigidBody(obj2world);
}

void BouncyBoxSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	// We always render all boxes

	const Vec3 turquoise = Vec3(64.0f / 255.0f, 224.0f / 255.0f, 208.0f / 255.0f);
	const Vec3 darkturquoise = Vec3(0.0f, 206.0f / 255.0f, 209.0f / 255.0f);
	const Vec3 lilac = Vec3(0x7D, 0x4F, 0xB0) / 255.0f; // #7D4FB0
	const Vec3 lightlilac = Vec3(0x90, 0x68, 0xBB) / 255.0f; // #9068BB
	const Vec3 green = Vec3(0x7B, 0xBB, 0x44) / 255.0f; // #7BBB44
	const Vec3 red = Vec3(0xBB, 0x48, 0x44) / 255.0f; // ##BB4844
	const Vec3 lightred = Vec3(0xBD, 0x63, 0x60) / 255.0f; // ##BD6360
	//const Vec3 blue = Vec3(0x44, 0xB7, 0xBB) / 255.0f; // ##44B7BB

	if (g_iTestCase == 0) {
		static const Vec3 cam_orig_pos = Vec3(0.0, 18.0, -30.0);
		Quat cam_rot = Quat(Vec3(0.0, 1.0, 0.0), frameCount++ * 0.0010908307825);
		Vec3 cam_pos = my_rotate(cam_orig_pos, cam_rot);
		DUC->g_camera.SetViewParams(cam_pos.toDirectXVector(), XMVectorSet(0.0, 0.0, 0.0, 1.0));
	}

	// Note: Boxes are drawn as unit-cubes around (0,0,0)
	//       our world matrix has to rotate, scale and translate accordingly.
	for (int i = 0; i < m_iCountBodies; i++) {
		Body &b = m_Bodies[i];
		
		if (m_renderBodies) {
			if (m_iTestCase == 0 && i == 0) {
				// We draw the floor in a different color
				drawBox(b, lightlilac, lilac);
			} else if (m_iTestCase == 0 && i <= 5) {
				// We draw walls not at all
				//drawBox(b, lightlilac, lilac);
			} else if (i == m_iSelectedBody) {
				drawBox(b, lightred, red);
			} else {
				drawBox(b, turquoise, darkturquoise);
			}
		}

		if (m_renderMPS) {
			Vec3 offset = b.position;

			Vec3 p0 = offset + m_MassPoints[squishMP_FixO].position;
			Vec3 p1 = offset + m_MassPoints[b.squishMP_FixX].position;
			Vec3 p2 = offset + m_MassPoints[b.squishMP_FixY].position;
			Vec3 p3 = offset + m_MassPoints[b.squishMP_FixZ].position;
			Vec3 p4 = offset + b.MP_Size->position;

			// render mass points
			DUC->setUpLighting(Vec3(), 0.4 * lightlilac, 100, 0.6 * lilac);
			DUC->drawSphere(p0, Vec3(0.05f));
			DUC->drawSphere(p1, Vec3(0.05f));
			DUC->drawSphere(p2, Vec3(0.05f));
			DUC->drawSphere(p3, Vec3(0.05f));
			DUC->drawSphere(p4, Vec3(0.05f));

			// render springs
			DUC->beginLine();
			DUC->drawLine(p0, green, p4, green);
			DUC->drawLine(p1, green, p4, green);
			DUC->drawLine(p2, green, p4, green);
			DUC->drawLine(p3, green, p4, green);
			DUC->endLine();
		}
	}
}

void BouncyBoxSimulator::externalForcesCalculations(float timeElapsed)
{
	// Attention: Called before simulateTimestep when running through simulator
	//            NOT called when running through tests (but tests do not use external forces ...)

	// => Best not to rely on this function being called or not and just do our own internal calculations for external forces!
}

void BouncyBoxSimulator::updateBodyInertiaTensor(Body& b) {
	Vec3 curr_size = b.MP_Size->position;

	// Note: We use the analytical moment of inertia
	//       We handle x=width y=height z=depth
	// Ref.: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
	b.iiit = Vec3(b.mass);
	b.iiit.x *= (curr_size.y * curr_size.y + curr_size.z * curr_size.z);
	b.iiit.y *= (curr_size.x * curr_size.x + curr_size.z * curr_size.z);
	b.iiit.z *= (curr_size.x * curr_size.x + curr_size.y * curr_size.y);
	// devide by 12
	b.iiit.x /= 12.0f;
	b.iiit.y /= 12.0f;
	b.iiit.z /= 12.0f;
	// invert
	b.iiit.x = 1 / b.iiit.x;
	b.iiit.y = 1 / b.iiit.y;
	b.iiit.z = 1 / b.iiit.z;
}

// calc CURRENT inverse inertia tensor from initial one
Mat4 constructIIIT(Vec3 iiit, Quat orientation) {
	Mat4 i0, rot, rotT;
	// construct I0^-1 matrix
	i0.initScaling(iiit.x, iiit.y, iiit.z);
	i0.value[3][3] = 0.0;
	// get rotation matrix
	rotT = rot = orientation.getRotMat();
	// transpose rotation matrix
	rotT.transpose();
	// calc inertia tensor
	return rot * i0 * rotT;
	//return rotT * i0 * rot;
}

Mat4 constructObj2World(Vec3 position, Vec3 size, Quat orientation) {
	Mat4 scale, rot, transl, b1_obj2world, b2_obj2world;
	scale.initScaling(size.x, size.y, size.z);
	rot = orientation.getRotMat();
	transl.initTranslation(position.x, position.y, position.z);
	return scale * rot * transl;
}

void BouncyBoxSimulator::spawnBody() {
	// try to spawn new body
	static const Vec3 spawnPos = Vec3(0.0f, 0.0f, 0.0f);
	static const Real maxRadius = 0.5 * my_norm(Vec3(BODY_MAX_SIZE));

	if (!m_newBodyPrepared) {
		m_newBody_size = Vec3(size_dist(generator), size_dist(generator), size_dist(generator));
		m_newBody_orientation = Quat(rot_dist(generator), rot_dist(generator), rot_dist(generator), rot_dist(generator)).unit();
		m_newBody_mass = mass_dist(generator);
		m_newBodyPrepared = true;
	}

	for (int i = m_iCountBodies - 1; i >= 0; i--) { // iterating in reverse order since last spawn is most likely to obstruct new spawn
		if (collisionDetectionBroadPhaseSphereCheck(spawnPos, maxRadius, m_Bodies[i].position, maxRadius)) {
			return; // not safe to spawn
		}
	}

	addRigidBody(spawnPos, m_newBody_size, m_newBody_mass);
	setOrientationOf(m_iCountBodies - 1, m_newBody_orientation);
	Vec3 dir = getNormalized( Vec3(dir_dist(generator), dir_dist(generator), dir_dist(generator)));
	setVelocityOf(m_iCountBodies - 1, 3.0 * dir);
	m_newBodyPrepared = false;
}

void BouncyBoxSimulator::runMassSpringSystem(float timeStep) { // TODO optimize with on loop ?
	// DO MIDPOINT INTEGRATION

	/* calc tmp_position at(t + h / 2) with OLD velocity */
	//  x(t+h/2) = x(t) + h/2 * v(t)
	for (int i = 0; i < m_iCountMassPoints; i++) {
		if (!m_MassPoints[i].isFixed) {
			m_MassPoints[i].tmp_position = m_MassPoints[i].position + (timeStep / 2.0f) * m_MassPoints[i].velocity;
		}
	}

	/* calc forces at(t + h) with OLD position */
	// reset forces
	for (int i = 0; i < m_iCountMassPoints; i++) {
		m_MassPoints[i].force = Vec3(0, 0, 0);
	}

	// calc internal forces
	//  for each spring
	for (int i = 0; i < m_iCountSprings; i++) {
		Spring& s = m_Springs[i];
		MassPoint& mp1 = m_MassPoints[s.masspoint1];
		MassPoint& mp2 = m_MassPoints[s.masspoint2];

		//if (mp1.isFixed && mp2.isFixed) { // this check is irrelevant since we don't add spring between two fixed MPs
		//	// do not apply any force on fixed points!
		//	continue;
		//}

		// calc force aka hookes law (on mp1)
		float distance = my_norm(mp1.position - mp2.position);
		Vec3 direction = getNormalized(mp1.position - mp2.position);
		Vec3 force = -s.stiffness * (distance - s.length) * direction;

		// add forces to mass points current forces
		if (mp1.isFixed && !mp2.isFixed) {
			mp2.force += -2 * force;
		}
		else if (!mp1.isFixed && mp2.isFixed) {
			mp2.force += 2 * force;
		}
		else { // both are not fixed
			mp1.force = mp1.force + force;
			mp2.force = mp2.force - force;
		}
	}

	// calc external forces
	//  damping  F_damp(t)=-m_fDamping*v(t)
	for (int i = 0; i < m_iCountMassPoints; i++) {
		m_MassPoints[i].force += -m_springDamping * m_MassPoints[i].velocity;
	}

	/* calc acceleration at(t + h) with forces at(t + h) */
	for (int i = 0; i < m_iCountMassPoints; i++) {
		if (!m_MassPoints[i].isFixed) {
			m_MassPoints[i].acceleration = m_MassPoints[i].force;
			m_MassPoints[i].acceleration.safeDivide(m_MassPoints[i].mass);
			// add constant acceleration (gravity)
			//m_MassPoints[i].acceleration += m_constantAcceleration;
		}
	}

	/* calc tmp_velocity at(t + h / 2) with acceleration at(t + h) */
	//  v(t+h/2) = v(t) + h/2 * a(t)
	for (int i = 0; i < m_iCountMassPoints; i++) {
		if (!m_MassPoints[i].isFixed) {
			m_MassPoints[i].tmp_velocity = m_MassPoints[i].velocity + (timeStep / 2.0f) * m_MassPoints[i].acceleration;
		}
	}

	/* update position with tmp_velocity */
	//  x(t+h) = x(t) + h * v(t+h/2)
	for (int i = 0; i < m_iCountMassPoints; i++) {
		if (!m_MassPoints[i].isFixed) {
			m_MassPoints[i].position += timeStep * m_MassPoints[i].tmp_velocity;
		}
	}

	/* calc forces at(t + h / 2) with tmp_position */
	// reset forces
	for (int i = 0; i < m_iCountMassPoints; i++) {
		m_MassPoints[i].force = Vec3(0, 0, 0);
	}

	// calc internal forces
	//  for each spring
	for (int i = 0; i < m_iCountSprings; i++) {
		Spring& s = m_Springs[i];
		MassPoint& mp1 = m_MassPoints[s.masspoint1];
		MassPoint& mp2 = m_MassPoints[s.masspoint2];

		//if (mp1.isFixed && mp2.isFixed) { // this check is irrelevant since we don't add spring between two fixed MPs
		//	// do not apply any force on fixed points!
		//	continue;
		//}

		// calc force aka hookes law (on mp1)
		float distance = my_norm(mp1.tmp_position - mp2.tmp_position);
		Vec3 direction = getNormalized(mp1.tmp_position - mp2.tmp_position);
		Vec3 force = -s.stiffness * (distance - s.length) * direction;

		// add forces to mass points current forces
		if (mp1.isFixed && !mp2.isFixed) {
			mp2.force += -2 * force;
		}
		else if (!mp1.isFixed && mp2.isFixed) {
			mp2.force += 2 * force;
		}
		else { // both are not fixed
			mp1.force = mp1.force + force;
			mp2.force = mp2.force - force;
		}
	}

	// calc external forces
	//  damping  F_damp(t)=-m_fDamping*v(t)
	for (int i = 0; i < m_iCountMassPoints; i++) {
		m_MassPoints[i].force += -m_springDamping * m_MassPoints[i].velocity;
	}

	/* calc acceleration at(t + h / 2) with forces at(t + h / 2) */
	for (int i = 0; i < m_iCountMassPoints; i++) {
		if (!m_MassPoints[i].isFixed) {
			m_MassPoints[i].acceleration = m_MassPoints[i].force;
			m_MassPoints[i].acceleration.safeDivide(m_MassPoints[i].mass);
			// add constant acceleration (gravity)
			//m_MassPoints[i].acceleration += m_constantAcceleration;
		}
	}

	/* update velocity with acceleration at(t + h / 2) */
	//  v(t+h) = v(0) + h * a(t+h/2)
	for (int i = 0; i < m_iCountMassPoints; i++) {
		if (!m_MassPoints[i].isFixed) {
			m_MassPoints[i].velocity += timeStep * m_MassPoints[i].acceleration;
		}
	}

	/* update all body sizes */
	for (int i = 0; i < m_iCountBodies; i++) {
		Body& b = m_Bodies[i];
		if (b.isFixed) { continue; }

		// maintain minimum size
		clampBodySize(b);
		updateBodyInertiaTensor(b); // TODO do not update iiit but update actuall iit directly
	}

}

void BouncyBoxSimulator::clampBodySize(Body &b) {
	b.MP_Size->position.x = std::max(b.MP_Size->position.x, BODY_MIN_SIZE);
	b.MP_Size->position.y = std::max(b.MP_Size->position.y, BODY_MIN_SIZE);
	b.MP_Size->position.z = std::max(b.MP_Size->position.z, BODY_MIN_SIZE);
	if (g_iTestCase == 0) {
		b.MP_Size->position.x = std::min(b.MP_Size->position.x, BODY_MAX_SIZE);
		b.MP_Size->position.y = std::min(b.MP_Size->position.y, BODY_MAX_SIZE);
		b.MP_Size->position.z = std::min(b.MP_Size->position.z, BODY_MAX_SIZE);
	}
}

Vec3 BouncyBoxSimulator::my_rotate(const Vec3 v, const Quat rot) {
	return rot.getRotMat()* v;

	Quat tmp = Quat(v.x, v.y, v.z, 0.0f);
	Quat rot_ = Quat(-rot.x, -rot.y, -rot.z, rot.w);
	Quat result = rot * tmp * rot_;
	return Vec3(result.x, result.y, result.z);
}

// MUST not check two fixed bodies!
bool BouncyBoxSimulator::collisionDetectionBroadPhaseSphereCheck(const Vec3 pos1, const Real radius1, const Vec3 pos2, const Real radius2) {
	Real squared_radius = radius1 + radius2;
	squared_radius = squared_radius * squared_radius;

	Vec3 relDist = pos1 - pos2;
	Real squared_dist = dot(relDist, relDist);

	return squared_dist <= squared_radius;
}

bool BouncyBoxSimulator::collisionDetectionBroadPhaseSphereCheck(const Body &b1, const Body &b2) {
	Vec3 b1s = b1.MP_Size->position;
	Vec3 b2s = b2.MP_Size->position;

	Vec3 relDist = b1.position - b2.position;
	Real sqd = relDist.x * relDist.x + relDist.y * relDist.y + relDist.z * relDist.z;

	Real r1_ = 0.25 * (b1s.x * b1s.x + b1s.y * b1s.y + b1s.z * b1s.z); // squared radius
	Real r2_ = 0.25 * (b2s.x * b2s.x + b2s.y * b2s.y + b2s.z * b2s.z);

	Real lhs = sqd - (r1_ + r2_);
	lhs = lhs * lhs;
	Real rhs = 4 * r1_ * r2_;

	return lhs <= rhs;
}

void BouncyBoxSimulator::addNarrowPhaseCheck(const int idxA, const int idxB) {
	m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = idxA;
	m_iCountNarrowPhaseBodies++;
	m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = idxB;
	m_iCountNarrowPhaseBodies++;
}

void BouncyBoxSimulator::collisionDetectionBroadPhase() {
	m_iCountNarrowPhaseBodies = 0; // Note: we save the two body indieces for the narrow test like |idxA|idxB|...
	
	/* FIXED-FREE CHECKS */
	if (g_iTestCase == 0) {
		// we know we have 6 fixed bodies (walls)
		// only limited checks are required!

		// furthest collision would be a block of maximum size rotated such that a corner collisdes with the wall
		static const Real maxRadius = 0.5 * my_norm(Vec3(BODY_MAX_SIZE));

		static const Real wall[6]{
			m_Bodies[0].position.y + 0.5 * m_Bodies[0].MP_Size->position.y + maxRadius, // wall y- (floor)
			m_Bodies[1].position.y - 0.5 * m_Bodies[1].MP_Size->position.y - maxRadius, // wall y+ (ceiling)
			m_Bodies[2].position.x + 0.5 * m_Bodies[2].MP_Size->position.x + maxRadius, // wall x-
			m_Bodies[3].position.x - 0.5 * m_Bodies[3].MP_Size->position.x - maxRadius, // wall x+
			m_Bodies[4].position.z + 0.5 * m_Bodies[4].MP_Size->position.z + maxRadius, // wall z-
			m_Bodies[5].position.z - 0.5 * m_Bodies[5].MP_Size->position.z - maxRadius  // wall z+
		};

		for (int i = m_iCountFixedBodies; i < m_iCountBodies; i++) {
			Body& b = m_Bodies[i];

			int res = (int)wall[0] > b.position.y; // either 0 or 1
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 0;
			m_iCountNarrowPhaseBodies += res;
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
			m_iCountNarrowPhaseBodies += res;

			res = (int)wall[1] < b.position.y; // either 0 or 1
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 1;
			m_iCountNarrowPhaseBodies += res;
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
			m_iCountNarrowPhaseBodies += res;


			res = (int)wall[2] > b.position.x; // either 0 or 1
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 2;
			m_iCountNarrowPhaseBodies += res;
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
			m_iCountNarrowPhaseBodies += res;

			res = (int)wall[3] < b.position.x; // either 0 or 1
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 3;
			m_iCountNarrowPhaseBodies += res;
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
			m_iCountNarrowPhaseBodies += res;


			res = (int)wall[4] > b.position.z; // either 0 or 1
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 4;
			m_iCountNarrowPhaseBodies += res;
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
			m_iCountNarrowPhaseBodies += res;

			res = (int)wall[5] < b.position.z; // either 0 or 1
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 5;
			m_iCountNarrowPhaseBodies += res;
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
			m_iCountNarrowPhaseBodies += res;

			/*
			if (wall[0] > b.position.y) {
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 0;
				m_iCountNarrowPhaseBodies++;
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
				m_iCountNarrowPhaseBodies++;
			} else if (wall[1] < b.position.y) {
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 1;
				m_iCountNarrowPhaseBodies++;
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
				m_iCountNarrowPhaseBodies++;
			}

			if (wall[2] > b.position.x) {
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 2;
				m_iCountNarrowPhaseBodies++;
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
				m_iCountNarrowPhaseBodies++;
			} else if (wall[3] < b.position.x) {
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 3;
				m_iCountNarrowPhaseBodies++;
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
				m_iCountNarrowPhaseBodies++;
			}

			if (wall[4] > b.position.z) {
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 4;
				m_iCountNarrowPhaseBodies++;
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
				m_iCountNarrowPhaseBodies++;
			}
			else if (wall[5] < b.position.z) {
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = 5;
				m_iCountNarrowPhaseBodies++;
				m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
				m_iCountNarrowPhaseBodies++;
			}
			*/
		}
	}
	else {
		// We have no special knowledge of the fixed bodies
		// => Just do normal sphere check
		for (int i = 0; i < m_iCountFixedBodies; i++) {
			Body& b1 = m_Bodies[i];
			Real radius1 = 0.5 * my_norm(b1.MP_Size->position);

			for (int o = m_iCountFixedBodies; o < m_iCountBodies; o++) {
				Body& b2 = m_Bodies[o];
				Real radius2 = 0.5 * my_norm(b2.MP_Size->position);

				if (collisionDetectionBroadPhaseSphereCheck(b1.position, radius1, b2.position, radius2)) {
					// collision possible => add to narrow phase
					m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
					m_iCountNarrowPhaseBodies++;
					m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = o;
					m_iCountNarrowPhaseBodies++;
				} // else collision NOT possible => ignore
			}
		}
	}

	/* FREE-FREE CHECKS */
	for (int i = m_iCountFixedBodies; i < m_iCountBodies; i++) {
		Body& b1 = m_Bodies[i];
		//Real radius1 = 0.5 * my_norm(m_MassPoints[b1.squishMP_Size].position);

		for (int o = i + 1; o < m_iCountBodies; o++) {
			Body& b2 = m_Bodies[o];
			//Real radius2 = 0.5 * my_norm(m_MassPoints[b2.squishMP_Size].position);

			//int res = (int)collisionDetectionBroadPhaseSphereCheck(b1.position, radius1, b2.position, radius2); // either 0 or 1
			int res = (int)collisionDetectionBroadPhaseSphereCheck(b1, b2); // either 0 or 1
			// if 1 => collision possible => add to narrow phase
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = i;
			m_iCountNarrowPhaseBodies += res;
			m_narrowPhaseBodies[m_iCountNarrowPhaseBodies] = o;
			m_iCountNarrowPhaseBodies += res;
			// else collision NOT possible => ignore
		}
	}
}

void BouncyBoxSimulator::simulateTimestep(float timeStep)
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

	if (m_iTestCase == 0) {
		m_iCountBodiesTarget = std::min(m_iCountBodiesTarget, m_iMaxCountBodies);
		if (m_iCountBodies < m_iCountBodiesTarget) {
			spawnBody(); // try to spawn a body (success only if there is space)
		}
	}

	/* RUN MASS SPRING SYSTEM */
	runMassSpringSystem(timeStep);



	/* RUN RIGID BODY SYSTEM */
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
		// Note: Using bool<->int magic here to avoid branching TODO
		if (!m_Bodies[i].isFixed) {
			m_Bodies[i].linear_velocity += timeStep * acceleration;
		}
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
		Quat angularQ = Quat(m_Bodies[i].angular_velocity.x, m_Bodies[i].angular_velocity.y, m_Bodies[i].angular_velocity.z, 0.0f);
		m_Bodies[i].orientation += (angularQ * m_Bodies[i].orientation) * (0.5 * timeStep);
		m_Bodies[i].orientation = m_Bodies[i].orientation.unit(); // renormalize

		/* update angular_velocity */
		//  using angular momentum & inertia tensor
		m_Bodies[i].angular_velocity = tensor * m_Bodies[i].angular_momentum;
	}


	/* CHECK COLLISIONS */
	collisionDetectionBroadPhase();

	// TODO handle walls differently in testcase 0
	//  narrow phase
	//   expensive box-on-box check (using SAT)
	for (int i = 0; i < m_iCountNarrowPhaseBodies; i += 2) {
		Body& b1 = m_Bodies[m_narrowPhaseBodies[i]];

		Mat4 b1_obj2world = constructObj2World(b1.position, b1.MP_Size->position, b1.orientation);
		Body& b2 = m_Bodies[m_narrowPhaseBodies[i+1]];
		Mat4 b2_obj2world = constructObj2World(b2.position, b2.MP_Size->position, b2.orientation);

		CollisionInfo col = checkCollisionSAT(b1_obj2world, b2_obj2world);

		if (col.isValid) {
			// calc velocities at collision point
			Vec3 b1_relcolpos = col.collisionPointWorld - b1.position;
			Vec3 b2_relcolpos = col.collisionPointWorld - b2.position;
			Vec3 b1_colvel = b1.linear_velocity + cross(b1_relcolpos, b1.angular_velocity);
			Vec3 b2_colvel = b2.linear_velocity + cross(b2_relcolpos, b2.angular_velocity);

			// calc relative velocity
			Vec3 relcolvel = b1_colvel - b2_colvel;

			float _dot = dot(col.normalWorld, relcolvel);
			if (_dot < 0.0f) { // actually colliding aka moving into each other => apply impuls
				/* calc impulse */
				// calc CURRENT inverse inertia tensor
				Mat4 b1_tensor = constructIIIT(b1.iiit, b1.orientation);
				Mat4 b2_tensor = constructIIIT(b2.iiit, b2.orientation);

				Vec3 b1_rot_change = cross(b1_tensor * (cross(b1_relcolpos, col.normalWorld)), b1_relcolpos);
				Vec3 b2_rot_change = cross(b2_tensor * (cross(b2_relcolpos, col.normalWorld)), b2_relcolpos);

				Real J = (-1.0 * (1.0 + (Real)m_fBounciness) * dot(relcolvel, col.normalWorld)) / ((Real)b1.invMass + (Real)b2.invMass + dot(b1_rot_change + b2_rot_change, col.normalWorld));
				
				/* apply impuls */
				b1.linear_velocity += (col.normalWorld * J) * (Real)b1.invMass;
				b2.linear_velocity -= (col.normalWorld * J) * (Real)b2.invMass;

				b1.angular_momentum += 0.15f * cross(b1_relcolpos, col.normalWorld * J);
				b2.angular_momentum -= 0.15f * cross(b2_relcolpos, col.normalWorld * J);


				Real scale = g_iTestCase == 0 ? 100 : 5;
				if (col.depth > 0.0f) {
					if (b2.isFixed) {
						Vec3 squish = b1_relcolpos;
						squish.x = std::abs(squish.x);
						squish.y = std::abs(squish.y);
						squish.z = std::abs(squish.z);
						squishBody(b1, -scale * squish * (Real)col.depth);
					}
					else if (b1.isFixed) {
						Vec3 squish = b2_relcolpos;
						squish.x = std::abs(squish.x);
						squish.y = std::abs(squish.y);
						squish.z = std::abs(squish.z);
						squishBody(b2, -scale * squish * (Real)col.depth);

					}
					else {
						Vec3 squish = b1_relcolpos;
						squish.x = std::abs(squish.x);
						squish.y = std::abs(squish.y);
						squish.z = std::abs(squish.z);
						squishBody(b1, 0.5 * -scale * squish * (Real)col.depth);
						squish = b2_relcolpos;
						squish.x = std::abs(squish.x);
						squish.y = std::abs(squish.y);
						squish.z = std::abs(squish.z);
						squishBody(b2, 0.5 * -scale * squish * (Real)col.depth);
					}
				}
			}
			// elseif >0 already separating
			// elseif ==0 sliding contact

			// Needs to be done regardles of separation, or future collisions are not detected correctly and ghosting occurs
			if (col.depth > 0.0f) {
				if (b2.isFixed) {
					b1.position += col.normalWorld * ((Real)col.depth);
				}
				else if (b1.isFixed) {
					b2.position += -col.normalWorld * ((Real)col.depth);
				}
				else {
					Real ratio = b2.mass / (b1.mass + b2.mass); // displacement is weighted by mass (lighter bodies get more displaced than heavier ones)
					b1.position += col.normalWorld * (ratio * ((Real)col.depth));
					b2.position += -col.normalWorld * ((1.0 - ratio) * ((Real)col.depth));
				}
			}

		}
	}

}

void BouncyBoxSimulator::squishBody(Body &b, const Vec3 delta) {
	b.MP_Size->position += delta;

	clampBodySize(b);
	updateBodyInertiaTensor(b);
}

void BouncyBoxSimulator::onClick(int x, int y)
{
	// Called WHILE mouse is clicked
	m_trackmouse = { x, y };
}

void BouncyBoxSimulator::onMouse(int x, int y)
{
	// Called WHILE mouse is NOT clicked
	m_oldtrackmouse = { x, y };
	m_trackmouse = { x, y };
}

void BouncyBoxSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	applyForceOnBody(m_Bodies[i], loc, force);
}

void BouncyBoxSimulator::applyForceOnBody(Body &b, Vec3 loc, Vec3 force) {
	// accumulate linear force
	b.force += force;

	// accumulate torque
	Vec3 rel_pos = loc - b.position;
	b.torque += cross(rel_pos, force);
}

// First all fixed bodies need to be added, then only free ones!
void BouncyBoxSimulator::addRigidBody(Vec3 position, Vec3 size, float mass, bool isFixed) {
	// a rigid body has to be in a valid state after being added
	
	// Sanity checks
	if (m_iCountBodies >= m_iMaxCountBodies) {
		std::cerr << "Too many bodies! (MAX = " << m_iMaxCountBodies << ")" << std::endl;
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
	if (isFixed && m_iCountBodies > 0 && !m_Bodies[m_iCountBodies-1].isFixed) {
		std::cerr << "Trying to add fixed body after free one!" << std::endl;
		exit(1);
	}

	m_Bodies[m_iCountBodies].position = position;
	m_Bodies[m_iCountBodies].linear_velocity = Vec3(0.0f);
	m_Bodies[m_iCountBodies].force = Vec3(0.0f);
	m_Bodies[m_iCountBodies].mass = mass;
	m_Bodies[m_iCountBodies].invMass = 1.0f / mass;

	m_Bodies[m_iCountBodies].orientation = Quat(Vec3(0.0f, 0.1f, 0.0f), 0.0f).unit();
	m_Bodies[m_iCountBodies].angular_velocity = Vec3(0.0f);
	m_Bodies[m_iCountBodies].angular_momentum = Vec3(0.0f);
	m_Bodies[m_iCountBodies].torque = Vec3(0.0f);

	// always select newest body
	m_iSelectedBody = m_iCountBodies;

	//m_Bodies[m_iCountBodies].size = size; => replaced by MPS
	// internal MPS for quishy-ness
	int MP_Size_idx = addMassPoint(1.0f, size, Vec3(0.0f), false);
	m_Bodies[m_iCountBodies].MP_Size = &m_MassPoints[MP_Size_idx];
	m_Bodies[m_iCountBodies].squishMP_FixX = addMassPoint(1.0f, Vec3(size.x, 0.0f, 0.0f), Vec3(0.0f), true); // fix point on x-axis
	m_Bodies[m_iCountBodies].squishMP_FixY = addMassPoint(1.0f, Vec3(0.0f, size.y, 0.0f), Vec3(0.0f), true); // fix point on y-axis
	m_Bodies[m_iCountBodies].squishMP_FixZ = addMassPoint(1.0f, Vec3(0.0f, 0.0f, size.z), Vec3(0.0f), true); // fix point on z-axis
	addSpring(100.0f, squishMP_FixO, MP_Size_idx, -1.0f);
	addSpring(5.0f, m_Bodies[m_iCountBodies].squishMP_FixX, MP_Size_idx, -1.0f);
	addSpring(5.0f, m_Bodies[m_iCountBodies].squishMP_FixY, MP_Size_idx, -1.0f);
	addSpring(5.0f, m_Bodies[m_iCountBodies].squishMP_FixZ, MP_Size_idx, -1.0f);

	m_Bodies[m_iCountBodies].isFixed = isFixed;
	if (isFixed) { // tricks for fixed bodies
		m_Bodies[m_iCountBodies].invMass = 0.0f;
		m_Bodies[m_iCountBodies].iiit = Vec3(0.0f);
		m_iCountFixedBodies++;
	}
	else {
		updateBodyInertiaTensor(m_Bodies[m_iCountBodies]);
	}

	m_iCountBodies++;
}

void BouncyBoxSimulator::setOrientationOf(int i, Quat orientation) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	m_Bodies[i].orientation = orientation.unit();
}

void BouncyBoxSimulator::setVelocityOf(int i, Vec3 velocity) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	m_Bodies[i].linear_velocity = velocity;
}

int BouncyBoxSimulator::getNumberOfRigidBodies() {
	return m_iCountBodies;
}

Vec3 BouncyBoxSimulator::getPositionOfRigidBody(int i) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	return m_Bodies[i].position;
}

Vec3 BouncyBoxSimulator::getLinearVelocityOfRigidBody(int i) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	return m_Bodies[i].linear_velocity;
}

Vec3 BouncyBoxSimulator::getAngularVelocityOfRigidBody(int i) {
	// Sanity checks
	if (i < 0 || i >= m_iCountBodies) {
		std::cerr << "Invalid index! There are only " << m_iCountBodies << " bodies." << std::endl;
		exit(1);
	}

	return m_Bodies[i].angular_velocity;
}


int BouncyBoxSimulator::addMassPoint(float mass, Vec3 position, Vec3 velocity, bool isFixed) {
	// Sanity checks
	if (m_iCountMassPoints >= m_iMaxCountMassPoints) {
		std::cerr << "Too many mass points! (MAX = " << m_iMaxCountMassPoints << ")" << std::endl;
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

	m_MassPoints[m_iCountMassPoints].mass = mass;
	m_MassPoints[m_iCountMassPoints].position = position;
	m_MassPoints[m_iCountMassPoints].tmp_position = position;
	m_MassPoints[m_iCountMassPoints].velocity = velocity;
	m_MassPoints[m_iCountMassPoints].isFixed = isFixed;

	return m_iCountMassPoints++;
}

void BouncyBoxSimulator::addSpring(float stiffness, int masspoint1, int masspoint2, float initialLength) {
	// Sanity checks
	if (m_iCountSprings >= m_iMaxCountSprings) {
		std::cerr << "Too many springs! (MAX = " << m_iMaxCountSprings << ")" << std::endl;
		exit(1);
	}
	if (stiffness == 0.0f) {
		std::cerr << "No stiffness set! stiffness == 0" << std::endl;
		exit(1);
	}
	if (stiffness <= 0.0f) {
		std::cerr << "Negative stiffness set!" << std::endl;
		exit(1);
	}
	if (masspoint1 < 0 || masspoint1 >= m_iMaxCountMassPoints) {
		std::cerr << "Invalid masspoint1! masspoint1 = " << masspoint1 << std::endl;
		exit(1);
	}
	if (masspoint2 < 0 || masspoint2 >= m_iMaxCountMassPoints) {
		std::cerr << "Invalid masspoint2! masspoint2 = " << masspoint2 << std::endl;
		exit(1);
	}
	if (masspoint1 == masspoint2) {
		std::cerr << "It is invalid to add spring to the same masspoint!" << std::endl;
		exit(1);
	}
	if (m_MassPoints[masspoint1].isFixed && m_MassPoints[masspoint2].isFixed) {
		std::cerr << "WARNING: You are trying to add a spring between two fix points! No spring added." << std::endl;
		return;
	}

	m_Springs[m_iCountSprings].stiffness = stiffness;
	m_Springs[m_iCountSprings].masspoint1 = masspoint1;
	m_Springs[m_iCountSprings].masspoint2 = masspoint2;
	// Note: if initialLength is negative, we use the current masspoint distance as spring length
	if (initialLength < 0.0f) {
		m_Springs[m_iCountSprings].length = my_norm(m_MassPoints[masspoint1].position - m_MassPoints[masspoint2].position);
	}
	else {
		m_Springs[m_iCountSprings].length = initialLength;
	}

	m_iCountSprings++;
}

int BouncyBoxSimulator::getNumberOfMassPoints()
{
	return m_iCountMassPoints;
}

int BouncyBoxSimulator::getNumberOfSprings()
{
	return m_iCountSprings;
}

Vec3 BouncyBoxSimulator::getPositionOfMassPoint(int index)
{
	if (index < 0 || index >= m_iCountMassPoints) {
		std::cerr << "Invalid index! (index = " << index << ")" << std::endl;
		return Vec3();
	}
	return m_MassPoints[index].position;
}

Vec3 BouncyBoxSimulator::getVelocityOfMassPoint(int index)
{
	if (index < 0 || index >= m_iCountMassPoints) {
		std::cerr << "Invalid index! (index = " << index << ")" << std::endl;
		return Vec3();
	}
	return m_MassPoints[index].velocity;
}
