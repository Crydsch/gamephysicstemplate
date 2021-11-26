#include "RigidBodySystemSimulator.h"

//RigidBodySystemSimulator::RigidBodySystemSimulator(int maxCountMassPoints, int maxCountSprings)
RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	//m_iMaxCountMassPoints = maxCountMassPoints;
	//m_iMaxCountSprings = maxCountSprings;
	//m_iCountMassPoints = 0;
	//m_iCountSprings = 0;
	//m_MassPoints = (MassPoint*)calloc(m_iMaxCountMassPoints, sizeof(MassPoint)); // default 10000*(4+3*4+3*4+1)=283KB
	//m_Springs = (Spring*)calloc(m_iMaxCountSprings, sizeof(Spring)); // default 10000*(4+4+4+4)=156KB

	m_iTestCase = 0;

	reset();
}

RigidBodySystemSimulator::~RigidBodySystemSimulator()
{
	/*free(m_MassPoints);
	free(m_Springs);*/
}

void RigidBodySystemSimulator::reset()
{
	//std::cerr << "reset()" << std::endl;

	m_externalForce = Vec3(0.0f);
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };

	// Note: No need to actually reset every mass point/spring
	//  Only the ones in [0,count[ are valid anyway
	/*m_iCountMassPoints = 0;
	m_iCountSprings = 0;*/
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
	switch (m_iTestCase)
	{
	case 0: /* Demo 1 */
		// ??? TODO
		// We construct our own Simulator similar to the Visual Studio Testcases

		rbss = new RigidBodySystemSimulator();
		// TODO
		/*std::cout << "\nSimulation State after one step (h=0.1f, EULER)\n" <<
			"  P[" << mp1 << "].position = " << msss->getPositionOfMassPoint(mp1).toString() << "\n" <<
			"  P[" << mp1 << "].velocity = " << msss->getVelocityOfMassPoint(mp1).toString() << "\n" <<
			"  P[" << mp2 << "].position = " << msss->getPositionOfMassPoint(mp2).toString() << "\n" <<
			"  P[" << mp2 << "].velocity = " << msss->getVelocityOfMassPoint(mp2).toString() << std::endl;*/
		delete rbss;

		// Demo 1 is a one shot => once done we switch to another testcase
		g_iTestCase = 1;
		notifyCaseChanged(g_iTestCase);
		break;

	case 1: /* Demo 2 */
		// ??? TODO
		// Run and render the simulation

		break;

	case 2: /* Demo 3 */
		// ??? TODO
		// Run and render the simulation

		break;

	case 3: /* Demo 4*/
		// ??? TODO
		// Run and render the simulation

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
	// integrator
	//TwAddButton(DUC->g_pTweakBar, "Use EULER", [](void* s) { *((int*)s) = EULER; }, &m_iIntegrator, "");
	//TwAddButton(DUC->g_pTweakBar, "Use MIDPOINT", [](void* s) { *((int*)s) = MIDPOINT; }, &m_iIntegrator, "");
	// damping
	//TwAddVarRW(DUC->g_pTweakBar, "Damping Factor", TW_TYPE_FLOAT, &m_fDamping, "step=0.05 min=0.0");

	switch (m_iTestCase)
	{
	case 0: // Demo 1: Nothing rendered
		break;
	case 1: // Demo 2
		break;
	case 2: // Demo 3
		break;
	case 3: // Demo 4
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

	//const Vec3 lilac = Vec3(0x7D, 0x4F, 0xB0) / 255.0f; // #7D4FB0
	//const Vec3 lightlilac = Vec3(0x90, 0x68, 0xBB) / 255.0f; // #9068BB
	//const Vec3 green = Vec3(0x7B, 0xBB, 0x44) / 255.0f; // #7BBB44
	//const Vec3 red = Vec3(0xBB, 0x48, 0x44) / 255.0f; // ##BB4844
	//const Vec3 blue = Vec3(0x44, 0xB7, 0xBB) / 255.0f; // ##44B7BB

	DUC->setUpLighting(Vec3(0.0f), 0.4 * turquoise, 100, 0.6 * darkturquoise);
	//BodyA.Obj2WorldMatrix = BodyA.scaleMat * BodyA.rotMat * BodyA.translatMat;
	//DUC->drawRigidBody(BodyA.Obj2WorldMatrix);

}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Attention: Called before simulateTimestep when running through simulator
	//            NOT called when running through tests (but tests do not use external forces ...)

	// => Best not to rely on this function being called or not and just do our own internal calculations for external forces!
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	Point2D mouseDiff;
	Vec3 user_force;
	// find a proper scale!
	float inputScale = 0.5f;

	// Note: We only use explicit EULER for the rigid body simulation
	
		///* calc forces with OLD position */
		//// reset forces
		//for (int i = 0; i < m_iCountMassPoints; i++) {
		//	m_MassPoints[i].force = Vec3(0, 0, 0);
		//}

		//// calc internal forces
		////  for each spring
		//for (int i = 0; i < m_iCountSprings; i++) {
		//	Spring& s = m_Springs[i];
		//	MassPoint& mp1 = m_MassPoints[s.masspoint1];
		//	MassPoint& mp2 = m_MassPoints[s.masspoint2];

		//	if (mp1.isFixed && mp2.isFixed) {
		//		// do not apply any force on fixed points!
		//		continue;
		//	}

		//	// calc force aka hookes law (on mp1)
		//	float distance = norm(mp1.position - mp2.position);
		//	Vec3 direction = getNormalized(mp1.position - mp2.position);
		//	Vec3 force = -s.stiffness * (distance - s.length) * direction;

		//	// add forces to mass points current forces
		//	if (mp1.isFixed && !mp2.isFixed) {
		//		mp2.force += -2 * force;
		//	}
		//	else if (!mp1.isFixed && mp2.isFixed) {
		//		mp2.force += 2 * force;
		//	}
		//	else { // both are not fixed
		//		mp1.force = mp1.force + force;
		//		mp2.force = mp2.force - force;
		//	}
		//}

		//// calc external forces
		////  damping  F_damp(t)=-m_fDamping*v(t)
		//for (int i = 0; i < m_iCountMassPoints; i++) {
		//	m_MassPoints[i].force += -m_fDamping * m_MassPoints[i].velocity;
		//}
		////  user interaction
		//mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
		//mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
		//if (mouseDiff.x != 0 || mouseDiff.y != 0)
		//{
		//	Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		//	worldViewInv = worldViewInv.inverse();
		//	Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		//	user_force = worldViewInv.transformVectorNormal(inputView);
		//	user_force = user_force * inputScale;
		//}
		//if (m_mainPoint != -1) {
		//	m_MassPoints[m_mainPoint].force += user_force;
		//}

		///* calc acceleration with NEW forces */
		//for (int i = 0; i < m_iCountMassPoints; i++) {
		//	if (!m_MassPoints[i].isFixed) {
		//		m_MassPoints[i].acceleration = m_MassPoints[i].force;
		//		m_MassPoints[i].acceleration.safeDivide(m_MassPoints[i].mass);
		//		// add constant acceleration (gravity)
		//		m_MassPoints[i].acceleration += m_constantAcceleration;
		//	}
		//}

		///* update position with OLD velocity */
		//for (int i = 0; i < m_iCountMassPoints; i++) {
		//	if (!m_MassPoints[i].isFixed) {
		//		m_MassPoints[i].position += timeStep * m_MassPoints[i].velocity;
		//	}
		//}
		//// handle collisions
		//clampMassPointsToBox(Vec3(-2, -0.95, -2), Vec3(2, 4, 2));

		///* update velocity with NEW acceleration */
		//for (int i = 0; i < m_iCountMassPoints; i++) {
		//	if (!m_MassPoints[i].isFixed) {
		//		m_MassPoints[i].velocity += timeStep * m_MassPoints[i].acceleration;
		//	}
		//}

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

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	// TODO
	return 0;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	// TODO
	return Vec3(0.0f);
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	// TODO
	return Vec3(0.0f);
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	// TODO
	return Vec3(0.0f);
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	// TODO

}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	// TODO

}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	// TODO

}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	// TODO

}
