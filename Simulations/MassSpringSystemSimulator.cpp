#include "MassSpringSystemSimulator.h"
#include <set>

MassSpringSystemSimulator::MassSpringSystemSimulator(int maxCountMassPoints, int maxCountSprings)
{
	m_iMaxCountMassPoints = maxCountMassPoints;
	m_iMaxCountSprings = maxCountSprings;
	m_iCountMassPoints = 0;
	m_iCountSprings = 0;
	m_MassPoints = (MassPoint*)calloc(m_iMaxCountMassPoints, sizeof(MassPoint)); // default 10000*(4+3*4+3*4+1)=283KB
	m_Springs = (Spring*)calloc(m_iMaxCountSprings, sizeof(Spring)); // default 10000*(4+4+4+4)=156KB

	m_iTestCase = 0;
	m_iIntegrator = EULER;

	m_fCubeSize = 1.0f;
	m_iCubeResolution = 1;
	m_constantAcceleration = Vec3(0.0f);

	reset();
}

MassSpringSystemSimulator::~MassSpringSystemSimulator() {
	free(m_MassPoints);
	free(m_Springs);
}

void MassSpringSystemSimulator::reset()
{
	//std::cerr << "reset()" << std::endl;

	m_fMass = 0.0f;
	m_fStiffness = 0.0f;
	m_fDamping = 0.0f;

	m_externalForce = Vec3();
	m_mouse = { 0, 0 };
	m_trackmouse = { 0, 0 };
	m_oldtrackmouse = { 0, 0 };

	// Note: No need to actually reset every mass point/spring
	//  Only the ones in [0,count[ are valid anyway
	m_iCountMassPoints = 0;
	m_iCountSprings = 0;

	m_mainPoint = -1;
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	// List testcases as strings
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

// Note: This is defined in main.cpp (as well as the vs-testcases)
//       We use this to programatically change the active testcase.
extern int g_iTestCase;

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	//std::cerr << "notifyCaseChanged(" << testCase << ")" << std::endl;
	// Called to initialize new TestCase
	
	reset();

	m_iTestCase = testCase;
	int mp1, mp2;
	MassSpringSystemSimulator* msss;
	switch (m_iTestCase)
	{
	case 0: /* Demo 1 */
		// 2 mass points w/ 1 spring
		// We construct our own Simulator similar to the Visual Studio Testcases
		
		// EULER:
		msss = new MassSpringSystemSimulator();
		mp1 = msss->addMassPoint(10, Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		mp2 = msss->addMassPoint(10, Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		msss->addSpring(40, mp1, mp2, 1);
		msss->setIntegrator(EULER);
		msss->simulateTimestep(0.1f);
		std::cout << "\nSimulation State after one step (h=0.1f, EULER)\n" <<
			"  P[" << mp1 << "].position = " << msss->getPositionOfMassPoint(mp1).toString() << "\n" << 
			"  P[" << mp1 << "].velocity = " << msss->getVelocityOfMassPoint(mp1).toString() << "\n" <<
			"  P[" << mp2 << "].position = " << msss->getPositionOfMassPoint(mp2).toString() << "\n" <<
			"  P[" << mp2 << "].velocity = " << msss->getVelocityOfMassPoint(mp2).toString() << std::endl;
		delete msss;

		// MIDPOINT:
		msss = new MassSpringSystemSimulator();
		mp1 = msss->addMassPoint(10, Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		mp2 = msss->addMassPoint(10, Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		msss->addSpring(40, mp1, mp2, 1);
		msss->setIntegrator(MIDPOINT);
		msss->simulateTimestep(0.1f);
		std::cout << "\nSimulation State after one step (h=0.1f, MIDPOINT)\n" <<
			"  P[" << mp1 << "].position = " << msss->getPositionOfMassPoint(mp1).toString() << "\n" <<
			"  P[" << mp1 << "].velocity = " << msss->getVelocityOfMassPoint(mp1).toString() << "\n" <<
			"  P[" << mp2 << "].position = " << msss->getPositionOfMassPoint(mp2).toString() << "\n" <<
			"  P[" << mp2 << "].velocity = " << msss->getVelocityOfMassPoint(mp2).toString() << "\n" << std::endl;
		delete msss;

		// Demo 1 is a one shot => once done we switch to another testcase
		g_iTestCase = 1;
		notifyCaseChanged(g_iTestCase);
		break;

	case 1: /* Demo 2 */
		// 2 mass points w/ 1 spring
		// Run and render the simulation

		mp1 = addMassPoint(10, Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		mp2 = addMassPoint(10, Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(40, mp1, mp2, 1);
		setIntegrator(EULER);
		break;

	case 2: /* Demo 3 */
		// 2 mass points w/ 1 spring
		// Run and render the simulation

		mp1 = addMassPoint(10, Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		mp2 = addMassPoint(10, Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(40, mp1, mp2, 1);
		setIntegrator(MIDPOINT);
		break;
	
	case 3: /* Demo 4*/
		// elastic cube
		// center point at (0,0,0)
		addCube();
		setDampingFactor(1.2f);
		setConstantAcceleration(Vec3(0.0f, -0.5f, 0.0f));
		setIntegrator(MIDPOINT);

		break;

	default:
		std::cerr << "Unknown TestCase selected: " << testCase << std::endl;
		break;
	}
}

UINT64 springID(int mp1, int mp2) {
	// Note: mass points ids are positive integer
	return (((UINT64)mp1) << 32) | ((UINT64)mp2);
}

void MassSpringSystemSimulator::addCube() {
	// generate cube points
	int res = (m_iCubeResolution * 2 + 1);
	float pointDistance = m_fCubeSize / (res - 1);
	Vec3 minPos = Vec3(-pointDistance * m_iCubeResolution);

	int* points = (int*)calloc(res * res * res, sizeof(int));
	if (points == nullptr) { std::cerr << "calloc failed!" << std::endl; exit(1); }

	for (int x = 0; x < res; x++) {
		for (int y = 0; y < res; y++) {
			for (int z = 0; z < res; z++) {
				Vec3 pos = Vec3(minPos.x + pointDistance * x, minPos.y + pointDistance * y, minPos.z + pointDistance * z);
				points[x + res * y + res * res * z] = addMassPoint(1, pos, Vec3(0.0f), false);
			}
		}
	}

	m_mainPoint = points[m_iCubeResolution + res * m_iCubeResolution + res * res * m_iCubeResolution];

	set<UINT64, greater<UINT64>> springs;
	// generate springs
	// we add a spring to all neighbours (direct and diagonal)

	// for all mass points
	for (int x = 0; x < res; x++) {
		for (int y = 0; y < res; y++) {
			for (int z = 0; z < res; z++) {
				int mp1 = points[x + res * y + res * res * z];
				//std::cerr << "# " << mp1 << ":" << std::endl;

				// for all neighbours
				for (int dx = -1; dx <= 1; dx++) {
					for (int dy = -1; dy <= 1; dy++) {
						for (int dz = -1; dz <= 1; dz++) {
							
							// if valid index
							int nx = x + dx;
							int ny = y + dy;
							int nz = z + dz;
							if (nx >= 0 && nx < res &&
								ny >= 0 && ny < res &&
								nz >= 0 && nz < res &&
								!(nx == x && ny == y && nz == z)) {
								int mp2 = points[nx + res * ny + res * res * nz];
								//std::cerr << dx << "\t" << dy << "\t" << dz << std::endl;

								// if no spring yet, add one
								if (springs.find(springID(mp1, mp2)) == springs.end() &&
									springs.find(springID(mp2, mp1)) == springs.end()) {
									springs.emplace(springID(mp1, mp2));
									addSpring(200.0f, mp1, mp2, -1.0f);
								}

							}

						}
					}
				}

			}
		}
	}

	free(points);
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddSeparator(DUC->g_pTweakBar, "", "");
	// integrator
	TwAddButton(DUC->g_pTweakBar, "Use EULER", [](void* s) { *((int*)s) = EULER; }, &m_iIntegrator, "");
	TwAddButton(DUC->g_pTweakBar, "Use MIDPOINT", [](void* s) { *((int*)s) = MIDPOINT; }, &m_iIntegrator, "");
	// damping
	TwAddVarRW(DUC->g_pTweakBar, "Damping Factor", TW_TYPE_FLOAT, &m_fDamping, "step=0.05 min=0.0");

	switch (m_iTestCase)
	{
	case 0: // Demo 1: Nothing rendered
		break;
	case 1: // Demo 2: Just render simple EULER simulation
	case 2: // Demo 3: Just render simple MIDPOINT simulation
		// Render options: TODO
		//  render masspoint mass depending on radius
		//   float radius_factor, radius_mass_factor
		//  render spring thickness depending on stiffness
		//  render spring color depending on generated force

		break;
	case 3:
		// prob, have to be callbacks
		//TwAddVarRW(DUC->g_pTweakBar, "Cube Size", TW_TYPE_FLOAT, &m_iCubeSize, "min=0.5 step=0.1");
		//TwAddVarRW(DUC->g_pTweakBar, "Cube Resolution", TW_TYPE_INT32, &m_iCubeResolution, "min=2");
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	// We always render all MassPoints as spheres (BONUS: radius == mass)
	//  and all Springs als lines between the spheres (BONUS: thickness == stiffness, color == force generated)

	//const Vec3 turquoise = Vec3(64.0f / 255.0f, 224.0f / 255.0f, 208.0f / 255.0f);
	//const Vec3 darkturquoise = Vec3(0.0f, 206.0f / 255.0f, 209.0f / 255.0f);

	const Vec3 lilac = Vec3(0x7D, 0x4F, 0xB0) / 255.0f; // #7D4FB0
	const Vec3 lightlilac = Vec3(0x90, 0x68, 0xBB) / 255.0f; // #9068BB
	const Vec3 green = Vec3(0x7B, 0xBB, 0x44) / 255.0f; // #7BBB44
	const Vec3 red = Vec3(0xBB, 0x48, 0x44) / 255.0f; // ##BB4844
	const Vec3 blue = Vec3(0x44, 0xB7, 0xBB) / 255.0f; // ##44B7BB

	// render mass points
	for (int i = 0; i < m_iCountMassPoints; i++) {
		DUC->setUpLighting(Vec3(), 0.4 * lightlilac, 100, 0.6 * lilac);
		DUC->drawSphere(m_MassPoints[i].position, Vec3(0.05f));
	}

	// render springs
	DUC->beginLine();
	for (int i = 0; i < m_iCountSprings; i++) {
		MassPoint& mp1 = m_MassPoints[m_Springs[i].masspoint1];
		MassPoint& mp2 = m_MassPoints[m_Springs[i].masspoint2];
		DUC->drawLine(mp1.position, green, mp2.position, green);
	}
	DUC->endLine();
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Attention: Called before simulateTimestep when running through simulator
	//            NOT called when running through tests (but tests do not use external forces ...)

	// => Best not to rely on this function being called or not and just do our own internal calculations for external forces!
}

// clamps all masspoints coordinates to a box around (0,0,0)
// This effectively acts as a collision with the ground (and the other sides)
void MassSpringSystemSimulator::clampMassPointsToBox(Vec3 pMin, Vec3 pMax) {
	for (int i = 0; i < m_iCountMassPoints; i++) {
		// Note: maximize/minimize are bugged
		//m_MassPoints[i].position.maximize(pMin);
		//m_MassPoints[i].position.minimize(pMax);
		m_MassPoints[i].position.x = max(m_MassPoints[i].position.x, pMin.x);
		m_MassPoints[i].position.y = max(m_MassPoints[i].position.y, pMin.y);
		m_MassPoints[i].position.z = max(m_MassPoints[i].position.z, pMin.z);
		m_MassPoints[i].position.x = min(m_MassPoints[i].position.x, pMax.x);
		m_MassPoints[i].position.y = min(m_MassPoints[i].position.y, pMax.y);
		m_MassPoints[i].position.z = min(m_MassPoints[i].position.z, pMax.z);
	}
}

void MassSpringSystemSimulator::clampTmpMassPointsToBox(Vec3 pMin, Vec3 pMax) {
	// Does the same as clampMassPointsToBox jsut for midpoint tmp positions
	for (int i = 0; i < m_iCountMassPoints; i++) {
		// Note: maximize/minimize are bugged
		//m_MassPoints[i].position.maximize(pMin);
		//m_MassPoints[i].position.minimize(pMax);
		m_MassPoints[i].position.x = max(m_MassPoints[i].position.x, pMin.x);
		m_MassPoints[i].position.y = max(m_MassPoints[i].position.y, pMin.y);
		m_MassPoints[i].position.z = max(m_MassPoints[i].position.z, pMin.z);
		m_MassPoints[i].position.x = min(m_MassPoints[i].position.x, pMax.x);
		m_MassPoints[i].position.y = min(m_MassPoints[i].position.y, pMax.y);
		m_MassPoints[i].position.z = min(m_MassPoints[i].position.z, pMax.z);
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	Point2D mouseDiff;
	Vec3 user_force;
	int res = (m_iCubeResolution * 2 + 1);
	// find a proper scale!
	float inputScale = 0.5f;

	switch (m_iIntegrator)
	{
	case EULER:
		/* Possible optimization: 
			- Combine masspoint.force and .accel since accel fully depends on force (maybe use union)
			- Combine for loops where applicable
			- Maybe split fixed points from normal points and get rid of branch prediction
			   (also saves space since fixpoints dont need force/accel/vel)
			- cache optimize/pack masspoints and spring arrays
		*/
		/* TODO:
		    - Change safe devide to normal one, since mass > 0
			- Add collision detection
			- Add external forces (gravity as direct accel)
			- Add damping
		*/

		// DO EULER STEP
		
		/* calc forces with OLD position */
		// reset forces
		for (int i = 0; i < m_iCountMassPoints; i++) {
			m_MassPoints[i].force = Vec3(0, 0, 0);
		}

		// calc internal forces
		//  for each spring
		for (int i = 0; i < m_iCountSprings; i++) {
			Spring &s = m_Springs[i];
			MassPoint &mp1 = m_MassPoints[s.masspoint1];
			MassPoint &mp2 = m_MassPoints[s.masspoint2];

			if (mp1.isFixed && mp2.isFixed) {
				// do not apply any force on fixed points!
				continue;
			}

			// calc force aka hookes law (on mp1)
			float distance = norm(mp1.position - mp2.position);
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
			m_MassPoints[i].force += -m_fDamping * m_MassPoints[i].velocity;
		}
		//  user interaction
		mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
		mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
		if (mouseDiff.x != 0 || mouseDiff.y != 0)
		{
			Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
			worldViewInv = worldViewInv.inverse();
			Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
			user_force = worldViewInv.transformVectorNormal(inputView);
			user_force = user_force * inputScale;
		}
		if (m_mainPoint != -1) {
			m_MassPoints[m_mainPoint].force += user_force;
		}

		/* calc acceleration with NEW forces */
		for (int i = 0; i < m_iCountMassPoints; i++) {
			if (!m_MassPoints[i].isFixed) {
				m_MassPoints[i].acceleration = m_MassPoints[i].force;
				m_MassPoints[i].acceleration.safeDivide(m_MassPoints[i].mass);
				// add constant acceleration (gravity)
				m_MassPoints[i].acceleration += m_constantAcceleration;
			}
		}

		/* update position with OLD velocity */
		for (int i = 0; i < m_iCountMassPoints; i++) {
			if (!m_MassPoints[i].isFixed) {
				m_MassPoints[i].position += timeStep * m_MassPoints[i].velocity;
			}
		}
		// handle collisions
		clampMassPointsToBox(Vec3(-2, -0.95, -2), Vec3(2, 4, 2));

		/* update velocity with NEW acceleration */
		for (int i = 0; i < m_iCountMassPoints; i++) {
			if (!m_MassPoints[i].isFixed) {
				m_MassPoints[i].velocity += timeStep * m_MassPoints[i].acceleration;
			}
		}

		break;

	case MIDPOINT:

		// DO MIDPOINT INTEGRATION

		/* calc tmp_position at(t + h / 2) with OLD velocity */
		//  x(t+h/2) = x(t) + h/2 * v(t)
		for (int i = 0; i < m_iCountMassPoints; i++) {
			if (!m_MassPoints[i].isFixed) {
				m_MassPoints[i].tmp_position = m_MassPoints[i].position + (timeStep/2.0f) * m_MassPoints[i].velocity;
			}
		}
		// handle collisions
		clampTmpMassPointsToBox(Vec3(-2, -0.95, -2), Vec3(2, 3, 2));

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

			if (mp1.isFixed && mp2.isFixed) {
				// do not apply any force on fixed points!
				continue;
			}

			// calc force aka hookes law (on mp1)
			float distance = norm(mp1.position - mp2.position);
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
			m_MassPoints[i].force += -m_fDamping * m_MassPoints[i].velocity;
		}
		//  user interaction
		mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
		mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
		if (mouseDiff.x != 0 || mouseDiff.y != 0)
		{
			Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
			worldViewInv = worldViewInv.inverse();
			Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
			user_force = worldViewInv.transformVectorNormal(inputView);
			user_force = user_force * inputScale;
		}
		if (m_mainPoint != -1) {
			m_MassPoints[m_mainPoint].force += user_force;
		}
		
		/* calc acceleration at(t + h) with forces at(t + h) */
		for (int i = 0; i < m_iCountMassPoints; i++) {
			if (!m_MassPoints[i].isFixed) {
				m_MassPoints[i].acceleration = m_MassPoints[i].force;
				m_MassPoints[i].acceleration.safeDivide(m_MassPoints[i].mass);
				// add constant acceleration (gravity)
				m_MassPoints[i].acceleration += m_constantAcceleration;
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
		// handle collisions
		clampMassPointsToBox(Vec3(-2, -0.95, -2), Vec3(2, 3, 2));

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

			if (mp1.isFixed && mp2.isFixed) {
				// do not apply any force on fixed points!
				continue;
			}

			// calc force aka hookes law (on mp1)
			float distance = norm(mp1.tmp_position - mp2.tmp_position);
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
			m_MassPoints[i].force += -m_fDamping * m_MassPoints[i].velocity;
		}
		//  user interaction
		if (m_mainPoint != -1) {
			m_MassPoints[m_mainPoint].force += user_force;
		}
		
		/* calc acceleration at(t + h / 2) with forces at(t + h / 2) */
		for (int i = 0; i < m_iCountMassPoints; i++) {
			if (!m_MassPoints[i].isFixed) {
				m_MassPoints[i].acceleration = m_MassPoints[i].force;
				m_MassPoints[i].acceleration.safeDivide(m_MassPoints[i].mass);
				// add constant acceleration (gravity)
				m_MassPoints[i].acceleration += m_constantAcceleration;
			}
		}


		/* update velocity with acceleration at(t + h / 2) */
		//  v(t+h) = v(0) + h * a(t+h/2)
		for (int i = 0; i < m_iCountMassPoints; i++) {
			if (!m_MassPoints[i].isFixed) {
				m_MassPoints[i].velocity += timeStep * m_MassPoints[i].acceleration;
			}
		}

		break;
	case LEAPFROG:
		// TODO
		std::cerr << "LEAPFROG integrator not implemented" << std::endl;
		m_iIntegrator = NO_INTEGRATOR;
		break;
	case NO_INTEGRATOR:
		// Just do nothing
		break;
	default:
		std::cerr << "Unknown integrator: " << m_iIntegrator << std::endl;
		m_iIntegrator = NO_INTEGRATOR;
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	//std::cerr << "onClick(" << x << "," << y << ")" << std::endl;
	// Called WHILE mouse is clicked

	m_trackmouse = { x, y };
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	//std::cerr << "onMouse(" << x << "," << y << ")" << std::endl;
	// Called WHILE mouse is NOT clicked

	m_oldtrackmouse = { x, y };
	m_trackmouse = { x, y };
}

void MassSpringSystemSimulator::setMass(float mass)
{
	// Sets global mass to be used for new MassPoints
	// Note: We associate every point with a mass (to allow differing masses)
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	// Sets global stiffness to be used for new Springs
	// Note: We associate every spring with a stiffness (to allow differing stiffnesses)
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	// Note: Damping is a force applied based on the velocity
	//  => add F_damp(t)=-m_fDamping*v(t) when calculating internal forces to each mass point
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	return addMassPoint(m_fMass, position, velocity, isFixed);
}

int MassSpringSystemSimulator::addMassPoint(float mass, Vec3 position, Vec3 velocity, bool isFixed) {
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
	m_MassPoints[m_iCountMassPoints].velocity = velocity;
	m_MassPoints[m_iCountMassPoints].isFixed = isFixed;

	return m_iCountMassPoints++;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	addSpring(m_fStiffness, masspoint1, masspoint2, initialLength);
}

void MassSpringSystemSimulator::addSpring(float stiffness, int masspoint1, int masspoint2, float initialLength) {
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
		m_Springs[m_iCountSprings].length = norm(m_MassPoints[masspoint1].position - m_MassPoints[masspoint2].position);
	}
	else {
		m_Springs[m_iCountSprings].length = initialLength;
	}

	m_iCountSprings++;
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_iCountMassPoints;
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_iCountSprings;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	if (index < 0|| index >= m_iCountMassPoints) {
		std::cerr << "Invalid index! (index = " << index << ")" << std::endl;
		return Vec3();
	}
	return m_MassPoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	if (index < 0 || index >= m_iCountMassPoints) {
		std::cerr << "Invalid index! (index = " << index << ")" << std::endl;
		return Vec3();
	}
	return m_MassPoints[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	// Sets static global external force (aka gravity)
	// will be applied to every mass point, every time step
	// Note: A real force would be mass dependent,
	// => You should not use this function. 
	m_externalForce = force;
}

void MassSpringSystemSimulator::setConstantAcceleration(Vec3 acceleration) {
	m_constantAcceleration = acceleration;
}