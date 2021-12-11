#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();

	m_fDiffusionCoefficient = 0.01;
	m_iGridWidth = 16;
	m_iGridHeight = 16;
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;


		T = new Grid(m_iGridWidth + 2, m_iGridHeight + 2);
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	TwAddSeparator(DUC->g_pTweakBar, "", "");
	//TwAddVarRW(DUC->g_pTweakBar, "Maximum Heat", TW_TYPE_DOUBLE, &m_fMaxHeat, "step=10 min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Diffusion Coefficient", TW_TYPE_DOUBLE, &m_fDiffusionCoefficient, "step=0.01 min=0");
	TwAddVarRW(DUC->g_pTweakBar, "Resolution Width", TW_TYPE_INT32, &m_iGridWidth, "step=1 min=0");
	TwAddVarRW(DUC->g_pTweakBar, "Resolution Height", TW_TYPE_INT32, &m_iGridHeight, "step=1 min=0");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	
	// setup something to diffuse
	for (int x = 1; x < m_iGridWidth / 2; x++) {
		for (int y = 1; y < m_iGridHeight / 2; y++) {
			T->set(x, y, 1.0);
		}
	}

	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	Grid* newT = new Grid(T->Width(), T->Height());
	// Note: We only read from the boundary, we do not calc or set anything there

	const Real dx = 1.0 / m_iGridWidth;
	const Real dy = 1.0 / m_iGridHeight;
	const Real dt = timeStep;

	for (int x = 1; x <= m_iGridWidth; x++) {
		for (int y = 1; y <= m_iGridHeight; y++) {

			Real tmpX = T->get(x + 1, y) - 2 * T->get(x, y) + T->get(x - 1, y);
			tmpX /= dx * dx;
			Real tmpY = T->get(x, y + 1) - 2 * T->get(x, y) + T->get(x, y - 1);
			tmpY /= dy * dy;
			
			Real tmp = (tmpX + tmpY) * m_fDiffusionCoefficient * dt;
			tmp += T->get(x, y);

			newT->set(x, y, tmp);
		}
	}

	return newT;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT();//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	Grid* tmp = NULL;
	switch (m_iTestCase)
	{
	case 0:
		tmp = T;
		T = diffuseTemperatureExplicit(timeStep);
		delete(tmp); // don't leak memory!
		tmp = NULL;
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

Vec3 lerp(Vec3 a, Vec3 b, Real alpha) {
	return a * (1.0 - alpha) + b * alpha;
}

void DiffusionSimulator::drawObjects()
{
	// Note: We render a sphere for each grid point with a color from red==hot to white==cold
	// We render the grid in the middle of the unit cube

	// TODO gray?
	const Vec3 white = Vec3(0xFF, 0xFF, 0xFF) / 255.0f;
	//const Vec3 red = Vec3(0xBB, 0x48, 0x44) / 255.0f; // #BB4844
	const Vec3 red = Vec3(0xFF, 0x0, 0x0) / 255.0f; // #BB4844

	/*DUC->setUpLighting(Vec3(), 0.4 * white, 100, 0.6 * white);
	DUC->drawSphere(Vec3(-0.5, 0.5, 0.0), Vec3(0.05f));

	DUC->setUpLighting(Vec3(), 0.4 * red, 100, 0.6 * red);
	DUC->drawSphere(Vec3(0.5, -0.5, 0.0), Vec3(0.05f));*/

	// render grid points
	const Real offsetX = -0.5;
	const Real offsetY = 0.5;
	const Real dx = 1.0 / m_iGridWidth;
	const Real dy = 1.0 / m_iGridHeight;

	for (int x = 1; x <= m_iGridWidth; x++) {
		for (int y = 1; y <= m_iGridHeight; y++) {
			Vec3 pos = Vec3(offsetX + (x - 1) * dx, offsetY - (y - 1) * dx, 0.0);
			Vec3 col = lerp(white, red, T->get(x, y));

			DUC->setUpLighting(Vec3(), 0.4 * col, 100, 0.6 * col);
			DUC->drawSphere(pos, Vec3(min(dx, dy)));
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
