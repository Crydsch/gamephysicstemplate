#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();

	m_fDiffusionCoefficient = 0.1;
	m_iGridWidth = 32;
	m_iGridHeight = 32;
	T[0] = NULL;
	T[1] = NULL;
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	if (T[0] != NULL) { delete(T[0]); }
	T[0] = new Grid(m_iGridWidth + 2, m_iGridHeight + 2);
	if (T[1] != NULL) { delete(T[1]); }
	T[1] = new Grid(m_iGridWidth + 2, m_iGridHeight + 2);
	m_iCurrGrid = 0;
}

void cbSetResWidth(const void* value, void* clientData) {
	DiffusionSimulator* df = (DiffusionSimulator*)clientData;
	df->m_iGridWidth = *((int*)value);
	df->notifyCaseChanged(df->m_iTestCase);
}

void cbGetResWidth(void* value, void* clientData) {
	DiffusionSimulator* df = (DiffusionSimulator*)clientData;
	*((int*)value) = df->m_iGridWidth;
}

void cbSetResHeight(const void* value, void* clientData) {
	DiffusionSimulator* df = (DiffusionSimulator*)clientData;
	df->m_iGridHeight = *((int*)value);
	df->notifyCaseChanged(df->m_iTestCase);
}

void cbGetResHeight(void* value, void* clientData) {
	DiffusionSimulator* df = (DiffusionSimulator*)clientData;
	*((int*)value) = df->m_iGridHeight;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	TwAddSeparator(DUC->g_pTweakBar, "", "");
	TwAddVarRW(DUC->g_pTweakBar, "Diffusion Coefficient", TW_TYPE_DOUBLE, &m_fDiffusionCoefficient, "step=0.01 min=0");
	//TwAddVarRW(DUC->g_pTweakBar, "Resolution Width", TW_TYPE_INT32, &m_iGridWidth, "step=1 min=4");
	//TwAddVarRW(DUC->g_pTweakBar, "Resolution Height", TW_TYPE_INT32, &m_iGridHeight, "step=1 min=4");
	TwAddVarCB(DUC->g_pTweakBar, "Resolution Width", TW_TYPE_INT32, cbSetResWidth, cbGetResWidth, this, "step=1 min=4");
	TwAddVarCB(DUC->g_pTweakBar, "Resolution Height", TW_TYPE_INT32, cbSetResHeight, cbGetResHeight, this, "step=1 min=4");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	reset();

	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	
	/* setup something to diffuse */
	// something hot
	for (int x = 1; x < m_iGridWidth / 2; x++) {
		for (int y = 1; y < m_iGridHeight / 2; y++) {
			T[m_iCurrGrid]->set(x, y, 1.0);
		}
	}
	// something cold
	for (int x = m_iGridWidth / 2; x < m_iGridWidth; x++) {
		for (int y = m_iGridHeight / 2; y < m_iGridHeight; y++) {
			T[m_iCurrGrid]->set(x, y, -1.0);
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
	Grid* oldT = T[m_iCurrGrid];
	// update current grid
	m_iCurrGrid = (m_iCurrGrid + 1) % 2;
	Grid* newT = T[m_iCurrGrid];
	// Note: We only read from the boundary, we do not calc or set anything there

	const Real dx = 1.0 / m_iGridWidth;
	const Real dy = 1.0 / m_iGridHeight;
	const Real dt = timeStep;

	for (int x = 1; x <= m_iGridWidth; x++) {
		for (int y = 1; y <= m_iGridHeight; y++) {

			Real tmpX = oldT->get(x + 1, y) - 2 * oldT->get(x, y) + oldT->get(x - 1, y);
			tmpX /= dx * dx;
			Real tmpY = oldT->get(x, y + 1) - 2 * oldT->get(x, y) + oldT->get(x, y - 1);
			tmpY /= dy * dy;
			
			Real tmp = (tmpX + tmpY) * m_fDiffusionCoefficient * dt;
			tmp += oldT->get(x, y);

			newT->set(x, y, tmp);
		}
	}

	return newT;
}

void DiffusionSimulator::setupB(std::vector<Real>& b) {
	// set vector B[sizeX*sizeY]
	for (int x = 0; x < T[m_iCurrGrid]->Width(); x++) {
		for (int y = 0; y < T[m_iCurrGrid]->Height(); y++) {
			b.at(x * y) = T[m_iCurrGrid]->get(x, y);
		}
	}
}

void DiffusionSimulator::fillT(std::vector<Real> res_x) {
	// fill T with solved vector x
	// make sure that the temperature in boundary cells stays zero
	for (int x = 1; x <= m_iGridWidth; x++) {
		for (int y = 1; y < m_iGridHeight; y++) {
			Real val = res_x.at(x * y);
			T[m_iCurrGrid]->set(x, y, val);
		}
	}
}

void setupA(SparseMatrix<Real>& A, double factor) {
	// setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}

void DiffusionSimulator::diffuseTemperatureImplicit() {
	// solve A T = b
	// to be implemented
	const int N = (m_iGridWidth+2) * (m_iGridHeight+2);//N = sizeX*sizeY*sizeZ
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
	fillT(x);//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	Grid* tmp = NULL;
	switch (m_iTestCase)
	{
	case 0:
		//tmp = T;
		//T = diffuseTemperatureExplicit(timeStep);
		//delete(tmp); // don't leak memory!
		//tmp = NULL;
		diffuseTemperatureExplicit(timeStep); // Note: we swap our grids internally
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

Real clamp(Real v, Real min, Real max) {
	return v < min ? min : (v > max ? max : v);
}

Vec3 lerp(Vec3 a, Vec3 b, Real alpha) {
	return a * (1.0 - alpha) + b * alpha;
}

void DiffusionSimulator::drawObjects()
{
	// Note: We render a sphere for each grid point with a color from red==hot to white==cold
	// We render the grid in the middle of the unit cube
	// color: ==0=>black, <0=>white, >0=>red

	const Vec3 white = Vec3(0xFF, 0xFF, 0xFF) / 255.0f;
	const Vec3 black = Vec3(0x0, 0x0, 0x0);
	const Vec3 red = Vec3(0xFF, 0x0, 0x0) / 255.0f; // #BB4844

	// render grid points
	const Real dx = 1.0 / (m_iGridWidth-1);
	const Real dy = 1.0 / (m_iGridHeight-1);
	const Real offsetX = -0.5 - dx;
	const Real offsetY = 0.5 + dy;

	Grid* curT = T[m_iCurrGrid];
	for (int x = 0; x < curT->Width(); x++) {
		for (int y = 0; y < curT->Height(); y++) {
			Vec3 pos = Vec3(offsetX + x * dx, offsetY - y * dy, 0.0);
			Real val = clamp(curT->get(x, y), -1.0, 1.0);
			Vec3 col;
			if (val >= 0) {
				col = lerp(black, white, val);
			}
			else {
				col = lerp(black, red, -val);
			}

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
