#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
	/*
	 * (0,0) ... (m,0)
	 *  ...       ...
	 * (0,n) ... (m,n)
	 */
public:
	// Construtors
	Grid(int m, int n) : m(m), n(n) {
		matrix = (Real*)calloc((size_t)m * (size_t)n, sizeof(Real));
	}

	~Grid() {
		free(matrix);
	}

	Real get(int x, int y) {
		// sanity check
		if (x < 0 || x >= m || y < 0 || y >= n) {
			std::cerr << "Invalid indices x=" << x << " y=" << y << " for matrix(" << m << "x" << n << ")" << std::endl;
			return NAN;
		}

		return matrix[x + y * m];
	}

	void set(int x, int y, Real value) {
		// sanity check
		if (x < 0 || x >= m || y < 0 || y >= n) {
			std::cerr << "Invalid indices x=" << x << " y=" << y << " for matrix(" << m << "x" << n << ")" << std::endl;
		}

		// Do not set values at boundary
		if (x == 0 || x == m-1 || y == 0 || y == n-1) {
			std::cerr << "Trying to set boundary at (" << x << "," << y << ") >:c" << std::endl;
			return;
		}

		matrix[x + y * m] = value;
	}

	/*Real& at(int x, int y) {
		// sanity check
		if (x < 0 || x >= m || y < 0 || y >= n) {
			std::cerr << "Invalid indices x=" << x << " y=" << y << " for matrix(" << m << "x" << n << ")" << std::endl;
			static Real nan = NAN;
			return nan;
		}
		
		return matrix[x + y * m];
	}*/

	int Width() {
		return m;
	}

	int Height() {
		return n;
	}

private:
	// Attributes
	int m;
	int n;

	Real *matrix;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit();

	int m_iGridWidth;
	int m_iGridHeight;

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid *T[2]; // save results of every time step
	int m_iCurrGrid; // Note: We swap between two grids, in order to avoid memory allocation each frame

	Real m_fDiffusionCoefficient;

	void setupB(std::vector<Real>& b);
	void fillT(std::vector<Real> x);
};

#endif