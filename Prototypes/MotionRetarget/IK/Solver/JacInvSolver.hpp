#pragma once

#include "IIKSolver.hpp"

namespace CForge {
using namespace Eigen;

/**
 * @brief Jacobian Inverse Solver
*/
class IKSjacInv : public IIKSolver {
public:
	enum Type {
		TRANSPOSE,
		SVD,
		DLS,
	} m_type = DLS;
	float m_dlsDamping = 3.;

	void solve(std::string segmentName, IKController* pController);
	MatrixXd calculateJacobianNumerical(std::string segmentName, IKController* pController);
	MatrixXd DampedLeastSquare(MatrixXd jac);

private:
};

}//CForge
