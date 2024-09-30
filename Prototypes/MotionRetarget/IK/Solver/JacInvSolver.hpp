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

	MatrixXd DampedLeastSquare(MatrixXd jac);

	/**
	 * @brief calculates Jacobian matrix of chain regarding influence on end effector,
	 *        the jacobian looks as follows:
	 * for every joint axis write change on each dim on endeffector
	 *              joint 1 x                      joint 1 y                     joint 1 z
	 *    eef1 x  | {angle dx, angle dy, angle dz, angle dx, angle dy, angle dz, angle dx, angle dy, angle dz,
	 *    eff1 y  | {angle dx, angle dy, angle dz, angle dx, angle dy, angle dz, angle dx, angle dy, angle dz,
	 *    eff1 z  | {angle dx, angle dy, angle dz, angle dx, angle dy, angle dz, angle dx, angle dy, angle dz,
	 *    eff2 x  v ,...}
	 *          all joints
	*/
	MatrixXd calculateJacobianNumerical(std::string segmentName, IKController* pController);

private:
};

}//CForge
