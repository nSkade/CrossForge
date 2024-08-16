#pragma once

namespace CForge {

class IKController;

/**
 * @brief Interface for various Inverse Kinematics Solvers.
 *        Solvers handle a single IKChain.
*/
class IIKSolver {
public:
	virtual void solve(std::string segmentName, IKController* pController) {};
	
protected:
	float m_thresholdDist = 1e-6f;
	float m_thresholdPosChange = 1e-6f;

	int32_t m_MaxIterations = 50;
};//IIKSolver

}//CForge
