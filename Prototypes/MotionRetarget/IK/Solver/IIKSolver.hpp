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
	
	int32_t m_MaxIterations = 100;
	float m_thresholdDist = 1e-6f;
	float m_thresholdPosChange = 1e-6f;
protected:
};//IIKSolver

}//CForge
