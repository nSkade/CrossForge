#pragma once

#include "IKChain.hpp"

namespace CForge {

class IKController;

/**
 * @brief Interface for various Inverse Kinematics Solver.
 *        Solvers handle a single IKChain.
*/
class IIKSolver {
public:
	//TODO(skade) implement common interface
	
protected:
	//IKChain m_chain; //TODO(skade) chain
	float m_thresholdDist = 1e-6f;
	float m_thresholdPosChange = 1e-6f;

	int32_t m_MaxIterations = 50;
};//IIKSolver

}//CForge
