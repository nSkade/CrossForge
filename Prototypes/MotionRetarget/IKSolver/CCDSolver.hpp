#pragma once
#include "IIKSolver.hpp"

class IKJoint;
class EndEffectorData;
class Quaternionf;

namespace CForge {
class IKSolverCCD : public IIKSolver {
public:
	//TODO(skade)
	//void solveForward();
	//void solveBackward();
	void ikCCD(std::string segmentName);
	Quaternionf computeUnconstrainedGlobalRotation(IKJoint* pJoint, IKController::EndEffectorData* pEffData);
private:
	float m_thresholdDist = 0.025; //TODO(skade)
	uint32_t m_MaxIterations = 50; //TODO(skade)
};

}//CForge

