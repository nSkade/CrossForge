#pragma once

#include "IIKSolver.hpp"

namespace CForge {
using namespace Eigen;

class IKSfabrik : public IIKSolver {
public:
	void solve(std::string segmentName, IKController* pController);

	/**
	 * @brief equ to IKController::forwardKinematics, compute local pos, rot from global
	*/
	void backwardKinematics(std::string segmentName, IKController* pController,const std::vector<Vector3f>& fbrkPoints);

	std::vector<Vector3f> fbrkPoints; // global position for fabrik calculation
	//TODOff(skade) single iterations useful?
	//void solveForward();
	//void solveBackward();
private:
};

}//CForge
