#pragma once
#include "IIKSolver.hpp"

//class IKJoint;
//class Quaternionf;

namespace CForge {

class IKSolverCCD : public IIKSolver {
public:
	enum CCDtype {
		BACKWARD,
		FORWARD,
	};

	template<CCDtype type>
	void solve(std::string segmentName, IKController* pController);

	//TODO(skade) fix or remove
	//// calculate rotation depending on whole chain
	//Quaternionf computeUnconstrainedGlobalRotation(IKJoint* pJoint, IKController::EndEffectorData* pEffData);
private:
};

}//CForge
