#pragma once
#include "IIKSolver.hpp"

//class IKJoint;
//class Quaternionf;

namespace CForge {

class IKSccd : public IIKSolver {
public:
	enum Type {
		BACKWARD,
		FORWARD,
	} m_type = BACKWARD;
	void solve(std::string segmentName, IKController* pController);

	//TODO(skade) fix or remove
	//// calculate rotation depending on whole chain
	//Quaternionf computeUnconstrainedGlobalRotation(IKJoint* pJoint, IKController::EndEffectorData* pEffData);
private:
};

}//CForge
