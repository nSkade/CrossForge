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
private:
};

}//CForge
