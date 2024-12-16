#include "IKArmature.hpp"
#include <Prototypes/MotionRetarget/IK/IKController.hpp>

namespace CForge {
using namespace Eigen;

void IKArmature::solve(IKController* pController) {
	//TODO(skade)f solve every chain from endeffector to centroids root
	for (auto& c : m_jointChains) {
		if (c.ikSolver)
			c.ikSolver->solve(c.name,pController);

		pController->forwardKinematics();
	}
}

}//CForge
