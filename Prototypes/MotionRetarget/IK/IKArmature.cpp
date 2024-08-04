#include "IKArmature.hpp"
#include <Prototypes/MotionRetarget/IKController.hpp>

namespace CForge {
using namespace Eigen;

void IKArmature::solve(IKController* pController) {
	for (auto& c : m_jointChains) {
		if (c.ikSolver)
			c.ikSolver->solve(c.name,pController);

		pController->forwardKinematics();
	}
}

}//CForge
