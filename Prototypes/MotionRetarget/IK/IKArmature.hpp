#pragma once

#include "IKChain.hpp"

namespace CForge {
using namespace Eigen;
class IKController;

/**
 * @brief Interface for Joint Constraints, applied during IK.
 * different IK methods need different Constraint Implementations for the same constraint Type.
 * Abstract interface, and automatically chose correct constraint implementation depending on solver.
*/
template<typename ConstraintImpl>
class IConstraint {
//	virtual void apply(IKController::IKSegment* seg, SkeletalAnimationController::SkeletalJoint* cJoint);
};//IConstraint

/**
 * @brief IKArmature solves posture for a whole skeleton consisting of multiple ik chains
*/
class IKArmature {
public:
	void solve(IKController* pController);

	//std::vector<IKChain> m_trueIKChains;

	std::vector<IKChain> m_jointChains;
	//std::vector<IConstraint> m_constraints;
private:
};

}//CForge
