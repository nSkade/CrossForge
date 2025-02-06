#pragma once

#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>
#include <Prototypes/MotionRetarget/IK/IKTarget.hpp>

#include "Solver/CCDSolver.hpp"
#include "Solver/FABRIKSolver.hpp"
#include "Solver/JacInvSolver.hpp"

namespace CForge {

struct IKJoint {
	Eigen::Vector3f posGlobal;
	Eigen::Quaternionf rotGlobal;

	//TODOff(skade) target pos needs to be handled by iksolver
	//std::vector<IKTarget> TargetPosGlobal; // Global target Positions the Joint tries to reach
	//JointLimits* pLimits;
};

//TODOff(skade) neighbour info for IKChain?
//class IKSegment {
//public:
//private:
//	IKSegment* m_pParent;
//	std::vector<IKSegment*> m_pChilds;
//};

/**
* @brief Segment of Skeleton on which IK is applied to.
*/
struct IKChain {
	std::string name;
	std::vector<SkeletalAnimationController::SkeletalJoint*> joints; // front() is end-effector joint
	std::weak_ptr<IKTarget> target;

	//float weight = 1.; // weight used for centroid interpolation,
	                   //TODOf(skade) contribution equals: weight / sum(all chain weights on centoid)

	std::unique_ptr<IIKSolver> ikSolver = std::make_unique<IKSjacInv>();
	//std::vector<std::pair<IKJoint*,IKTarget*>> pEndEff;
};
//class IKChain {
//public:
//private:
//	IKSegment* m_pRoot;
//	std::vector<IKSegment*> m_pSegments;
//};

}//CForge
