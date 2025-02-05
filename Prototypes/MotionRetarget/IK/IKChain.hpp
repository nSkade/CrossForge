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

	//TODO(skade) target pos needs to be handled by iksolver
	//std::vector<IKTarget> TargetPosGlobal; // Global target Positions the Joint tries to reach

	//JointLimits* pLimits;
};
//TODO(skade)
//class IKSegment {
//public:
//private:
//	IKSegment* m_pParent;
//	std::vector<IKSegment*> m_pChilds;
//};

//TODO(skade) new structures

//TODO(skade) priority of IK Segments?
/**
* @brief Segment of Skeleton on which IK is applied to.
*/
struct IKChain {
	std::string name;
	std::vector<SkeletalAnimationController::SkeletalJoint*> joints; // front() is end-effector joint
	std::weak_ptr<IKTarget> target;

	//IKJoint* pRoot = nullptr; //TODO(skade) make sure memory safe, IKChain always deleted before corr controller

	//float weight = 1.; // weight used for centroid interpolation,
	                   //TODO(skade) contribution equals: weight / sum(all chain weights on centoid)

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
