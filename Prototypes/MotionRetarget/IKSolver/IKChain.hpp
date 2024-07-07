#pragma once

#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>
#include <Prototypes/MotionRetarget/Constraints/IKTarget.hpp>

namespace CForge {

struct IKJoint {
	Eigen::Vector3f posGlobal;
	Eigen::Quaternionf rotGlobal;

	IKJoint* pParent;
	std::vector<IKJoint*> pChilds;

	//TODO(skade) target pos needs to be handled by iksolver
	//std::vector<IKTarget> TargetPosGlobal; // Global target Positions the Joint tries to reach

	//JointLimits* pLimits;
};

//TODO(skade) new structures

//TODO(skade) priority of IK Segments?
/**
* @brief Segment of Skeleton on which IK is applied to.
*/
struct IKChain {
	std::string name;
	std::vector<SkeletalAnimationController::SkeletalJoint*> joints; // front() is end-effector joint
	IKTarget* target;
};

//TODO(skade)
//class IKSegment {
//public:
//private:
//	IKSegment* m_pParent;
//	std::vector<IKSegment*> m_pChilds;
//};
//
//class IKChain {
//public:
//private:
//	IKSegment* m_pRoot;
//	std::vector<IKSegment*> m_pSegments;
//};

}//CForge
