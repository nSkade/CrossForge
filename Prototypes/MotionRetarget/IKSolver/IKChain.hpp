#pragma once

#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>
#include <Prototypes/MotionRetarget/Constraints/IKTarget.hpp>

namespace CForge {

//struct EndEffectorData { // X corresponds to entry from end-effector to root
//	Eigen::Matrix3Xf EEPosLocal;      // current local joint positions, applied onto Controller
//	Eigen::Matrix3Xf EEPosGlobal;     // current global joint positions
//	Eigen::Matrix3Xf TargetPosGlobal; // target global joint positions
//};

struct IKJoint {
	Eigen::Vector3f posGlobal;
	Eigen::Quaternionf rotGlobal;

	//Eigen::Vector3f posLocal;         //TODO(skade) requred? corresponded to EEPosLocal
	//TODO(skade) target pos needs to be handled by iksolver
	//std::vector<IKTarget> TargetPosGlobal; // Global target Positions the Joint tries to reach

	//EndEffectorData* pEndEffectorData;     //TODO(skade) remove
	
	//JointLimits* pLimits;
};

//TODO(skade) priority of IK Segments?
/**
 * @brief Segment of Skeleton on which IK is applied to.
 */
struct IKSegment {
	std::string name;
	std::vector<SkeletalAnimationController::SkeletalJoint*> joints; // [0] is end-effector joint
	IKTarget* target;
};

//TODO(skade) improve SPOT remove
/**
 * @brief EndEffector object for interaction and visualization outside of this class.
 */
//struct SkeletalEndEffector {
//	SkeletalJoint* joint = nullptr;
//	IKJoint* jointIK = nullptr;
//	std::string segmentName;
//};

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
