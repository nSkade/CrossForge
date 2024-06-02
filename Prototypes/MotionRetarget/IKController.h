#pragma once

#include "../../crossforge/AssetIO/T3DMesh.hpp"
#include "../../crossforge/Graphics/UniformBufferObjects/UBOBoneData.h"
#include "../../crossforge/Graphics/Shader/ShaderCode.h"
#include "../../crossforge/Graphics/Shader/GLShader.h"

//#include "JointLimits/HingeLimits.h"

#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>

#include "Constraints/IKTarget.hpp"

namespace CForge {
using namespace Eigen;

class IKController : public SkeletalAnimationController {
public:
	//struct EndEffectorData { // X corresponds to entry from end-effector to root
	//	Eigen::Matrix3Xf EEPosLocal;      // current local joint positions, applied onto Controller
	//	Eigen::Matrix3Xf EEPosGlobal;     // current global joint positions
	//	Eigen::Matrix3Xf TargetPosGlobal; // target global joint positions
	//};

	struct IKJoint {
		Eigen::Vector3f posGlobal;
		Eigen::Quaternionf rotGlobal;

		Eigen::Vector3f posLocal;         //TODO(skade) requred? corresponded to EEPosLocal
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
		std::vector<SkeletalJoint*> joints; // front() is end-effector joint
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

	/**
	 * \brief Returns reference to m_IKJoints.
	 */
	//std::vector<SkeletalEndEffector> retrieveEndEffectors(void);

	IKController(void);
	~IKController(void);

	// pMesh has to hold skeletal definition
	void init(T3DMesh<float>* pMesh);
	void init(T3DMesh<float>* pMesh, std::string ConfigFilepath);
	void update(float FPSScale);
	void clear(void);

	//void applyAnimation(bool UpdateUBO = true);
	void applyAnimation(Animation* pAnim, bool UpdateUBO = true);

	void retrieveSkinningMatrices(std::vector<Eigen::Matrix4f>* pSkinningMats);

	SkeletalAnimationController::SkeletalJoint* getBone(uint32_t idx);
	uint32_t boneCount();
	void updateSkeletonValues(std::vector<SkeletalAnimationController::SkeletalJoint*>* pSkeleton);

	void translateTarget(std::string segmentName, Eigen::Vector3f Translation);
	//Eigen::Matrix3Xf getTargetPoints(std::string segmentName);

	/**
	 * @brief Update IK Bone values to current animation frame.
	 */
	void updateBones(Animation* pAnim);
	//void updateEndEffectorPoints();
	std::vector<IKTarget*> getTargets() { return m_targets; };

	//TODO(skade) find better solution
	IKSegment* getSegment(IKTarget* target) {
		for (auto& a : m_JointChains) {
			if (a.second.target == target)
				return &a.second;
		}
	}

protected:
	
	void initJointProperties(T3DMesh<float>* pMesh, const nlohmann::json& ConstraintData);
	void initSkeletonStructure(T3DMesh<float>* pMesh, const nlohmann::json& StructureData);
	void buildKinematicChain(std::string name, std::string rootName, std::string endEffectorName);
	void initEndEffectorPoints();

	//TODO(skade) remove
	/**
	 * @brief inits for every skeleton endeffector a target
	*/
	void initTargetPoints();
	std::vector<IKTarget*> m_targets;

	// end-effector -> root CCD IK
	//void ikCCDglobal(std::string segmentName);
	//TODO(skade) remove
	void ikCCD(std::string segmentName);
	void rotateGaze();

	//TODO(skade) remove 
	//TODO(skade) EndEffectorData
	//Eigen::Quaternionf computeUnconstrainedGlobalRotation(IKJoint* pJoint, EndEffectorData* pEffData);

	/**
	 * @brief Computes global position and rotation of joints of the skeletal hierarchy.
	*/
	void forwardKinematics(SkeletalJoint* pJoint);

	std::map<SkeletalJoint*,IKJoint*> m_IKJoints; // extends m_Joints

	//TODOf(skade) name included here and in IKSegment, improve SPOT
	std::map<std::string,IKSegment> m_JointChains;

	float m_thresholdDist = 1e-6f;
	float m_thresholdPosChange = 1e-6f;

	int32_t m_MaxIterations = 50;
};//IKController

}//CForge

