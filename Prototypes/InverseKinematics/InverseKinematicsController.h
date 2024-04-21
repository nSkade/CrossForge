#pragma once

#include "../../crossforge/AssetIO/T3DMesh.hpp"
#include "../../crossforge/Graphics/UniformBufferObjects/UBOBoneData.h"
#include "../../crossforge/Graphics/Shader/ShaderCode.h"
#include "../../crossforge/Graphics/Shader/GLShader.h"

#include "JointLimits/HingeLimits.h"

#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>

namespace CForge {
	class InverseKinematicsController : public SkeletalAnimationController {
	public:
		struct EndEffectorData { // X corresponds to entry from end-effector to root
			Eigen::Matrix3Xf LocalEndEffectorPoints;  // current local joint positions, applied onto Controller
			Eigen::Matrix3Xf GlobalEndEffectorPoints; // current global joint positions
			Eigen::Matrix3Xf GlobalTargetPoints;      // target global joint positions
		};

		struct IKJoint {
			Eigen::Vector3f GlobalPosition;
			Eigen::Quaternionf GlobalRotation;

			EndEffectorData* pEndEffectorData;
			JointLimits* pLimits;
		};

		/**
		 * @brief Segment of Skeleton on which IK is applied to.
		 */
		struct IKSegment {
			std::string name;
			std::vector<SkeletalJoint*> joints; // front() is end-effector joint
		};

		//TODO(skade) SPOT, remove struct
		/**
		 * @brief EndEffector object for interaction and visualization.
		 */
		struct SkeletalEndEffector : public CForgeObject {
			int32_t JointID;
			std::string JointName;
			std::string segmentName;
			Eigen::Matrix3Xf EndEffectorPoints;
			Eigen::Matrix3Xf TargetPoints;

			SkeletalEndEffector(void) : CForgeObject("InverseKinematicsController::SkeletalEndEffector") {
				JointID = -1;
				segmentName = "";
			}
		};

		InverseKinematicsController(void);
		~InverseKinematicsController(void);

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
		std::vector<SkeletalAnimationController::SkeletalJoint*> retrieveSkeleton(void) const;
		void updateSkeletonValues(std::vector<SkeletalAnimationController::SkeletalJoint*>* pSkeleton);

		std::vector<SkeletalEndEffector*> retrieveEndEffectors(void) const;
		void updateEndEffectorValues(std::vector<SkeletalEndEffector*>* pEndEffectors);
		void translateTarget(std::string segmentName, Eigen::Vector3f Translation);
		Eigen::Matrix3Xf getTargetPoints(std::string segmentName);

		/**
		 * Update IK Bone values to current animation frame.
		 */
		void updateBones(Animation* pAnim);
		void updateEndEffectorPoints();

	protected:
		
		void initJointProperties(T3DMesh<float>* pMesh, const nlohmann::json& ConstraintData);
		void initSkeletonStructure(T3DMesh<float>* pMesh, const nlohmann::json& StructureData);
		void buildKinematicChain(std::string name, std::string rootName, std::string endEffectorName);
		void initEndEffectorPoints();
		void initTargetPoints(void);

		// end-effector -> root CCD IK
		void ikCCD(std::string segmentName);
		void rotateGaze();
		Eigen::Quaternionf computeUnconstrainedGlobalRotation(IKJoint* pJoint, EndEffectorData* pEffData);
		void forwardKinematics(SkeletalJoint* pJoint);

		std::map<SkeletalJoint*,IKJoint*> m_IKJoints; // extends m_Joints

		//TODOf(skade) name included here and in IKSegment, improve SPOT
		std::map<std::string,IKSegment> m_JointChains;

		float m_thresholdDist = 1e-6f;
		float m_thresholdPosChange = 1e-6f;

		int32_t m_MaxIterations = 50;
	};//InverseKinematicsController

}//CForge
