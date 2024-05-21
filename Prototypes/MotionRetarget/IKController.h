#pragma once

#include "../../crossforge/AssetIO/T3DMesh.hpp"
#include "../../crossforge/Graphics/UniformBufferObjects/UBOBoneData.h"
#include "../../crossforge/Graphics/Shader/ShaderCode.h"
#include "../../crossforge/Graphics/Shader/GLShader.h"

#include "JointLimits/HingeLimits.h"

#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>

namespace CForge {
	class IKController : public SkeletalAnimationController {
	public:
		struct EndEffectorData { // X corresponds to entry from end-effector to root
			Eigen::Matrix3Xf EEPosLocal;      // current local joint positions, applied onto Controller
			Eigen::Matrix3Xf EEPosGlobal;     // current global joint positions
			Eigen::Matrix3Xf TargetPosGlobal; // target global joint positions
		};

		//TODO(skade)
		struct IKTarget {
			Eigen::Vector3f pos;
			float influence = 1.; //TODO(skade)
		};

		struct IKJoint {
			Eigen::Vector3f GlobalPosition;
			Eigen::Quaternionf GlobalRotation;

			Eigen::Vector3f LocalPosition; //TODO(skade) requred? corresponded to EEPosLocal
			std::vector<IKTarget> TargetPosGlobal; // Global target Positions the Joint tries to reach
			EndEffectorData* pEndEffectorData; //TODO(skade) remove
			

			JointLimits* pLimits;
		};

		//TODO(skade) priority of IK Segments?
		/**
		 * @brief Segment of Skeleton on which IK is applied to.
		 */
		struct IKSegment {
			std::string name;
			std::vector<SkeletalJoint*> joints; // front() is end-effector joint
		};

		//TODO(skade) improve SPOT
		/**
		 * @brief EndEffector object for interaction and visualization outside of this class.
		 */
		struct SkeletalEndEffector {
			SkeletalJoint* joint = nullptr;
			IKJoint* jointIK = nullptr;
			std::string segmentName;
		};

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

		/**
		 * \brief Returns reference to m_IKJoints.
		 */
		std::vector<SkeletalEndEffector> retrieveEndEffectors(void);
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
		void ikJacobi(std::string segmentName);
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
	};//IKController

}//CForge
