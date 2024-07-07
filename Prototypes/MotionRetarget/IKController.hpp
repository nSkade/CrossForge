#pragma once

#include <crossforge/AssetIO/T3DMesh.hpp>
#include <crossforge/Graphics/UniformBufferObjects/UBOBoneData.h>
#include <crossforge/Graphics/Shader/ShaderCode.h>
#include <crossforge/Graphics/Shader/GLShader.h>
#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>

#include "Animation/JointPickable.hpp"
#include "Constraints/IKTarget.hpp"

#include "IKSolver/CCDSolver.hpp"
#include "IKSolver/FABRIKSolver.hpp"

//#include "JointLimits/HingeLimits.h"

#include "IKSolver/IKChain.hpp"

namespace CForge {
using namespace Eigen;
class RenderDevice;

class IKController : public SkeletalAnimationController {
public:
	enum TestIKslvSelect {
		IKSS_CCD_F,
		IKSS_CCD_B,
		IKSS_CCD_FABRIK,
	} testIKslvSelect = IKSS_CCD_B;
	SkeletalJoint* getRoot() { return m_pRoot; }

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

	//TODO(skade) remove, unnused
	//void updateBones(Animation* pAnim);
	std::vector<std::weak_ptr<IKTarget>> getTargets() {
		return std::vector<std::weak_ptr<IKTarget>>(m_targets.begin(),m_targets.end());
	};
	void getTargets(std::vector<std::shared_ptr<IKTarget>>* targets) {
		*targets = m_targets;
	};

	//TODO(skade) find better solution
	IKChain* getIKChain(IKTarget* target) {
		for (auto& a : m_JointChains) {
			if (a.second.target == target)
				return &a.second;
		}
		return nullptr;
	}

	/**
	 * @brief Computes global position and rotation of joints of the skeletal hierarchy.
	*/
	void forwardKinematics(SkeletalJoint* pJoint);

	std::map<SkeletalJoint*,IKJoint*> m_IKJoints; // extends m_Joints

	//TODOf(skade) name included here and in IKChain, improve SPOT
	std::map<std::string,IKChain> m_JointChains;
	IKSolverCCD m_iksCCD;
	std::vector<IKSolverFABRIK> m_iksFABRIK;
	std::vector<std::weak_ptr<JointPickable>> getJointPickables() {
		return std::vector<std::weak_ptr<JointPickable>>(m_jointPickables.begin(),m_jointPickables.end());
	};
	void renderJointPickables(RenderDevice* pRenderDev);

	void setJointOpacity(float opacity) { m_jointPickableMesh.setOpacity(opacity); };
	float getJointOpacity() { return m_jointPickableMesh.getOpacity(); };
	
private:
	std::vector<std::shared_ptr<JointPickable>> m_jointPickables;
	JointPickableMesh m_jointPickableMesh;
	
	void initJointProperties(T3DMesh<float>* pMesh, const nlohmann::json& ConstraintData);
	void initSkeletonStructure(T3DMesh<float>* pMesh, const nlohmann::json& StructureData);
	void buildKinematicChain(std::string name, std::string rootName, std::string endEffectorName);

	//TODO(skade) remove
	/**
	 * @brief inits for every skeleton endeffector a target
	*/
	void initTargetPoints();
	void clearTargetPoints();
	void updateTargetPoints();
	std::vector<std::shared_ptr<IKTarget>> m_targets;

	//TODO(skade) rotate head
	//void rotateGaze();
};//IKController

}//CForge
