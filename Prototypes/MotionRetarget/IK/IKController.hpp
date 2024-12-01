#pragma once

#include <crossforge/AssetIO/T3DMesh.hpp>
#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>

#include <Prototypes/MotionRetarget/Animation/JointPickable.hpp>

#include "IKTarget.hpp"
#include "IKArmature.hpp"

//#include "JointLimits/HingeLimits.h"

namespace CForge {
using namespace Eigen;
class RenderDevice;

class IKController : public SkeletalAnimationController {
public:
	SkeletalJoint* getRoot() { return m_pRoot; }

	IKController(void);
	~IKController(void);

	// pMesh has to hold skeletal definition
	void init(T3DMesh<float>* pMesh);
	void init(T3DMesh<float>* pMesh, std::string ConfigFilepath);
	void initRestpose();
	void update(float FPSScale);
	void clear(void);

	//void applyAnimation(bool UpdateUBO = true);
	void applyAnimation(Animation* pAnim, bool UpdateUBO = true);

	void retrieveSkinningMatrices(std::vector<Eigen::Matrix4f>* pSkinningMats);

	SkeletalAnimationController::SkeletalJoint* getBone(uint32_t idx);
	uint32_t boneCount();

	IKChain* getIKChain(std::string name) {
		for (auto& a : getJointChains()) {
			if (a.name == name)
				return &a;
		}
		return nullptr;
	}

	/**
	 * @brief Computes global position and rotation of joints of the skeletal hierarchy.
	*/
	void forwardKinematics(SkeletalJoint* pJoint);
	void forwardKinematics() { forwardKinematics(m_pRoot); };

	//TODOff(skade) cleanup
	// helper functions 
	std::vector<IKChain>& getJointChains() { return m_ikArmature.m_jointChains; };

	std::vector<std::vector<Vector3f>> getFABRIKpoints() {
		std::vector<std::vector<Vector3f>> ret;
		for (IKChain& ikc : m_ikArmature.m_jointChains) {
			if (IKSfabrik* iks = dynamic_cast<IKSfabrik*>(ikc.ikSolver.get())) {
				ret.push_back(iks->fbrkPoints);
			}
		}
		return ret;
	};

	std::vector<std::weak_ptr<JointPickable>> getJointPickables() {
		std::vector<std::weak_ptr<JointPickable>> ret;
		ret.reserve(m_jointPickables.size());
		for (auto& [sj,jp] : m_jointPickables)
			ret.push_back(jp);
		return ret;
	};
	std::weak_ptr<JointPickable> getJointPickable(SkeletalJoint* joint) {
		return m_jointPickables[joint];
	}

	//TODOff(skade) unify with chain editor func
	/**
	 * @brief Builds new IKChain from names and places them into getJointChains()
	*/
	void buildKinematicChain(std::string name, std::string rootName, std::string endEffectorName);

	/**
	 * @brief inits for every skeleton endeffector a target
	*/
	void initTargetPoints();
	void clearTargetPoints();
public:
	// no smartptr needed as controller owns SkeletalJoint
	std::map<SkeletalJoint*,IKJoint> m_IKJoints; // extends m_Joints
	IKArmature m_ikArmature;
	std::vector<std::shared_ptr<IKTarget>> m_targets;
private:
	std::map<SkeletalJoint*,std::shared_ptr<JointPickable>> m_jointPickables;
	JointPickableMesh m_jointPickableMesh;
	
	/**
	 * @brief Initializes IKJoints
	*/
	void initJointProperties(T3DMesh<float>* pMesh);

	//TODOff(skade) store armature as json

	// Json interface

	void initConstraints(T3DMesh<float>* pMesh, const nlohmann::json& ConstraintData);
	void initSkeletonStructure(T3DMesh<float>* pMesh, const nlohmann::json& StructureData);

	/**
	 * @brief update target points from corresponding current animation joint positions.
	*/
	void updateTargetPoints();

	//TODOff(skade) rotate head
	//void rotateGaze();
};//IKController

}//CForge
