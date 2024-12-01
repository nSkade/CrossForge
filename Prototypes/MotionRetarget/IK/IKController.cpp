#include "IKController.hpp"

#include <crossforge/Graphics/Shader/SShaderManager.h>
#include <crossforge/Math/CForgeMath.h>
#include <crossforge/Core/SLogger.h>

#include <crossforge/Graphics/RenderDevice.h> // for JointVis

#include <Prototypes/MotionRetarget/CMN/EigenFWD.hpp>

#include <fstream>
#include <iostream>
#include <cmath>

namespace CForge {
using namespace Eigen;

//TODOfff(skade) CForgeObject classname wrong
IKController::IKController(void) : SkeletalAnimationController() {
	m_pRoot = nullptr;
	//m_pHead = nullptr;

	m_pShadowPassShader = nullptr;
	m_pShadowPassFSCode = nullptr;
	m_pShadowPassVSCode = nullptr;

#ifdef SHADER_GLES
	m_GLSLVersionTag = "300 es";
	m_GLSLPrecisionTag = "lowp";
#else
	m_GLSLVersionTag = "330 core";
	m_GLSLPrecisionTag = "lowp";
#endif

}//constructor

IKController::~IKController(void) {
	clear();
}//Destructor

void IKController::init(T3DMesh<float>* pMesh) {
	clear();

	if (!pMesh)
		throw NullpointerExcept("pMesh");
	if (pMesh->boneCount() == 0)
		throw CForgeExcept("Mesh has no bones!");
	
	T3DMesh<float>::SkeletalAnimation* pAnimation = nullptr;
	if (pMesh->skeletalAnimationCount() > 0)
		pAnimation = pMesh->getSkeletalAnimation(0); // used as initial pose of skeleton

	// copy structure
	for (uint32_t i = 0; i < pMesh->boneCount(); ++i) {
		const T3DMesh<float>::Bone* pRef = pMesh->getBone(i);
		SkeletalJoint* pJoint = new SkeletalJoint();

		pJoint->Parent = (pRef->pParent != nullptr) ? pRef->pParent->ID : -1;

		pJoint->ID = pRef->ID;
		pJoint->Name = pRef->Name;
		pJoint->OffsetMatrix = pRef->InvBindPoseMatrix;
		pJoint->SkinningMatrix = Matrix4f::Identity(); // computed during applyAnimation()

		pJoint->Children.reserve(pRef->Children.size());
		for (uint32_t k = 0; k < pRef->Children.size(); ++k)
			pJoint->Children.push_back(pRef->Children[k]->ID);
		m_Joints.emplace_back(pJoint);
	}//for[bones]

	// find root bone
	for (uint32_t i = 0; i < m_Joints.size(); ++i) {
		if (m_Joints[i]->Parent == -1) {
			m_pRoot = m_Joints[i];
			break;
		}
	}//for[joints]
	
	// initialize UBO
	m_UBO.init(m_Joints.size());

	for (uint32_t i = 0; i < m_Joints.size(); ++i) {
		m_UBO.skinningMatrix(i, m_Joints[i]->OffsetMatrix);
	}//for[joints]

	SShaderManager* pSMan = SShaderManager::instance();

	m_pShadowPassFSCode = pSMan->createShaderCode("Shader/ShadowPassShader.frag",
	                      m_GLSLVersionTag, 0, m_GLSLPrecisionTag);
	m_pShadowPassVSCode = pSMan->createShaderCode("Shader/ShadowPassShader.vert", m_GLSLVersionTag,
	                      ShaderCode::CONF_SKELETALANIMATION | ShaderCode::CONF_LIGHTING, m_GLSLPrecisionTag);

	ShaderCode::SkeletalAnimationConfig SkelConfig;
	SkelConfig.BoneCount = m_Joints.size();
	m_pShadowPassVSCode->config(&SkelConfig);

	std::vector<ShaderCode*> VSSources;
	std::vector<ShaderCode*> FSSources;
	VSSources.push_back(m_pShadowPassVSCode);
	FSSources.push_back(m_pShadowPassFSCode);

	m_pShadowPassShader = pSMan->buildShader(&VSSources, &FSSources, nullptr);

	pSMan->release();

	for (uint32_t i=0;i<m_Joints.size();++i)
		m_jointPickables[m_Joints[i]] = std::make_shared<JointPickable>(&m_jointPickableMesh,m_Joints[i],this);
	for (auto& [sj,jp] : m_jointPickables)
		jp->init();
		//m_jointPickables.emplace_back(std::make_shared<JointPickable>(&m_jointPickableMesh,m_Joints[i],this));

	initJointProperties(pMesh);
	initRestpose();
}

// pMesh has to hold skeletal definition
void IKController::init(T3DMesh<float>* pMesh, std::string ConfigFilepath) {
	init(pMesh);
	
	std::ifstream f(ConfigFilepath);
	const nlohmann::json ConfigData = nlohmann::json::parse(f);

	initConstraints(pMesh, ConfigData.at("JointLimits"));
	initSkeletonStructure(pMesh, ConfigData.at("SkeletonStructure"));
	forwardKinematics(m_pRoot); // initialize global positions and rotations of all joints
	initTargetPoints();
}//initialize

void IKController::initRestpose() {
	std::function<void(SkeletalAnimationController::SkeletalJoint* pJoint, Matrix4f offP)> initJoint;
	initJoint = [&](SkeletalAnimationController::SkeletalJoint* pJoint, Matrix4f offP) {
		Matrix4f iom = pJoint->OffsetMatrix.inverse();
		Matrix4f t = offP.inverse() * iom;

		// https://math.stackexchange.com/questions/237369/given-this-transformation-matrix-how-do-i-decompose-it-into-translation-rotati
		pJoint->LocalPosition = t.block<3,1>(0,3);
		pJoint->LocalScale = Vector3f(t.block<3,1>(0,0).norm(),
		                              t.block<3,1>(0,1).norm(),
		                              t.block<3,1>(0,2).norm());
		Matrix3f rotScale;
		rotScale.row(0) = pJoint->LocalScale;
		rotScale.row(1) = pJoint->LocalScale;
		rotScale.row(2) = pJoint->LocalScale;
		pJoint->LocalRotation = Quaternionf(t.block<3,3>(0,0).cwiseQuotient(rotScale));
		pJoint->LocalRotation.normalize();

		for (uint32_t i = 0; i < pJoint->Children.size(); ++i)
			initJoint(getBone(pJoint->Children[i]),iom);
	};
	initJoint(m_pRoot,Matrix4f::Identity());
}

void IKController::clear(void) {
	m_pRoot = nullptr;
	for (uint32_t i=0;i<m_Joints.size();++i)
		delete m_Joints[i];
	m_Joints.clear();
	
	m_IKJoints.clear();
	getJointChains().clear();
	
	m_UBO.clear();

	// instances get deleted by the Shader Manager
	m_pShadowPassShader = nullptr;
	m_pShadowPassFSCode = nullptr;
	m_pShadowPassVSCode = nullptr;

}//clear


void IKController::initJointProperties(T3DMesh<float>* pMesh) {
	//T3DMesh<float>::SkeletalAnimation* pAnimation = pMesh->getSkeletalAnimation(0); // used as initial pose of skeleton

	for (uint32_t i = 0; i < pMesh->boneCount(); ++i) {
		SkeletalJoint* pJoint = m_Joints[i];
		IKJoint pIKJoint;
		pIKJoint.posGlobal = Vector3f::Zero(); // computed after joint hierarchy has been constructed
		pIKJoint.rotGlobal = Quaternionf::Identity(); // computed after joint hierarchy has been constructed
		m_IKJoints[pJoint] = pIKJoint;
	}
}//initJointProperties

//TODOff(skade) parse constraint / ik config data
void IKController::initConstraints(T3DMesh<float>* pMesh, const nlohmann::json& ConstraintData) {
	T3DMesh<float>::SkeletalAnimation* pAnimation = pMesh->getSkeletalAnimation(0); // used as initial pose of skeleton

	for (uint32_t i = 0; i < pMesh->boneCount(); ++i) {
		//SkeletalJoint* pJoint = m_Joints[i];
		//IKJoint* pIKJoint = new IKJoint();

		//pIKJoint->posGlobal = Vector3f::Zero(); // computed after joint hierarchy has been constructed
		//pIKJoint->rotGlobal = Quaternionf::Identity(); // computed after joint hierarchy has been constructed
		//m_IKJoints[pJoint] = pIKJoint;

		//// create user defined joint constraints
		//const nlohmann::json& JointData = ConstraintData.at(pJoint->Name);
		//std::string Type = JointData.at("Type").get<std::string>();

		//if (Type == "Unconstrained")
		//	pIKJoint->pLimits = nullptr;

		//if (Type == "Hinge") {
		//	std::string Hinge = JointData.at("HingeAxis").get<std::string>();
		//	std::string Forward = JointData.at("BoneForward").get<std::string>();

		//	if (Hinge == Forward)
		//		throw CForgeExcept("JointLimits for '" + pJoint->Name
		//		                   + "': HingeAxis and BoneForward cannot be the same joint axis!");

		//	Vector3f HingeAxis = Vector3f::Zero();
		//	Vector3f BoneForward = Vector3f::Zero();

		//	if (Hinge == "x") HingeAxis = Vector3f::UnitX();
		//	else if (Hinge == "y") HingeAxis = Vector3f::UnitY();
		//	else if (Hinge == "z") HingeAxis = Vector3f::UnitZ();
		//	else throw CForgeExcept("JointLimits for '" + pJoint->Name + "': HingeAxis must be 'x', 'y' or 'z'!");

		//	if (Forward == "x") BoneForward = Vector3f::UnitX();
		//	else if (Forward == "y") BoneForward = Vector3f::UnitY();
		//	else if (Forward == "z") BoneForward = Vector3f::UnitZ();
		//	else throw CForgeExcept("JointLimits for '" + pJoint->Name + "': BoneForward must be 'x', 'y' or 'z'!");

		//	float MinRad = CForgeMath::degToRad(JointData.at("MinAngleDegrees").get<float>());
		//	float MaxRad = CForgeMath::degToRad(JointData.at("MaxAngleDegrees").get<float>());

		//	HingeLimits* pNewLimits = new HingeLimits(pJoint->LocalRotation, HingeAxis, BoneForward, MinRad, MaxRad);
		//	pIKJoint->pLimits = pNewLimits;
		//}
		
		////TODOff(skade) rewrite and abstract hinge limits

		//if (Type == "SwingXZTwistY") {
		//	float MinTwist = CForgeMath::degToRad(JointData.at("MinTwist").get<float>());
		//	float MaxTwist = CForgeMath::degToRad(JointData.at("MaxTwist").get<float>());
		//	float MinXSwing = CForgeMath::degToRad(JointData.at("MinXSwing").get<float>());
		//	float MaxXSwing = CForgeMath::degToRad(JointData.at("MaxXSwing").get<float>());
		//	float MinZSwing = CForgeMath::degToRad(JointData.at("MinZSwing").get<float>());
		//	float MaxZSwing = CForgeMath::degToRad(JointData.at("MaxZSwing").get<float>());
		//	
		//	SwingXZTwistYLimits* pNewLimits = new SwingXZTwistYLimits(pJoint->LocalRotation, MinXSwing, MaxXSwing, MinZSwing, MaxZSwing, MinTwist, MaxTwist);
		//	pIKJoint->pLimits = pNewLimits;
		//}

		//if (Type == "SwingXTwistY") {
		//	float MinTwist = CForgeMath::degToRad(JointData.at("MinTwist").get<float>());
		//	float MaxTwist = CForgeMath::degToRad(JointData.at("MaxTwist").get<float>());
		//	float MinSwing = CForgeMath::degToRad(JointData.at("MinSwing").get<float>());
		//	float MaxSwing = CForgeMath::degToRad(JointData.at("MaxSwing").get<float>());
		//	
		//	SwingXTwistYLimits* pNewLimits = new SwingXTwistYLimits(pJoint->LocalRotation, MinSwing, MaxSwing, MinTwist, MaxTwist);
		//	pIKJoint->pLimits = pNewLimits;
		//}

		//if (Type == "SwingZTwistY") {
		//	float MinTwist = CForgeMath::degToRad(JointData.at("MinTwist").get<float>());
		//	float MaxTwist = CForgeMath::degToRad(JointData.at("MaxTwist").get<float>());
		//	float MinSwing = CForgeMath::degToRad(JointData.at("MinSwing").get<float>());
		//	float MaxSwing = CForgeMath::degToRad(JointData.at("MaxSwing").get<float>());

		//	SwingZTwistYLimits* pNewLimits = new SwingZTwistYLimits(pJoint->LocalRotation, MinSwing, MaxSwing, MinTwist, MaxTwist);
		//	pIKJoint->pLimits = pNewLimits;
		//}

		//if (Type == "SwingXYTwistZ") {
		//	float MinTwist = CForgeMath::degToRad(JointData.at("MinTwist").get<float>());
		//	float MaxTwist = CForgeMath::degToRad(JointData.at("MaxTwist").get<float>());
		//	float MinXSwing = CForgeMath::degToRad(JointData.at("MinXSwing").get<float>());
		//	float MaxXSwing = CForgeMath::degToRad(JointData.at("MaxXSwing").get<float>());
		//	float MinYSwing = CForgeMath::degToRad(JointData.at("MinYSwing").get<float>());
		//	float MaxYSwing = CForgeMath::degToRad(JointData.at("MaxYSwing").get<float>());
		//	
		//	SwingXYTwistZLimits* pNewLimits = new SwingXYTwistZLimits(pJoint->LocalRotation, MinXSwing, MaxXSwing, MinYSwing, MaxYSwing, MinTwist, MaxTwist);
		//	pIKJoint->pLimits = pNewLimits;
		//}

		//if (Type == "SwingXTwistZ") ...
		//if (Type == "SwingYTwistZ") ...
		//if (Type == "SwingYZTwistX") ...
		//if (Type == "SwingYTwistX") ...
		//if (Type == "SwingZTwistX") ...
	}//for[bones]
}//initConstraints

void IKController::initSkeletonStructure(T3DMesh<float>* pMesh, const nlohmann::json& StructureData) {
	// create user defined skeleton segments 
	for (auto it : StructureData.items()) {
		if(it.value().contains("Root") && it.value().contains("EndEffector"))
			buildKinematicChain(it.key(),it.value().at("Root").get<std::string>(),it.value().at("EndEffector").get<std::string>());
	}
}//initSkeletonStructure

//TODOff(skade) cleanup
void IKController::buildKinematicChain(std::string name, std::string rootName, std::string endEffectorName) {
	//getJointChains().try_emplace(name,IKChain());
	//getJointChains().at(name).name = name;
	//std::vector<SkeletalJoint*>& joints = getJointChains().at(name).joints;

	IKChain nc;
	nc.name = name;
	std::vector<SkeletalJoint*>& joints = nc.joints;

	// fill chain in order end-effector -> chain root
	SkeletalJoint* pCurrent = nullptr;
	for (SkeletalJoint* i : m_Joints) {
		if (i->Name == endEffectorName) {
			pCurrent = i;
			break;
		}
	}
	if (!pCurrent)
		throw NullpointerExcept("pCurrent"); // couldn't find the end-effector joint

	SkeletalJoint* pEnd = nullptr;
	for (SkeletalJoint* i : m_Joints) {
		if (i->Name == rootName) {
			pEnd = i;
			break;
		}
	}
	if (!pEnd)
		throw NullpointerExcept("pEnd"); // couldn't find the root joint
			
	joints.push_back(pCurrent);
	do {
		pCurrent = m_Joints[pCurrent->Parent];
		if (pCurrent == m_pRoot && pEnd != m_pRoot)
			throw CForgeExcept("Reached root joint of skeleton without finding root joint of chain!");

		joints.push_back(pCurrent);
	} while (pCurrent->ID != pEnd->ID);

	auto& jc = getJointChains();
	jc.emplace_back(std::move(nc));
	//if (joints.size() < 2)
	//	throw CForgeExcept("Initialization of chain failed, joints size < 2");
}//buildKinematicChain

void IKController::initTargetPoints() {
	clearTargetPoints();
	for (auto& c : getJointChains()) {
		std::string name = c.name + " target";
		
		BoundingVolume bv;
		Vector3f d =Vector3f(0.05f, 0.05f, 0.05f);
		Box b; b.init(-d*.5,d*.5);
		bv.init(b);

		m_targets.emplace_back(std::make_shared<IKTarget>(name,bv));

		// assign position
		IKJoint& eff = m_IKJoints[c.joints[0]];
		m_targets.back()->pos = eff.posGlobal;

		//TODOff(skade) unify with add new target
		c.target = m_targets.back();
	}
}//initTargetPoints

//TODOf(skade) ui
void IKController::clearTargetPoints() {
	for (auto& c : getJointChains())
		c.target.reset();
	m_targets.clear();
}

//TODO(skade)
void IKController::updateTargetPoints() {
	for (auto& c : getJointChains()) {
		IKJoint& eff = m_IKJoints[c.joints[0]];
		if (IKTarget* nt = c.target.lock().get())
			nt->pos = eff.posGlobal;
	}
}

void IKController::update(float FPSScale) {
	m_ikArmature.solve(this);
}//update

void IKController::applyAnimation(Animation* pAnim, bool UpdateUBO) {
	if (pAnim) {
		SkeletalAnimationController::applyAnimation(pAnim,UpdateUBO);

		// no chains except maybe
		forwardKinematics(m_pRoot);
		updateTargetPoints(); //TODO(skade) target points need to be trackable to other animation (controllers?)
	} else {
		transformSkeleton(m_pRoot, Matrix4f::Identity());
	}
	if (UpdateUBO) {
		for (uint32_t i = 0; i < m_Joints.size(); ++i)
			m_UBO.skinningMatrix(i, m_Joints[i]->SkinningMatrix);
	}
}//applyAnimation

void IKController::forwardKinematics(SkeletalJoint* pJoint) {
	if (!pJoint)
		throw NullpointerExcept("pJoint");

	IKJoint& pJointIK = m_IKJoints[pJoint];

	if (pJoint == m_pRoot) {
		pJointIK.posGlobal = pJoint->LocalPosition;
		pJointIK.rotGlobal = pJoint->LocalRotation;
	}
	else {
		IKJoint& pJIKparent = m_IKJoints[m_Joints[pJoint->Parent]];
		pJointIK.posGlobal = (pJIKparent.rotGlobal * pJoint->LocalPosition) + pJIKparent.posGlobal;
		pJointIK.rotGlobal =  pJIKparent.rotGlobal * pJoint->LocalRotation;
	}

	pJointIK.rotGlobal.normalize();

	for (auto i : pJoint->Children)
		forwardKinematics(m_Joints[i]);
}//forwardKinematics

void IKController::retrieveSkinningMatrices(std::vector<Matrix4f>* pSkinningMats) {
	if (nullptr == pSkinningMats) throw NullpointerExcept("pSkinningMats");
	pSkinningMats->clear();
	pSkinningMats->reserve(m_Joints.size());
	for (auto i : m_Joints)
		pSkinningMats->push_back(i->SkinningMatrix);
}//retrieveSkinningMatrices

SkeletalAnimationController::SkeletalJoint* IKController::getBone(uint32_t idx) {
	return m_Joints[idx];
}

uint32_t IKController::boneCount() {
	return m_Joints.size();
}

}//CForge
