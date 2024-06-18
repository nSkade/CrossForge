#include "IKController.hpp"

#include <crossforge/Graphics/Shader/SShaderManager.h>
#include <crossforge/Math/CForgeMath.h>
#include <crossforge/Core/SLogger.h>

#include <crossforge/Graphics/RenderDevice.h> //TODO(skade) for JointVis

#include "CMN/EigenFWD.hpp"

#include <fstream>
#include <iostream>
#include <cmath>

namespace CForge {
using namespace Eigen;

//TODOf(skade) CForgeObject classname wrong
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

	if (nullptr == pMesh)
		throw NullpointerExcept("pMesh");
	if (pMesh->boneCount() == 0)
		throw CForgeExcept("Mesh has no bones!");

	// use first keyframe of first animation as rest pose of skeleton for now
	if (pMesh->skeletalAnimationCount() == 0)
		throw CForgeExcept("Mesh has no animation data!");
	
	T3DMesh<float>::SkeletalAnimation* pAnimation = pMesh->getSkeletalAnimation(0); // used as initial pose of skeleton

	//TODO(skade) add option
	//for (uint32_t i = 0; i < pMesh->boneCount(); ++i) {
	//	const T3DMesh<float>::Bone* pRef = pMesh->getBone(i);
	//	SkeletalJoint* pJoint = new SkeletalJoint();

	//	pJoint->ID = pRef->ID;
	//	pJoint->Name = pRef->Name;

	//	Matrix4f iom = pRef->InvBindPoseMatrix.inverse();

	//	//TODOf(skade) insert rest pose values with solver: loc*paret * offset = identity (skinningmat)

	//	// iom = p * r * s
	//	pJoint->LocalPosition = iom.block<3,1>(0,3);
	//	pJoint->LocalRotation = Quaternionf(iom.block<3,3>(0,0));
	//	pJoint->LocalScale = Vector3f(1.,1.,1.);
	//	//pJoint->LocalScale = Vector3f(iom.data()[0],iom.data()[5],iom.data()[10]);
	//	//pJoint->LocalScale = iom.block<3,3>(0,0).inverse()*pJoint->LocalScale;

	//	//pJoint->LocalScale = pAnimation->Keyframes[i]->Scalings[0];
	//	//pJoint->LocalPosition = pAnimation->Keyframes[i]->Positions[0];
	//	//pJoint->LocalRotation = pAnimation->Keyframes[i]->Rotations[0].normalized();
	//	
	//	pJoint->OffsetMatrix = pRef->InvBindPoseMatrix;
	//	pJoint->SkinningMatrix = Matrix4f::Identity(); // computed during applyAnimation()
	//	m_Joints.push_back(pJoint);
	//}

	// compute local joint parameters for restpose
	for (uint32_t i = 0; i < pMesh->boneCount(); ++i)
		m_Joints.emplace_back(new SkeletalJoint());

	std::function<void(const T3DMesh<float>::Bone* pBone, Matrix4f offP)> initJoint;
	initJoint = [&](const T3DMesh<float>::Bone* pBone, Matrix4f offP) {
		Matrix4f iom = pBone->InvBindPoseMatrix.inverse();
		Matrix4f t = offP.inverse() * iom;

		SkeletalJoint* pJoint = m_Joints[pBone->ID];
		pJoint->ID = pBone->ID;
		pJoint->Name = pBone->Name;
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

		pJoint->OffsetMatrix = pBone->InvBindPoseMatrix;
		pJoint->SkinningMatrix = Matrix4f::Identity(); // computed during applyAnimation()

		for (uint32_t i = 0; i < pBone->Children.size(); ++i)
			initJoint(pBone->Children[i],iom);
	};
	initJoint(pMesh->rootBone(),Matrix4f::Identity());

	// copy structure
	for (uint32_t i = 0; i < pMesh->boneCount(); ++i) {
		const T3DMesh<float>::Bone* pRef = pMesh->getBone(i);
		SkeletalJoint* pJoint = m_Joints[i];

		pJoint->Parent = (pRef->pParent != nullptr) ? pRef->pParent->ID : -1;

		for (uint32_t k = 0; k < pRef->Children.size(); ++k)
			pJoint->Children.push_back(pRef->Children[k]->ID);
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

	for (uint32_t i=0;i<m_Joints.size();++i) {
		m_jointPickables.emplace_back(std::make_shared<JointPickable>(&m_jointPickableMesh,m_Joints[i]));
		//m_jointPickables.back()->BVtrans
	}
}

// pMesh has to hold skeletal definition
void IKController::init(T3DMesh<float>* pMesh, std::string ConfigFilepath) {
	init(pMesh);
	
	std::ifstream f(ConfigFilepath);
	const nlohmann::json ConfigData = nlohmann::json::parse(f);

	initJointProperties(pMesh, ConfigData.at("JointLimits"));
	initSkeletonStructure(pMesh, ConfigData.at("SkeletonStructure"));
	forwardKinematics(m_pRoot); // initialize global positions and rotations of all joints
	initTargetPoints();
}//initialize

void IKController::clear(void) {
	m_pRoot = nullptr;
	for (auto& i : m_IKJoints) {
		if (i.second) {
			//if (i.second->pEndEffectorData)
			//	delete i.second->pEndEffectorData;
			//TODO(skade)
			//if (i.second->pLimits) delete i.second->pLimits;
			delete i.second;
			i.second = nullptr;
		}
	}

	m_Joints.clear();
	m_IKJoints.clear();
	m_JointChains.clear();
	
	m_UBO.clear();

	// instances get deleted by the Shader Manager
	m_pShadowPassShader = nullptr;
	m_pShadowPassFSCode = nullptr;
	m_pShadowPassVSCode = nullptr;

}//clear

void IKController::initJointProperties(T3DMesh<float>* pMesh, const nlohmann::json& ConstraintData) {
	T3DMesh<float>::SkeletalAnimation* pAnimation = pMesh->getSkeletalAnimation(0); // used as initial pose of skeleton

	for (uint32_t i = 0; i < pMesh->boneCount(); ++i) {
		SkeletalJoint* pJoint = m_Joints[i];
		IKJoint* pIKJoint = new IKJoint();

		pIKJoint->posGlobal = Vector3f::Zero(); // computed after joint hierarchy has been constructed
		pIKJoint->rotGlobal = Quaternionf::Identity(); // computed after joint hierarchy has been constructed
		m_IKJoints[pJoint] = pIKJoint;

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
		
		////TODO(skade) rewrite and abstract hinge limits

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
}//initJointProperties

void IKController::initSkeletonStructure(T3DMesh<float>* pMesh, const nlohmann::json& StructureData) {
	// create user defined skeleton segments 
	for (auto it : StructureData.items()) {
		if(it.value().contains("Root") && it.value().contains("EndEffector"))
			buildKinematicChain(it.key(),it.value().at("Root").get<std::string>(),it.value().at("EndEffector").get<std::string>());
	}
}//initSkeletonStructure

void IKController::buildKinematicChain(std::string name, std::string rootName, std::string endEffectorName) {
	m_JointChains.try_emplace(name,IKSegment());
	m_JointChains.at(name).name = name;
	std::vector<SkeletalJoint*>& Chain = m_JointChains.at(name).joints;

	// fill chain in order end-effector -> chain root
	SkeletalJoint* pCurrent = nullptr;
	for (SkeletalJoint* i : m_Joints) {
		if (i->Name == endEffectorName) {
			pCurrent = i;
			break;
		}
	}
	if (pCurrent == nullptr)
		throw NullpointerExcept("pCurrent"); // couldn't find the end-effector joint

	SkeletalJoint* pEnd = nullptr;
	for (SkeletalJoint* i : m_Joints) {
		if (i->Name == rootName) {
			pEnd = i;
			break;
		}
	}
	if (pEnd == nullptr)
		throw NullpointerExcept("pEnd"); // couldn't find the root joint
			
	while (pCurrent->ID != pEnd->ID) {
		if (pCurrent == m_pRoot && pEnd != m_pRoot)
			throw CForgeExcept("Reached root joint of skeleton without finding root joint of chain!");

		Chain.push_back(pCurrent);
		pCurrent = m_Joints[pCurrent->Parent];
	}

	if (Chain.size() < 2)
		throw CForgeExcept("Initialization of chain failed, Chain size < 2");
}//buildKinematicChain

//void IKController::initEndEffectorPoints() {
//	transformSkeleton(m_pRoot,Eigen::Matrix4f::Identity());
//
//	for (auto c : m_JointChains) {
//		IKSegment Seg = (IKSegment) i;
//		SkeletalJoint* eff = c.second.joints[0];
//		IKJoint* effIK = m_IKJoints[c.second.joints[0]];
//		EndEffectorData* pEffData = new EndEffectorData();
//
//		int32_t PointIdx = 0;
//		pEffData->EEPosLocal.resize(3, c.second.joints.size());
//		pEffData->EEPosGlobal.resize(3, c.second.joints.size());
//		
//		// initialize joint points from offset matrices
//		for (int32_t j=0;j<c.second.joints.size();j++) {
//			SkeletalJoint* pJC = c.second.joints[j];
//			Eigen::Matrix4f om = pJC->SkinningMatrix * pJC->OffsetMatrix.inverse();
//			pEffData->EEPosGlobal(0, PointIdx) = om.data()[12];
//			pEffData->EEPosGlobal(1, PointIdx) = om.data()[13];
//			pEffData->EEPosGlobal(2, PointIdx) = om.data()[14];
//			PointIdx++;
//		}
//
//		for (int32_t i = 0; i < pEffData->EEPosLocal.cols(); ++i) {
//			pEffData->EEPosLocal.col(i) = effIK->rotGlobal.conjugate()
//			       * (pEffData->EEPosGlobal.col(i) - effIK->posGlobal);
//		}
//
//		TODO(skade)
//		effIK->pEndEffectorData = pEffData;
//	}
//}//initEndEffectorPoints

void IKController::initTargetPoints() {
	clearTargetPoints();
	for (auto& c : m_JointChains) {
		IKJoint* eff = m_IKJoints[c.second.joints[0]];
		//eff->pEndEffectorData->TargetPosGlobal = eff->pEndEffectorData->EEPosGlobal;
		IKTarget* nt = new IKTarget();

		nt->pos = eff->posGlobal;
		BoundingVolume bv;
		Vector3f d =Vector3f(0.05f, 0.05f, 0.05f);
		Box b; b.init(-d*.5,d*.5);
		bv.init(b);
		nt->bv = bv;
		
		c.second.target = nt;
		m_targets.emplace_back(std::shared_ptr<IKTarget>(nt));
	}
}//initTargetPoints

void IKController::clearTargetPoints() {
	for (auto& c : m_JointChains)
		c.second.target = nullptr;
	m_targets.clear();
}

void IKController::updateTargetPoints() {
	for (auto& c : m_JointChains) {
		IKJoint* eff = m_IKJoints[c.second.joints[0]];
		//eff->pEndEffectorData->TargetPosGlobal = eff->pEndEffectorData->EEPosGlobal;
		IKTarget* nt = c.second.target;
		nt->pos = eff->posGlobal;
	}
}

void IKController::update(float FPSScale) {
	while (m_iksFABRIK.size() < m_JointChains.size())
		m_iksFABRIK.push_back(IKSolverFABRIK());

	int i=0;
	for (auto c : m_JointChains) {
		forwardKinematics(m_pRoot);
		//if (c.second.name!="RightLeg")
		//	continue;
		
		//TODO(skade) define solver for each chain
		switch (testIKslvSelect)
		{
		case CForge::IKController::IKSS_CCD_F:
			m_iksCCD.solve<IKSolverCCD::FORWARD>(c.first,this);
			break;
		case CForge::IKController::IKSS_CCD_B:
			m_iksCCD.solve<IKSolverCCD::BACKWARD>(c.first,this);
			break;
		case CForge::IKController::IKSS_CCD_FABRIK:
			m_iksFABRIK[i].solve(c.first,this);
			break;
		default:
			break;
		}

		++i;
	}
}//update

void IKController::applyAnimation(Animation* pAnim, bool UpdateUBO) {
	if (pAnim) {
		SkeletalAnimationController::applyAnimation(pAnim,UpdateUBO);

		//TODO(skade) no chains except
		forwardKinematics(m_pRoot);
		updateTargetPoints();
	} else {
		transformSkeleton(m_pRoot, Matrix4f::Identity());
	}
	if (UpdateUBO) {
		for (uint32_t i = 0; i < m_Joints.size(); ++i)
			m_UBO.skinningMatrix(i, m_Joints[i]->SkinningMatrix);
	}
}//applyAnimation

//TODO(skade) rewrite?
//void IKController::rotateGaze(void) {
//	Vector3f EPos = m_pHead->pEndEffectorData->EEPosGlobal.col(0);
//	Vector3f TPos = m_pHead->pEndEffectorData->TargetPosGlobal.col(0);
//	Vector3f CurrentDir = (EPos - m_pHead->posGlobal).normalized();
//	Vector3f TargetDir = (TPos - m_pHead->posGlobal).normalized();
//
//	if (std::abs(1.0f - CurrentDir.dot(TargetDir) > 1e-6f)) {
//		// compute unconstrained global rotation to align both directional vectors in world space
//		Quaternionf GlobalIncrement;
//		GlobalIncrement.setFromTwoVectors(CurrentDir, TargetDir);
//		Quaternionf NewGlobalRotation = GlobalIncrement * m_pHead->rotGlobal;
//		
//		// transform new global rotation to new local rotation
//		Quaternionf NewLocalRotation = (m_pHead == m_pRoot) ? NewGlobalRotation : m_pHead->pParent->rotGlobal.conjugate() * NewGlobalRotation;
//		NewLocalRotation.normalize();
//
//		// constrain new local rotation if joint is not unconstrained
//		if (m_pHead->pLimits != nullptr) NewLocalRotation = m_pHead->pLimits->constrain(NewLocalRotation);
//
//		// apply new local rotation to joint 
//		m_pHead->LocalRotation = NewLocalRotation;
//
//		// compute new global joint rotation and apply to gaze direction
//		forwardKinematics(m_pHead);
//	}
//}//rotateGaze

void IKController::forwardKinematics(SkeletalJoint* pJoint) {
	if (!pJoint)	
		throw NullpointerExcept("pJoint");

	IKJoint* pJointIK = m_IKJoints[pJoint];
	if (!pJointIK) //TODO(skade)
		return;

	if (pJoint == m_pRoot) {
		pJointIK->posGlobal = pJoint->LocalPosition;
		pJointIK->rotGlobal = pJoint->LocalRotation;
	}
	else {
		IKJoint* pJIKparent = m_IKJoints[m_Joints[pJoint->Parent]];
		pJointIK->posGlobal = (pJIKparent->rotGlobal * pJoint->LocalPosition) + pJIKparent->posGlobal;
		pJointIK->rotGlobal = pJIKparent->rotGlobal * pJoint->LocalRotation;
	}

	pJointIK->rotGlobal.normalize();

	for (auto i : pJoint->Children)
		forwardKinematics(m_Joints[i]);
}//forwardKinematics

void IKController::retrieveSkinningMatrices(std::vector<Matrix4f>* pSkinningMats) {
	if (nullptr == pSkinningMats) throw NullpointerExcept("pSkinningMats");
	pSkinningMats->clear();
	for (auto i : m_Joints)
		pSkinningMats->push_back(i->SkinningMatrix);
}//retrieveSkinningMatrices

SkeletalAnimationController::SkeletalJoint* IKController::getBone(uint32_t idx) {
	return m_Joints[idx];
}

uint32_t IKController::boneCount() {
	return m_Joints.size();
}

void IKController::renderJointPickables(RenderDevice* pRenderDev) {
#if 0 // from hierarchy
	std::function<void(SkeletalAnimationController::SkeletalJoint* pJoint, Matrix4f parMat)> renderJoint;
	renderJoint = [&](SkeletalAnimationController::SkeletalJoint* pJoint, Matrix4f fromPar) {
		//TODO(skade) appply sg node transform
		const Matrix4f R = CForgeMath::rotationMatrix(pJoint->LocalRotation);
		const Matrix4f T = CForgeMath::translationMatrix(pJoint->LocalPosition);
		const Matrix4f S = CForgeMath::scaleMatrix(pJoint->LocalScale);
		const Matrix4f JointTransform = T * R * S;
		Matrix4f LocalTransform = fromPar * JointTransform;

		Vector3f BoneVec; // vector to next bone
		if (pJoint->Children.size() > 0)
			BoneVec = m_IKController->getBone(pJoint->Children[0])->LocalPosition;
		else
			BoneVec = pJoint->LocalPosition;
		float Length = BoneVec.norm(); // length to next bone

		Quaternionf LR = EigenFWD::FromTwoVectors(Vector3f::UnitX(), BoneVec.normalized()); // obj Joint points to +x axis
		Matrix4f t = LocalTransform * CForgeMath::rotationMatrix(LR) * CForgeMath::scaleMatrix(Vector3f(Length,Length,Length));
		
		m_RenderDev.modelUBO()->modelMatrix(t);
		m_JointVisActor.render(&m_RenderDev,Eigen::Quaternionf::Identity(),Eigen::Vector3f(),Eigen::Vector3f(1.f,1.f,1.f));
		for (uint32_t i = 0; i < pJoint->Children.size(); ++i)
			renderJoint(m_IKController->getBone(pJoint->Children[i]),LocalTransform);
	};
	renderJoint(m_IKController->getRoot(),Matrix4f::Identity());
#elif 1 // from skinning matrices, should be faster
	for (uint32_t i = 0; i < boneCount(); i++) {
		SkeletalAnimationController::SkeletalJoint* pJoint = m_Joints[i];
		//TODO(skade) appply sg node transform
		const Matrix4f R = CForgeMath::rotationMatrix(pJoint->LocalRotation);
		const Matrix4f T = CForgeMath::translationMatrix(pJoint->LocalPosition);
		const Matrix4f S = CForgeMath::scaleMatrix(pJoint->LocalScale);
		const Matrix4f JointTransform = T * R * S;

		Matrix4f fromPar = Matrix4f::Identity();
		if (pJoint->Parent != -1)
			fromPar = m_Joints[pJoint->Parent]->SkinningMatrix
			          * m_Joints[pJoint->Parent]->OffsetMatrix.inverse();
		Matrix4f LocalTransform = fromPar * JointTransform;

		Vector3f BoneVec; // vector to next bone
		if (pJoint->Children.size() > 0)
			BoneVec = m_Joints[pJoint->Children[0]]->LocalPosition;
		else
			BoneVec = pJoint->LocalPosition;
		float Length = BoneVec.norm(); // length to next bone
		//
		Quaternionf LR = EigenFWD::FromTwoVectors(Vector3f::UnitX(), BoneVec.normalized()); // obj Joint points to +x axis
		Matrix4f t = LocalTransform * CForgeMath::rotationMatrix(LR) * CForgeMath::scaleMatrix(Vector3f(Length,Length,Length));
		
		pRenderDev->modelUBO()->modelMatrix(t);
		m_jointPickableMesh.actor.render(pRenderDev,
		                                  Eigen::Quaternionf::Identity(),Eigen::Vector3f(),Eigen::Vector3f(1.f,1.f,1.f));
	}
#endif
}

void IKController::updateBones(Animation* pAnim) {
	for (uint32_t i = 0; i < m_Joints.size(); ++i) {
		T3DMesh<float>::SkeletalAnimation* pAnimData = m_SkeletalAnimations[pAnim->AnimationID];
		SkeletalJoint* pJoint = m_Joints[i];
		IKJoint* pIKJoint = m_IKJoints[pJoint];

		//TODO(skade) insert rest pose values with solver: loc*paret * offset = identity (skinningmat)
		pJoint->LocalPosition = Vector3f::Zero();
		pJoint->LocalRotation = Quaternionf::Identity();
		pJoint->LocalScale = Vector3f(1.f,1.f,1.f);

		uint32_t animIdx = 0;

		for (uint32_t k = 0; k < pAnimData->Keyframes[i]->Timestamps.size() - 1; ++k) {
			float Time = pAnimData->Keyframes[i]->Timestamps[k];
			if (Time <= pAnim->t) {
				animIdx = k;
				break;
			}
		}
		
		pJoint->LocalScale = pAnimData->Keyframes[i]->Scalings[animIdx];
		pJoint->LocalPosition = pAnimData->Keyframes[i]->Positions[animIdx];
		pJoint->LocalRotation = pAnimData->Keyframes[i]->Rotations[animIdx].normalized();
		pJoint->LocalScale = pAnimData->Keyframes[i]->Scalings[animIdx];
	}
}

}//CForge
