#include "IKController.h"

#include "JointLimits/SwingXZTwistYLimits.h"
#include "JointLimits/SwingXTwistYLimits.h"
#include "JointLimits/SwingZTwistYLimits.h"
#include "JointLimits/SwingXYTwistZLimits.h"

#include <crossforge/Graphics/Shader/SShaderManager.h>
#include <crossforge/Math/CForgeMath.h>
#include <crossforge/Core/SLogger.h>

#include <fstream>

#include <iostream>

#include <cmath>

using namespace Eigen;

namespace CForge {

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

	//TODO(skade) implement
	void IKController::init(T3DMesh<float>* pMesh) {
		
	}

	// pMesh has to hold skeletal definition
	void IKController::init(T3DMesh<float>* pMesh, std::string ConfigFilepath) {
		clear();

		if (nullptr == pMesh) throw NullpointerExcept("pMesh");
		if (pMesh->boneCount() == 0) throw CForgeExcept("Mesh has no bones!");

		// use first keyframe of first animation as rest pose of skeleton for now
		if (pMesh->skeletalAnimationCount() == 0)
			throw CForgeExcept("Mesh has no animation data!");
		
		std::ifstream f(ConfigFilepath);
		const nlohmann::json ConfigData = nlohmann::json::parse(f);
		
		initJointProperties(pMesh, ConfigData.at("JointLimits"));
		
		initSkeletonStructure(pMesh, ConfigData.at("SkeletonStructure"));
		
		forwardKinematics(m_pRoot); // initialize global positions and rotations of all joints
		
		initEndEffectorPoints();
		
		initTargetPoints();
		
		// initialize UBO
		m_UBO.init(m_Joints.size());

		for (uint32_t i = 0; i < m_Joints.size(); ++i) {
			m_UBO.skinningMatrix(i, m_Joints[i]->OffsetMatrix);
		}//for[joints]

		SShaderManager* pSMan = SShaderManager::instance();

		m_pShadowPassFSCode = pSMan->createShaderCode("Shader/ShadowPassShader.frag", m_GLSLVersionTag, 0, m_GLSLPrecisionTag);
		m_pShadowPassVSCode = pSMan->createShaderCode("Shader/ShadowPassShader.vert", m_GLSLVersionTag, ShaderCode::CONF_SKELETALANIMATION | ShaderCode::CONF_LIGHTING, m_GLSLPrecisionTag);

		ShaderCode::SkeletalAnimationConfig SkelConfig;
		SkelConfig.BoneCount = m_Joints.size();
		m_pShadowPassVSCode->config(&SkelConfig);

		std::vector<ShaderCode*> VSSources;
		std::vector<ShaderCode*> FSSources;
		VSSources.push_back(m_pShadowPassVSCode);
		FSSources.push_back(m_pShadowPassFSCode);

		m_pShadowPassShader = pSMan->buildShader(&VSSources, &FSSources, nullptr);

		pSMan->release();

	}//initialize

	void IKController::clear(void) {
		m_pRoot = nullptr;
		for (auto& i : m_IKJoints) {
			if (i.second) {
				if (i.second->pEndEffectorData) delete i.second->pEndEffectorData;
				if (i.second->pLimits) delete i.second->pLimits;
				delete i.second;
				i.second == nullptr;
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
			const T3DMesh<float>::Bone* pRef = pMesh->getBone(i);
			SkeletalJoint* pJoint = new SkeletalJoint();
			IKJoint* pIKJoint = new IKJoint();

			pJoint->ID = pRef->ID;
			pJoint->Name = pRef->Name;

			//TODOf(skade) insert rest pose values with solver: loc*paret * offset = identity (skinningmat)
			pJoint->LocalPosition = Vector3f::Zero();
			pJoint->LocalRotation = Quaternionf::Identity();
			pJoint->LocalScale = Vector3f(1.f,1.f,1.f);

			pJoint->LocalScale = pAnimation->Keyframes[i]->Scalings[0];
			pJoint->LocalPosition = pAnimation->Keyframes[i]->Positions[0];
			pJoint->LocalRotation = pAnimation->Keyframes[i]->Rotations[0].normalized();
			pJoint->LocalScale = pAnimation->Keyframes[i]->Scalings[0];
			
			pJoint->OffsetMatrix = pRef->InvBindPoseMatrix;
			pJoint->SkinningMatrix = Matrix4f::Identity(); // computed during applyAnimation()
			
			pIKJoint->GlobalPosition = Vector3f::Zero(); // computed after joint hierarchy has been constructed
			pIKJoint->GlobalRotation = Quaternionf::Identity(); // computed after joint hierarchy has been constructed

			// create user defined joint constraints
			const nlohmann::json& JointData = ConstraintData.at(pJoint->Name);
			std::string Type = JointData.at("Type").get<std::string>();

			if (Type == "Unconstrained")
				pIKJoint->pLimits = nullptr;

			if (Type == "Hinge") {
				std::string Hinge = JointData.at("HingeAxis").get<std::string>();
				std::string Forward = JointData.at("BoneForward").get<std::string>();

				if (Hinge == Forward) throw CForgeExcept("JointLimits for '" + pJoint->Name + "': HingeAxis and BoneForward cannot be the same joint axis!");

				Vector3f HingeAxis = Vector3f::Zero();
				Vector3f BoneForward = Vector3f::Zero();

				if (Hinge == "x") HingeAxis = Vector3f::UnitX();
				else if (Hinge == "y") HingeAxis = Vector3f::UnitY();
				else if (Hinge == "z") HingeAxis = Vector3f::UnitZ();
				else throw CForgeExcept("JointLimits for '" + pJoint->Name + "': HingeAxis must be 'x', 'y' or 'z'!");

				if (Forward == "x") BoneForward = Vector3f::UnitX();
				else if (Forward == "y") BoneForward = Vector3f::UnitY();
				else if (Forward == "z") BoneForward = Vector3f::UnitZ();
				else throw CForgeExcept("JointLimits for '" + pJoint->Name + "': BoneForward must be 'x', 'y' or 'z'!");

				float MinRad = CForgeMath::degToRad(JointData.at("MinAngleDegrees").get<float>());
				float MaxRad = CForgeMath::degToRad(JointData.at("MaxAngleDegrees").get<float>());

				HingeLimits* pNewLimits = new HingeLimits(pJoint->LocalRotation, HingeAxis, BoneForward, MinRad, MaxRad);
				pIKJoint->pLimits = pNewLimits;
			}
			
			//TODO(skade) rewrite and abstract hinge limits

			if (Type == "SwingXZTwistY") {
				float MinTwist = CForgeMath::degToRad(JointData.at("MinTwist").get<float>());
				float MaxTwist = CForgeMath::degToRad(JointData.at("MaxTwist").get<float>());
				float MinXSwing = CForgeMath::degToRad(JointData.at("MinXSwing").get<float>());
				float MaxXSwing = CForgeMath::degToRad(JointData.at("MaxXSwing").get<float>());
				float MinZSwing = CForgeMath::degToRad(JointData.at("MinZSwing").get<float>());
				float MaxZSwing = CForgeMath::degToRad(JointData.at("MaxZSwing").get<float>());
				
				SwingXZTwistYLimits* pNewLimits = new SwingXZTwistYLimits(pJoint->LocalRotation, MinXSwing, MaxXSwing, MinZSwing, MaxZSwing, MinTwist, MaxTwist);
				pIKJoint->pLimits = pNewLimits;
			}

			if (Type == "SwingXTwistY") {
				float MinTwist = CForgeMath::degToRad(JointData.at("MinTwist").get<float>());
				float MaxTwist = CForgeMath::degToRad(JointData.at("MaxTwist").get<float>());
				float MinSwing = CForgeMath::degToRad(JointData.at("MinSwing").get<float>());
				float MaxSwing = CForgeMath::degToRad(JointData.at("MaxSwing").get<float>());
				
				SwingXTwistYLimits* pNewLimits = new SwingXTwistYLimits(pJoint->LocalRotation, MinSwing, MaxSwing, MinTwist, MaxTwist);
				pIKJoint->pLimits = pNewLimits;
			}

			if (Type == "SwingZTwistY") {
				float MinTwist = CForgeMath::degToRad(JointData.at("MinTwist").get<float>());
				float MaxTwist = CForgeMath::degToRad(JointData.at("MaxTwist").get<float>());
				float MinSwing = CForgeMath::degToRad(JointData.at("MinSwing").get<float>());
				float MaxSwing = CForgeMath::degToRad(JointData.at("MaxSwing").get<float>());

				SwingZTwistYLimits* pNewLimits = new SwingZTwistYLimits(pJoint->LocalRotation, MinSwing, MaxSwing, MinTwist, MaxTwist);
				pIKJoint->pLimits = pNewLimits;
			}

			if (Type == "SwingXYTwistZ") {
				float MinTwist = CForgeMath::degToRad(JointData.at("MinTwist").get<float>());
				float MaxTwist = CForgeMath::degToRad(JointData.at("MaxTwist").get<float>());
				float MinXSwing = CForgeMath::degToRad(JointData.at("MinXSwing").get<float>());
				float MaxXSwing = CForgeMath::degToRad(JointData.at("MaxXSwing").get<float>());
				float MinYSwing = CForgeMath::degToRad(JointData.at("MinYSwing").get<float>());
				float MaxYSwing = CForgeMath::degToRad(JointData.at("MaxYSwing").get<float>());
				
				SwingXYTwistZLimits* pNewLimits = new SwingXYTwistZLimits(pJoint->LocalRotation, MinXSwing, MaxXSwing, MinYSwing, MaxYSwing, MinTwist, MaxTwist);
				pIKJoint->pLimits = pNewLimits;
			}

			//if (Type == "SwingXTwistZ") ...
			//if (Type == "SwingYTwistZ") ...
			//if (Type == "SwingYZTwistX") ...
			//if (Type == "SwingYTwistX") ...
			//if (Type == "SwingZTwistX") ...

			m_Joints.push_back(pJoint);
			m_IKJoints[pJoint] = pIKJoint;
		}//for[bones]
	}//initJointProperties

	void IKController::initSkeletonStructure(T3DMesh<float>* pMesh, const nlohmann::json& StructureData) {
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

	void IKController::initEndEffectorPoints() {
		transformSkeleton(m_pRoot,Eigen::Matrix4f::Identity());

		for (auto c : m_JointChains) {
			//IKSegment Seg = (IKSegment) i;
			SkeletalJoint* eff = c.second.joints.front();
			IKJoint* effIK = m_IKJoints[c.second.joints.front()];
			EndEffectorData* pEffData = new EndEffectorData();

			int32_t PointIdx = 0;
			pEffData->EEPosLocal.resize(3, c.second.joints.size());
			pEffData->EEPosGlobal.resize(3, c.second.joints.size());
			
			// initialize joint points from offset matrices
			for (int32_t j=0;j<c.second.joints.size();j++) {
				SkeletalJoint* pJC = c.second.joints[j];
				Eigen::Matrix4f om = pJC->SkinningMatrix * pJC->OffsetMatrix.inverse();
				pEffData->EEPosGlobal(0, PointIdx) = om.data()[12];
				pEffData->EEPosGlobal(1, PointIdx) = om.data()[13];
				pEffData->EEPosGlobal(2, PointIdx) = om.data()[14];
				PointIdx++;
			}

			for (int32_t i = 0; i < pEffData->EEPosLocal.cols(); ++i) {
				pEffData->EEPosLocal.col(i) = effIK->GlobalRotation.conjugate()
				       * (pEffData->EEPosGlobal.col(i) - effIK->GlobalPosition);
			}

			effIK->pEndEffectorData = pEffData;
		}
	}//initEndEffectorPoints

	void IKController::initTargetPoints(void) {
		for (auto& c : m_JointChains) {
			IKJoint* eff = m_IKJoints[c.second.joints.front()];
			eff->pEndEffectorData->TargetPosGlobal = eff->pEndEffectorData->EEPosGlobal;
		}
	}//initTargetPoints

	void IKController::update(float FPSScale) {
		forwardKinematics(m_pRoot);
		for (auto c : m_JointChains) {
			//ikJacobi(c.first);
			ikCCD(c.first); //TODO(skade) different methods
		}
	}//update

	void IKController::applyAnimation(Animation* pAnim, bool UpdateUBO) {
		if (pAnim) {
			SkeletalAnimationController::applyAnimation(pAnim,UpdateUBO);
		} else {
			transformSkeleton(m_pRoot, Matrix4f::Identity());
		}
		if (UpdateUBO) {
			for (uint32_t i = 0; i < m_Joints.size(); ++i)
				m_UBO.skinningMatrix(i, m_Joints[i]->SkinningMatrix);
		}
	}//applyAnimation


	//TODO(skade) OMR impl doesnt use multiple iterations
	void IKController::ikJacobi(const std::string segmentName) {
		std::vector<SkeletalJoint*>& Chain = m_JointChains.at(segmentName).joints;
		EndEffectorData* pEffData = m_IKJoints[Chain.front()]->pEndEffectorData;
		Matrix3Xf LastEndEffectorPoints;

		for (int32_t i = 0; i < m_MaxIterations; ++i) {
			LastEndEffectorPoints = pEffData->EEPosGlobal;

			for (int32_t k = 0; k < Chain.size(); ++k) {
				SkeletalJoint* pCurrent = Chain[k];
				IKJoint* pCurrentIK = m_IKJoints[pCurrent];

				// compute unconstrained global rotation that best aligns position and orientation of end effector with desired target values
				Quaternionf GlobalIncrement = computeUnconstrainedGlobalRotation(pCurrentIK, pEffData);
				Quaternionf NewGlobalRotation = GlobalIncrement * pCurrentIK->GlobalRotation;
				
				// transform new global rotation to new local rotation
				Quaternionf NewLocalRotation;
				if (pCurrent == m_pRoot)
					NewLocalRotation = NewGlobalRotation;
				else
					NewLocalRotation = m_IKJoints[m_Joints[pCurrent->Parent]]->GlobalRotation.conjugate() * NewGlobalRotation;

				NewLocalRotation.normalize();

				// constrain new local rotation if joint is not unconstrained
				if (pCurrentIK->pLimits != nullptr)
					NewLocalRotation = pCurrentIK->pLimits->constrain(NewLocalRotation);

				// apply new local rotation to joint
				pCurrent->LocalRotation = NewLocalRotation;

				// update kinematic chain
				forwardKinematics(pCurrent);
			}//for[each joint in chain]

			// check for termination -> condition: end-effector has reached the targets position and orientation
			Matrix3Xf EffectorTargetDiff = pEffData->TargetPosGlobal - pEffData->EEPosGlobal;
			float DistError = EffectorTargetDiff.cwiseProduct(EffectorTargetDiff).sum() / float(EffectorTargetDiff.cols());
			if (DistError < m_thresholdDist)
				return;

			Matrix3Xf EffectorPosDiff = pEffData->EEPosGlobal - LastEndEffectorPoints;
			float PosChangeError = EffectorPosDiff.cwiseProduct(EffectorPosDiff).sum() / float(EffectorPosDiff.cols());
			if (PosChangeError < m_thresholdPosChange)
				return;
		}//for[m_MaxIterations]
	}//ikJacobi

	void IKController::ikCCD(const std::string segmentName) {
		std::vector<SkeletalJoint*>& Chain = m_JointChains.at(segmentName).joints;
		EndEffectorData* pEffData = m_IKJoints[Chain.front()]->pEndEffectorData;
		Matrix3Xf LastEndEffectorPoints;

		for (int32_t i = 0; i < m_MaxIterations; ++i) {
			LastEndEffectorPoints = pEffData->EEPosGlobal;

			// check for termination -> condition: end-effector has reached the targets position and orientation
			Matrix3Xf EffectorTargetDiff = pEffData->TargetPosGlobal - pEffData->EEPosGlobal;
			float DistError = EffectorTargetDiff.cwiseProduct(EffectorTargetDiff).sum() / float(EffectorTargetDiff.cols());
			if (DistError < m_thresholdDist)
				return;

			//TODO(skade) implement Forward CCD
			// Backward CCD
			//for (int32_t k = Chain.size()-1; k >= 0; --k) {
			for (int32_t k = 0; k < Chain.size(); ++k) {
				// start at base joint
				SkeletalJoint* pCurrent = Chain[k];
				IKJoint* pCurrentIK = m_IKJoints[pCurrent];

				// calculate rotation axis
				// joint position to end effector
				Eigen::Vector3f jpToEE = pEffData->EEPosGlobal.col(0)-pCurrentIK->GlobalPosition;
				// joint position to target position
				Eigen::Vector3f jpToTar = pEffData->TargetPosGlobal.col(0)-pCurrentIK->GlobalPosition;

				// rotation angle
				float theta = std::acos(jpToEE.dot(jpToTar));
				if (std::abs(theta) < FLT_EPSILON)
					continue;

				// vector we rotate around
				Eigen::Vector3f rotVec = jpToEE.cross(jpToTar);
				rotVec.normalize();

				Quaternionf GlobalIncrement = Quaternionf(Eigen::AngleAxis(theta,rotVec));

				Quaternionf NewGlobalRotation = GlobalIncrement * pCurrentIK->GlobalRotation;
				
				// transform new global rotation to new local rotation
				Quaternionf NewLocalRotation;
				if (pCurrent == m_pRoot)
					NewLocalRotation = NewGlobalRotation;
				else
					NewLocalRotation = m_IKJoints[m_Joints[pCurrent->Parent]]->GlobalRotation.conjugate() * NewGlobalRotation;

				NewLocalRotation.normalize();

				// constrain new local rotation if joint is not unconstrained
				if (pCurrentIK->pLimits != nullptr)
					NewLocalRotation = pCurrentIK->pLimits->constrain(NewLocalRotation);

				// apply new local rotation to joint
				pCurrent->LocalRotation = NewLocalRotation;

				// update kinematic chain
				forwardKinematics(pCurrent);
			}//for[each joint in chain]

			Matrix3Xf EffectorPosDiff = pEffData->EEPosGlobal - LastEndEffectorPoints;
			float PosChangeError = EffectorPosDiff.cwiseProduct(EffectorPosDiff).sum() / float(EffectorPosDiff.cols());
			if (PosChangeError < m_thresholdPosChange)
				return;
		}//for[m_MaxIterations]
	}//ikCCD

	//TODO(skade) rewrite?
	void IKController::rotateGaze(void) {
		//Vector3f EPos = m_pHead->pEndEffectorData->EEPosGlobal.col(0);
		//Vector3f TPos = m_pHead->pEndEffectorData->TargetPosGlobal.col(0);
		//Vector3f CurrentDir = (EPos - m_pHead->GlobalPosition).normalized();
		//Vector3f TargetDir = (TPos - m_pHead->GlobalPosition).normalized();

		//if (std::abs(1.0f - CurrentDir.dot(TargetDir) > 1e-6f)) {
		//	// compute unconstrained global rotation to align both directional vectors in world space
		//	Quaternionf GlobalIncrement;
		//	GlobalIncrement.setFromTwoVectors(CurrentDir, TargetDir);
		//	Quaternionf NewGlobalRotation = GlobalIncrement * m_pHead->GlobalRotation;
		//	
		//	// transform new global rotation to new local rotation
		//	Quaternionf NewLocalRotation = (m_pHead == m_pRoot) ? NewGlobalRotation : m_pHead->pParent->GlobalRotation.conjugate() * NewGlobalRotation;
		//	NewLocalRotation.normalize();

		//	// constrain new local rotation if joint is not unconstrained
		//	if (m_pHead->pLimits != nullptr) NewLocalRotation = m_pHead->pLimits->constrain(NewLocalRotation);

		//	// apply new local rotation to joint 
		//	m_pHead->LocalRotation = NewLocalRotation;

		//	// compute new global joint rotation and apply to gaze direction
		//	forwardKinematics(m_pHead);
		//}
	}//rotateGaze

	Quaternionf IKController::computeUnconstrainedGlobalRotation(IKJoint* pJoint, IKController::EndEffectorData* pEffData) {

		//TODO(skade) this is not CCD but not Jacobi either

		//TODO: combine points of multiple end effectors and targets into 2 point clouds to compute CCD rotation for multiple end effectors?
		Matrix3Xf EndEffectorPoints = pEffData->EEPosGlobal.colwise() - pJoint->GlobalPosition; // current local joint position
		Matrix3Xf TargetPoints = pEffData->TargetPosGlobal.colwise() - pJoint->GlobalPosition;  // target local joint position

#if 0

		// compute matrix W
		Matrix3f W = TargetPoints * EndEffectorPoints.transpose();

		// compute singular value decomposition
		JacobiSVD<Matrix3f> SVD(W, ComputeFullU | ComputeFullV);
		Matrix3f U = SVD.matrixU();
		Matrix3f V = SVD.matrixV();

		// compute rotation
		Matrix3f R = U * V.transpose();
		Quaternionf GlobalRotation(R);
		GlobalRotation.normalize();

#else

		// compute rotation using quaternion characteristic polynomial from: "Closed-form solution of absolute orientation using unit quaternions." - Berthold K. P. Horn, 1987
		// https://web.stanford.edu/class/cs273/refs/Absolute-OPT.pdf

		//
		//			0	1	2
		//		0	Sxx	Sxy	Sxz
		// S =	1	Syx	Syy	Syz
		//		2	Szx	Szy	Szz
		//		
		Matrix3f S = EndEffectorPoints * TargetPoints.transpose();

		//
		// N = 
		//		Sxx + Syy + Szz		Syz - Szy			 Szx - Sxz			 Sxy - Syx
		//		Syz - Szy			Sxx - Syy - Szz		 Sxy + Syx			 Szx + Sxz
		//		Szx - Sxz			Sxy + Syx			-Sxx + Syy - Szz	 Syz + Szy
		//		Sxy - Syx			Szx + Sxz			 Syz + Szy			-Sxx - Syy + Szz
		//
		Matrix4f N = Matrix4f::Zero();

		N(0, 0) = S(0, 0) + S(1, 1) + S(2, 2);  //  Sxx + Syy + Szz
		N(0, 1) = S(1, 2) - S(2, 1);            //  Syz - Szy
		N(0, 2) = S(2, 0) - S(0, 2);            //  Szx - Sxz
		N(0, 3) = S(0, 1) - S(1, 0);            //  Sxy - Syx
		
		N(1, 0) = N(0, 1);                      //  Syz - Szy
		N(1, 1) = S(0, 0) - S(1, 1) - S(2, 2);  //  Sxx - Syy - Szz
		N(1, 2) = S(0, 1) + S(1, 0);            //  Sxy + Syx
		N(1, 3) = S(2, 0) + S(0, 2);            //  Szx + Sxz
		
		N(2, 0) = N(0, 2);                      //  Szx - Sxz
		N(2, 1) = N(1, 2);                      //  Sxy + Syx
		N(2, 2) = -S(0, 0) + S(1, 1) - S(2, 2); // -Sxx + Syy - Szz
		N(2, 3) = S(1, 2) + S(2, 1);            //  Syz + Szy
		
		N(3, 0) = N(0, 3);                      //  Sxy - Syx
		N(3, 1) = N(1, 3);                      //  Szx + Sxz
		N(3, 2) = N(2, 3);                      //  Syz + Szy
		N(3, 3) = -S(0, 0) - S(1, 1) + S(2, 2); // -Sxx - Syy + Szz
		
		Eigen::SelfAdjointEigenSolver<Matrix4f> Solver(4);
		Solver.compute(N);
		Vector4f BiggestEVec = Solver.eigenvectors().col(3); // last column of eigenvectors() matrix contains eigenvector of largest eigenvalue;
		                                                     // that's supposed to equal the desired rotation quaternion
		Quaternionf GlobalRotation = Quaternionf(BiggestEVec(0), BiggestEVec(1), BiggestEVec(2), BiggestEVec(3));

#endif

		return GlobalRotation;
	}//computeUnconstrainedGlobalRotation

	void IKController::forwardKinematics(SkeletalJoint* pJoint) {
		if (nullptr == pJoint) throw NullpointerExcept("pJoint");

		IKJoint* pJointIK = m_IKJoints[pJoint];

		if (pJoint == m_pRoot) {
			pJointIK->GlobalPosition = pJoint->LocalPosition;
			pJointIK->GlobalRotation = pJoint->LocalRotation;
		}
		else {
			IKJoint* pJIKparent = m_IKJoints[m_Joints[pJoint->Parent]];
			pJointIK->GlobalPosition = (pJIKparent->GlobalRotation * pJoint->LocalPosition) + pJIKparent->GlobalPosition;
			pJointIK->GlobalRotation = pJIKparent->GlobalRotation * pJoint->LocalRotation;
		}

		pJointIK->GlobalRotation.normalize();

		if (pJointIK->pEndEffectorData != nullptr) {
			auto& EEPosGlobal = pJointIK->pEndEffectorData->EEPosGlobal;
			auto& EEPosLocal = pJointIK->pEndEffectorData->EEPosLocal;

			for (int32_t i = 0; i < EEPosGlobal.cols(); ++i)
				EEPosGlobal.col(i) = (pJointIK->GlobalRotation * EEPosLocal.col(i)) + pJointIK->GlobalPosition;
		}

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

	//TODO(skade) used for IKStickFigureActor // remove?
	void IKController::updateSkeletonValues(std::vector<SkeletalAnimationController::SkeletalJoint*>* pSkeleton) {
		if (nullptr == pSkeleton) throw NullpointerExcept("pSkeleton");

		for (auto i : (*pSkeleton)) {
			i->OffsetMatrix = m_Joints[i->ID]->OffsetMatrix;
			i->LocalPosition = m_Joints[i->ID]->LocalPosition;
			i->LocalRotation = m_Joints[i->ID]->LocalRotation;
			i->LocalScale = m_Joints[i->ID]->LocalScale;
			i->SkinningMatrix = m_Joints[i->ID]->SkinningMatrix;
		}
	}//updateSkeletonValues

	std::vector<IKController::SkeletalEndEffector> IKController::retrieveEndEffectors(void) {
		std::vector<SkeletalEndEffector> Rval;

		for (const auto& c : m_JointChains) {
			SkeletalEndEffector pEff;
			IKJoint* eff = m_IKJoints.at(c.second.joints.front());
			pEff.joint = c.second.joints.front();
			pEff.jointIK = eff;
			pEff.segmentName = c.first;

			Rval.push_back(pEff);
		}

		return Rval;
	}//retrieveEndEffectors

	void IKController::translateTarget(std::string segmentName, Vector3f Translation) {
		EndEffectorData* pEffData = pEffData = m_IKJoints[m_JointChains.at(segmentName).joints.front()]->pEndEffectorData;
		pEffData->TargetPosGlobal.col(0) = Translation;
	}//endEffectorTarget

	Eigen::Matrix3Xf IKController::getTargetPoints(std::string segmentName) {
		EndEffectorData* pEffData = pEffData = m_IKJoints[m_JointChains.at(segmentName).joints.front()]->pEndEffectorData;
		return pEffData->TargetPosGlobal;
	}

	//TODO(skade) use?
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

	void IKController::updateEndEffectorPoints() {
		for (auto c : m_JointChains) {
			//IKSegment Seg = (IKSegment) i;
			SkeletalJoint* eff = c.second.joints.front();
			IKJoint* effIK = m_IKJoints[c.second.joints.front()];
			EndEffectorData* pEffData = effIK->pEndEffectorData;

			int32_t PointIdx = 0;
			
			// initialize joint points from offset matrices
			for (int32_t j=0;j<c.second.joints.size();j++) {
				SkeletalJoint* pJC = c.second.joints[j];
				Eigen::Matrix4f om = pJC->SkinningMatrix * pJC->OffsetMatrix.inverse();
				pEffData->EEPosGlobal(0, PointIdx) = om.data()[12];
				pEffData->EEPosGlobal(1, PointIdx) = om.data()[13];
				pEffData->EEPosGlobal(2, PointIdx) = om.data()[14];
				//pJC = m_Joints[pJC->Parent];
				PointIdx++;
			}

			for (int32_t i = 0; i < pEffData->EEPosLocal.cols(); ++i) {
				pEffData->EEPosLocal.col(i) = effIK->GlobalRotation.conjugate()
				                                          * (pEffData->EEPosGlobal.col(i)
				                                          - effIK->GlobalPosition);
			}
		}
	}

}//CForge
