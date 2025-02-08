#include "MRlimb.hpp"

#include "Prototypes/MotionRetarget/CMN/MRMutil.hpp"

#include <iostream>

namespace CForge {
using namespace Eigen;

void MRlimb::initialize(std::shared_ptr<CharEntity> source, std::shared_ptr<CharEntity> target, std::vector<int> corr) {
	reset();

	m_sCE = source;
	m_tCE = target;
	m_ikcorr = corr;
	m_active = true;

	// assign limb scaling values
	auto& sCtrl = source->controller;
	auto& tCtrl = target->controller;

	for (int it = 0; it < m_ikcorr.size();++it) {
		int is = m_ikcorr[it];
		IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];
		IKChain& ct = tCtrl->m_ikArmature.m_jointChains[it];
		
		m_scale_limbs.push_back(1.);
	}

	for (int it = 0; it < tCtrl->m_ikArmature.m_jointChains.size(); ++it) {
		IKChain& ct = tCtrl->m_ikArmature.m_jointChains[it];
		float tarLen = 0.;
		for (int i = 0; i < ct.joints.size(); ++i)
			tarLen += ct.joints[i]->LocalPosition.norm();
		m_tar_limbLen.push_back(tarLen);
	}
	for (int is = 0; is < sCtrl->m_ikArmature.m_jointChains.size(); ++is) {
		IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];
		float srcLen = 0.;
		for (int i = 0; i < cs.joints.size(); ++i)
			srcLen += cs.joints[i]->LocalPosition.norm();
		m_src_limbLen.push_back(srcLen);
	}

	m_src_rootPos = sCtrl->getRoot()->LocalPosition;
	m_tar_rootPos = tCtrl->getRoot()->LocalPosition;

	// reinitialize targets
	source->autoCreateTargets();
	target->autoCreateTargets();

	// create copyies of source iktargets for target char
	m_targets.clear();
	for (int it = 0; it < m_ikcorr.size();++it) {
		int is = m_ikcorr[it];
		IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];
		IKChain& ct = tCtrl->m_ikArmature.m_jointChains[it];

		//TODOff(skade) auto create targets if they dont exist
		std::shared_ptr<IKTarget> t =
			std::make_shared<IKTarget>(*sCtrl->m_ikArmature.m_jointChains[is].target.lock().get());

		Vector3f pos; Quaternionf rot; Vector3f scale;
		target->sgn.buildTansformation(&pos,&rot,&scale);
		t->m_sgnT = CForgeMath::translationMatrix(pos) * CForgeMath::rotationMatrix(rot) * CForgeMath::scaleMatrix(scale);

		m_targets.emplace_back(t);
		ct.target = t;
	}
};
std::vector<int> MRlimb::jointIndexingFunc(int tarIdx, IKChain& cs, IKChain& ct) {
	auto source = m_sCE.lock();
	auto target = m_tCE.lock();
	if (!source || !target) {
		m_active = false;
		return std::vector<int>();
	}
	auto& sCtrl = source->controller;
	auto& tCtrl = target->controller;
#if 0
	// by distance
	float closestDist = std::numeric_limits<float>::max();
	int closestIdx = -1;
	SkeletalAnimationController::SkeletalJoint* jt = ct.joints[tarIdx];
	for (int i=0;i<cs.joints.size();++i) {
		auto* js = cs.joints[i];
		float dist = (tCtrl->m_IKJoints[jt].posGlobal-sCtrl->m_IKJoints[js].posGlobal).norm(); //TODO(skade) from current pose, need rest pose global pos instead
		if (dist < closestDist) {
			closestDist = dist;
			closestIdx = i;
		}
	}
	return closestIdx;
#endif
#if 0
	// by distribution
	return std::floor((float(tarIdx)/ct.joints.size())*cs.joints.size());
#endif
#if 0
	// by index
	if (tarIdx < cs.joints.size())
		return {tarIdx};
#endif
#if 1
	//TODOff(skade) cache these results in init func
	if (cs.joints.size() == ct.joints.size())
		return {tarIdx}; // identical matching possible
	else {
		//// the ideal method
		//// distribute by joint length
		//float totalLenS = 0.f;
		//for (auto j : cs.joints)
		//	totalLenS += j->LocalPosition.norm();
		//float totalLenT = 0.f;
		//for (auto j : ct.joints)
		//	totalLenT += j->LocalPosition.norm();

		//float currSrcLen=0.f;
		//float currTarLen=0.f;
		//for (int i=ct.joints.size()-1;i>=0;--i) {
		//	//
		//	currTarLen += ct.joints[i]->LocalPosition.norm();
		//	//if (i==tarIdx)

		//}

		////for (int i=cs.joints.size()-1;i>=0;--i) {
		////	//
		////	currSrcLen += cs.joints[i]->LocalPosition.norm();
		////}

		//if (cs.joints.size() > ct.joints.size()) {
		//	// more source joints, need to merge multiple source joints
		//	// ideally combine into longest bone as it should have most influence
		//} else {
		// //(cs.joints.size() < ct.joints.size())
		//	// more target joints, need to omit some indexing
		//	// ideally omit shortest bone which should have least significant influence
		//}

//////////
	// Compute cumulative lengths from end effector to root
		std::vector<float> srcCumulative, tarCumulative;
		float totalLenS = 0.f, totalLenT = 0.f;

		for (int i = 0; i < cs.joints.size(); ++i) {
			totalLenS += cs.joints[i]->LocalPosition.norm();
			srcCumulative.push_back(totalLenS);
		}

		for (int i = 0; i < ct.joints.size(); ++i) {
			totalLenT += ct.joints[i]->LocalPosition.norm();
			tarCumulative.push_back(totalLenT);
		}

		// Normalize cumulative lengths
		std::vector<float> srcNorm, tarNorm;
		for (float len : srcCumulative) {
			srcNorm.push_back((totalLenS > 0) ? (len / totalLenS) : 0.f);
		}
		for (float len : tarCumulative) {
			tarNorm.push_back((totalLenT > 0) ? (len / totalLenT) : 0.f);
		}

		if (cs.joints.size() > ct.joints.size()) {
			// Merge source joints into target
			float start = (tarIdx == 0) ? -std::numeric_limits<float>::infinity() : tarNorm[tarIdx - 1];
			float end = tarNorm[tarIdx];
			std::vector<int> indices;
			for (int j = 0; j < srcNorm.size(); ++j) {
				if (srcNorm[j] > start && srcNorm[j] <= end) {
					indices.push_back(j);
				}
			}
			return indices;
		} else {
			// Source is shorter: find closest source joint or omit
			float tarValue = tarNorm[tarIdx];
			int closestJ = -1;
			float minDist = std::numeric_limits<float>::max();
			for (int j = 0; j < srcNorm.size(); ++j) {
				float dist = std::abs(srcNorm[j] - tarValue);
				if (dist < minDist) {
					minDist = dist;
					closestJ = j;
				}
			}

			if (closestJ == -1) {
				return {};
			}

			// Calculate threshold as half the average interval between source joints
			float avgInterval = (srcNorm.size() > 1) ? (1.0f / (srcNorm.size() - 1)) : 1.0f;
			float threshold = 0.5f * avgInterval;

			if (minDist <= threshold) {
				return {closestJ};
			} else {
				return {};
			}
		}
	}

#endif
	return std::vector<int>();
}
void MRlimb::update() {
	auto source = m_sCE.lock();
	auto target = m_tCE.lock();
	if (!source || !target)
		m_active = false;
	if (!m_active) {
		m_targets.clear();
		return;
	}

	auto& sCtrl = source->controller;
	auto& tCtrl = target->controller;

	for (int it = 0; it < m_ikcorr.size();++it) {
		int is = m_ikcorr[it];
		IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];
		IKChain& ct = tCtrl->m_ikArmature.m_jointChains[it];
		//ct.target = cs.target; // old way, assign other char entity target 
		
		// update target position
		{ // rescale limb target positions
			float scale = m_tar_limbLen[it]/m_src_limbLen[is];
			scale = CForgeMath::lerp(1.f,scale,m_scale_limbs[it]);

			//TODO(skade) srp needs offset of parent chain transform
			// root pos of chain
			Vector3f srp = sCtrl->m_IKJoints[cs.joints.back()].posGlobal;
			Vector3f sdir = cs.target.lock()->pos - srp;

			//TODO(skade) append limb dir to last frame not ideal
			ct.target.lock()->pos = tCtrl->m_IKJoints[ct.joints.back()].posGlobal + sdir*scale;
		}

//TODO(skade) look for reusable code
////		// get parent rot of root joint for reference
////		auto csRoot = cs.joints.back();
////		Quaternionf rootGlobRot = Quaternionf::Identity();
////		rootGlobRot = sCtrl->m_IKJoints[csRoot].rotGlobal;
////		//TODO(skade) parent of root?
////		//if (csRoot->Parent != -1)
////		//	rootGlobRot = sCtrl->m_IKJoints[sCtrl->getBone(csRoot->Parent)].rotGlobal;
////		Quaternionf parRot = rootGlobRot;
////		
////		// imitate joint angles, start from root of chain
//		for (int i=ct.joints.size()-1; i >= 0; --i) {
//			// joint, apply angle to
//			SkeletalAnimationController::SkeletalJoint* jt = ct.joints[i];
//
//			// find closest source joint to imitate angle from
//			int matchIdx = jointIndexingFunc(i,cs,ct);
//			if (matchIdx != -1) {
//				 // source joint
//				SkeletalAnimationController::SkeletalJoint* js = cs.joints[matchIdx];
//#if 1
//				jt->LocalRotation = js->LocalRotation;
//				jt->OffsetMatrix = js->OffsetMatrix;
////				// relative source rot to root
////				Quaternionf locRot = sCtrl->m_IKJoints[js].rotGlobal;
////				if (js->Parent != -1)
////					locRot = locRot * sCtrl->m_IKJoints[sCtrl->getBone(js->Parent)].rotGlobal.inverse();
////				locRot.normalize();
////				//parRot = locRot * parRot;
////				//parRot.normalize();
////				
////				//jt->LocalRotation = r * parRot.inverse();
////				jt->LocalRotation = locRot * jt->OffsetMatrix.block<3,3>(0,0).inverse();
////				jt->LocalRotation.normalize();
//#endif
//			}
//		}
	}

	// create map of joints which chains they contain //TODOff(skade) only compute once
	std::map<SkeletalAnimationController::SkeletalJoint*,std::vector<IKChain*>> jointToChain;
	auto& chains = tCtrl->m_ikArmature.m_jointChains;
	for (uint32_t i = 0; i < chains.size(); ++i)
		for (auto j : chains[i].joints)
			jointToChain[j].push_back(&chains[i]);

	std::function<void(SkeletalAnimationController::SkeletalJoint* j, Matrix4f parentT)> imitate;

	//TODO(skade) might be redundant
	sCtrl->forwardKinematics();

	imitate = [&](SkeletalAnimationController::SkeletalJoint* jt, Matrix4f parentT) {
		IKChain* ct = nullptr;
		if (jointToChain[jt].size() > 0)
			ct = jointToChain[jt][0]; //TODOff(skade) multiple chains?
		bool noMatch = true;
		if (ct) {
			//TODO(skade) find corresponding retarget chain
			int is = 0;
			for (int it = 0; it < m_ikcorr.size();++it)
				if (&tCtrl->m_ikArmature.m_jointChains[it] == ct)
					is = m_ikcorr[it];
			IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];

			int i = std::distance(ct->joints.begin(),std::find(ct->joints.begin(),ct->joints.end(),jt));
			std::vector<int> matchIdx = jointIndexingFunc(i,cs,*ct);

			if (matchIdx.size()) {
				// old parent joint retrival
				SkeletalAnimationController::SkeletalJoint* js = cs.joints[matchIdx[0]];
				Eigen::Matrix4f jsT = CForgeMath::translationMatrix(js->LocalPosition)
				                    * CForgeMath::rotationMatrix(js->LocalRotation)
				                    * CForgeMath::scaleMatrix(js->LocalScale);
				Eigen::Matrix4f parentS = Matrix4f::Identity();
				{
					auto* jsc = js;
					Matrix4f adjS = Matrix4f::Identity();
					while (jsc->Parent != -1) {
						jsc = sCtrl->getBone(jsc->Parent);
						Eigen::Matrix4f jscT = CForgeMath::translationMatrix(jsc->LocalPosition)
											* CForgeMath::rotationMatrix(jsc->LocalRotation)
											* CForgeMath::scaleMatrix(jsc->LocalScale);
						parentS = jscT * parentS;

						////TODO(skade) adj
						//// with adjustment
						//parentS = jscT * adjS * parentS;
						//Matrix4f adjS = js->OffsetMatrix.inverse() * adjS;
					}
				}
				
				//Matrix4f t = jt->OffsetMatrix * parentT.inverse() * parentS * js->OffsetMatrix.inverse() * jsT;
				//Matrix4f t = jt->OffsetMatrix * parentT.inverse() * parentS * js->OffsetMatrix.inverse() * jsT;

				// new local transform
				Matrix4f t = Matrix4f::Identity();

				// parent target
				//if (jt->Parent != -1 && js->Parent != -1) {
				//	auto* jtp = tCtrl->getBone(jt->Parent);
				//	auto* jsp = sCtrl->getBone(js->Parent);

					// current global transform of retargeted parent
					//Matrix4f parentGlobal =  parentT * jtp->OffsetMatrix;
					//Matrix4f parentGlobalS = parentS * jsp->OffsetMatrix;

					// allign next transform so global transform of target and source are identical
					//t = jsT;

					////TODO(skade) adj
					//// get parent relative joint change of restpose
					//Matrix4f adjS = js->OffsetMatrix.inverse() * jsp->OffsetMatrix;
					//Matrix4f adjT = jt->OffsetMatrix.inverse() * jtp->OffsetMatrix;
					//adjS.block<3,1>(0,3) = Vector3f::Zero();
					//adjT.block<3,1>(0,3) = Vector3f::Zero();
					//t = adjT.inverse() * parentT.inverse() * parentS * adjS
					//	* jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();
					
					// correct but doesnt account for rest pose differences
					//t = parentT.inverse() * js->SkinningMatrix * jt->OffsetMatrix.inverse();
					//t = parentT.inverse() * parentS * jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();

					//t = parentT.inverse() * parentS
					//	* jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();

					//TODO(skade) optionally insert multiple source joints
					//if (js->Parent != -1) {
						Quaternionf combinedSourceRot = sCtrl->m_IKJoints[js].rotGlobal;
						for (int j=1;j<matchIdx.size();++j) {
							combinedSourceRot = cs.joints[j]->LocalRotation * combinedSourceRot;
						}

						t = parentT.inverse() //* parentS
							//* jsT
							//* CForgeMath::rotationMatrix(sCtrl->m_IKJoints[sCtrl->getBone(js->Parent)].rotGlobal).inverse()
							* CForgeMath::rotationMatrix(combinedSourceRot)
							* js->OffsetMatrix
							* jt->OffsetMatrix.inverse();
					//}
							//* (CForgeMath::translationMatrix(sCtrl->m_IKJoints[js].posGlobal)

					//parentS = parentS * js->OffsetMatrix * jsT;

					// correct
					parentT = parentT * t;
					// but also means:
					//parentT = parentS
					//	* jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();

					////TODO(skade) adj
					//parentT = parentS * adjS
					//	* jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();
				//}
				
				{ // set local rotation of joint
					Vector3f p,s; Quaternionf r;
					MRMutil::deconstructMatrix(t,&p,&r,&s);
					//jt->LocalPosition = p;
					jt->LocalRotation = r;
					//jt->LocalScale = s;
				}
				noMatch = false;
			}
		}
		if (noMatch) {
			Eigen::Matrix4f jtT = CForgeMath::translationMatrix(jt->LocalPosition)
			                    * CForgeMath::rotationMatrix(jt->LocalRotation)
			                    * CForgeMath::scaleMatrix(jt->LocalScale);
			
			////TODO(skade) adj
			//Matrix4f adjT = Matrix4f::Identity();
			//if (jt->Parent != -1) {
			//	auto* jtp = tCtrl->getBone(jt->Parent);
			//	Matrix4f adjT = jt->OffsetMatrix.inverse() * jtp->OffsetMatrix;
			//}
			//parentT = parentT * jtT * adjT;

			parentT = parentT * jtT;
		}

		for (auto child : jt->Children) {
			imitate(tCtrl->getBone(child), parentT);
		}
	};

	if (m_imitiateAngle)
		imitate(tCtrl->getRoot(), Eigen::Matrix4f::Identity());

	// copy root position
	for (int i=0;i< tCtrl->boneCount();++i) {
		auto jt = tCtrl->getBone(i);
		if (jt->Parent == -1) {
			auto jt = tCtrl->getBone(i);
			for (int j=0;j< sCtrl->boneCount();++j) {
				auto js = sCtrl->getBone(j);
				if (js->Parent == -1) {
					if (m_copy_rootPos) {
						float scale = m_tar_rootPos.norm()/m_src_rootPos.norm();
						scale = CForgeMath::lerp(1.f,scale,m_scale_rootPos);
						jt->LocalPosition = js->LocalPosition * scale;
					}
					if (m_copy_rootRot)
						jt->LocalRotation = Quaternionf(js->LocalRotation.toRotationMatrix() * js->OffsetMatrix.block<3,3>(0,0) * jt->OffsetMatrix.inverse().block<3,3>(0,0));
					break;
				}
			}
			break;
		}
	}
	tCtrl->forwardKinematics();
};
void MRlimb::reset() {
	m_scale_limbs.clear();
	m_tar_limbLen.clear();
	m_src_limbLen.clear();

	m_ikcorr.clear();
	m_sCE.reset();
	m_tCE.reset();
	m_active = false;
}

void MRlimb::bakingInit() {
	m_isBaking = true;
	m_bakingInit = false;
}
void MRlimb::bakingUpdate(float FPS) {
	auto& sce = m_sCE.lock();
	auto& tce = m_tCE.lock();
	if (!sce || !tce || !sce->pAnimCurr) {
		if (!sce->pAnimCurr)
			std::cerr << "error baking: source has no animation selected";
		m_isBaking = false;
		return;
	}

	static T3DMesh<float>::SkeletalAnimation* newAnim = nullptr;

	static bool finished = true;
	static float t = 0.;

	if (!m_bakingInit) {
		finished = false;
		// create new pAnim for source
		sce->pAnimCurr->t = 0.;
		auto* sourceAnim = sce->controller->animation(sce->pAnimCurr->AnimationID);

		//create new animation container to write keyframes back to
		newAnim = new T3DMesh<float>::SkeletalAnimation();
		newAnim->Duration = sce->pAnimCurr->Duration;
		newAnim->Name = sourceAnim->Name;
		newAnim->SamplesPerSecond = sce->pAnimCurr->SamplesPerSecond;
		tce->mesh.addSkeletalAnimation(newAnim,false);
		m_bakingInit = true;

		for (int i=0;i<tce->mesh.boneCount();++i) {
			auto b = tce->mesh.getBone(i);
			T3DMesh<float>::BoneKeyframes* nkf = new T3DMesh<float>::BoneKeyframes();
			nkf->BoneID = b->ID;
			nkf->BoneName = b->Name;
			nkf->ID = i;
			newAnim->Keyframes.push_back(nkf);
		}
	}
	assert(newAnim);

	{// source do normal playback but snap to keyframes instead of interpolating
		auto* pA = sce->pAnimCurr;
		int animRotSize = sce->controller->animation(pA->AnimationID)->Keyframes[0]->Rotations.size()-1;
		float animTime = sce->controller->animation(pA->AnimationID)->Keyframes[0]->Timestamps.back();

		t += 1./FPS * pA->Speed; // make sure no individual keyframes are skipped here
		sce->animFrameCurr = t / animTime * animRotSize;

		// clamp timing back to keyframe so it ggets correctly displayed
		pA->t = float(sce->animFrameCurr) / (animRotSize) * animTime; //TODO(skade) make pose configurable, see set and get on sequencer

		if (sce->animFrameCurr > animRotSize)
			finished = true; //baking completed
		sce->actor->update();
	}
	sce->controller->forwardKinematics();
	tce->controller->forwardKinematics();

	update();
	// target complete ik
	if (tce->m_IKCupdate)
		tce->controller->update(60.0f / FPS);
	tce->actor->update();

	if (newAnim->Keyframes[0]->Timestamps.size() == 0 || sce->pAnimCurr->t > newAnim->Keyframes[0]->Timestamps.back()) {
		// read back target keyframe data into container
		for (int i=0;i<tce->controller->boneCount(); ++i) {
			auto* b = tce->controller->getBone(i);

			//TODO(skade) assumes that mesh and controller bones are the same
			newAnim->Keyframes[i]->Positions.push_back(b->LocalPosition);
			newAnim->Keyframes[i]->Rotations.push_back(b->LocalRotation);
			newAnim->Keyframes[i]->Scalings.push_back(b->LocalScale);
			newAnim->Keyframes[i]->Timestamps.push_back(sce->pAnimCurr->t);
		}
	}

	if (finished) {
		t = 0.;

		tce->controller->addAnimationData(newAnim);
		m_isBaking = false;
		newAnim = nullptr;
	}
}

}//CForge
