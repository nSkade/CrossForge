#include "MRlimb.hpp"

#include "Prototypes/MotionRetarget/CMN/MRMutil.hpp"

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

		//TODO(skade) create target if it doesnt exist
		std::shared_ptr<IKTarget> t =
			std::make_shared<IKTarget>(*sCtrl->m_ikArmature.m_jointChains[is].target.lock().get());

		Vector3f pos; Quaternionf rot; Vector3f scale;
		target->sgn.buildTansformation(&pos,&rot,&scale);
		t->m_sgnT = CForgeMath::translationMatrix(pos) * CForgeMath::rotationMatrix(rot) * CForgeMath::scaleMatrix(scale);

		m_targets.emplace_back(t);
		ct.target = t;
	}
};
int MRlimb::jointIndexingFunc(int tarIdx, IKChain& cs, IKChain& ct) {
	auto source = m_sCE.lock();
	auto target = m_tCE.lock();
	if (!source || !target) {
		m_active = false;
		return -1;
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
#if 1
	// by index
	if (tarIdx < cs.joints.size())
		return tarIdx;
#endif
	return -1; // no match
}
void MRlimb::update() {
	auto source = m_sCE.lock();
	auto target = m_tCE.lock();
	if (!source || !target)
		m_active = false;
	if (!m_active) {
		//m_ikcorr.clear(); //TODO(skade) more cleanup?
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
			int matchIdx = jointIndexingFunc(i,cs,*ct);

			if (matchIdx != -1) {
				SkeletalAnimationController::SkeletalJoint* js = cs.joints[matchIdx];
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

					t = parentT.inverse() * parentS
						* jsT * js->OffsetMatrix * jt->OffsetMatrix.inverse();

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

}//CForge
