#include "MRlimb.hpp"

namespace CForge {
using namespace Eigen;

void MRlimb::initialize(std::shared_ptr<CharEntity> source, std::shared_ptr<CharEntity> target, std::vector<int> corr) {
	//int idx=0;
	//for (IKChain& c : target->m_ikArmature.m_jointChains) {
	//	//TODO(skade) for now match joint by ikchain order make parameter to be configurable by ui

	//	//TODO(skade) find corresponding ikchain and match
	//	c.target = source->m_ikArmature.m_jointChains[idx].target;
	//	idx++;
	//}
	m_sCE = source;
	m_tCE = target;
	m_ikcorr = corr;
	m_active = true;
};
void MRlimb::update() {
	auto source = m_sCE.lock();
	auto target = m_tCE.lock();
	if (!source || !target) {
		m_active = false;
		return;
	}
	auto& sCtrl = source->controller;
	auto& tCtrl = target->controller;

	//int idx=0;
	//for (IKChain& c : target->m_ikArmature.m_jointChains) {
	//	//TODO(skade) for now match joint by ikchain order make parameter to be configurable by ui

	//	//TODO(skade) find corresponding ikchain and match
	//	c.target = source->m_ikArmature.m_jointChains[idx].target;
	//	idx++;
	//}
	for (int it = 0; it < m_ikcorr.size();++it) {
		int is = m_ikcorr[it];
		//TODO(skade) for now match joint by ikchain order make parameter to be configurable by ui

		//TODO(skade) find corresponding ikchain and match
		IKChain& cs = sCtrl->m_ikArmature.m_jointChains[is];
		IKChain& ct = tCtrl->m_ikArmature.m_jointChains[it];
		ct.target = cs.target;

		//TODO(skade) root
		// get parent rot of root joint for reference
		auto csRoot = cs.joints.back();
		Quaternionf rootGlobRot = Quaternionf::Identity();
		rootGlobRot = sCtrl->m_IKJoints[csRoot].rotGlobal;
		//if (csRoot->Parent != -1)
		//	rootGlobRot = sCtrl->m_IKJoints[sCtrl->getBone(csRoot->Parent)].rotGlobal;
		Quaternionf parRot = rootGlobRot;


		// imitate joint angles, start from root of chain
		for (int i=ct.joints.size()-1; i >= 0; --i) {
			SkeletalAnimationController::SkeletalJoint* jt = ct.joints[i];

			// find closest source joint to imitate angle from
			SkeletalAnimationController::SkeletalJoint* js = nullptr;

			// by distance
			//float closestDist = std::numeric_limits<float>::max();
			//for (auto* js : cs.joints) {
			//	float dist = (tCtrl->m_IKJoints[jt].posGlobal-sCtrl->m_IKJoints[js].posGlobal).norm(); //TODO(skade) from current pose, need rest pose global pos instead
			//	if (dist < closestDist) {
			//		closestDist = dist;
			//		cj = js;
			//	}
			//}

			// by distribution
			//cj = cs.joints[std::floor((float(i)/ct.joints.size())*cs.joints.size())];

			// by index
			if (i < cs.joints.size()) {
				js = cs.joints[i];
			}
			if (js) {
#if 1
				// relative source rot to root
				Quaternionf locRot = sCtrl->m_IKJoints[js].rotGlobal;
				if (js->Parent != -1)
					locRot = locRot * sCtrl->m_IKJoints[sCtrl->getBone(js->Parent)].rotGlobal.inverse();
				locRot.normalize();
				//parRot = locRot * parRot;
				//parRot.normalize();
				
				//jt->LocalRotation = r * parRot.inverse();
				jt->LocalRotation = locRot * jt->OffsetMatrix.block<3,3>(0,0).inverse();
				jt->LocalRotation.normalize();
#endif
			}
		}
	}
	// copy root position
	for (int i=0;i< tCtrl->boneCount();++i) {
		auto jt = tCtrl->getBone(i);
		if (jt->Parent == -1) {
			auto jt = tCtrl->getBone(i);
			for (int j=0;j< sCtrl->boneCount();++j) {
				auto js = sCtrl->getBone(j);
				if (js->Parent == -1) {
					jt->LocalPosition = js->LocalPosition;
					break;
				}
			}
			break;
		}
	}
};
void MRlimb::reset() {
	m_ikcorr.clear();
	m_sCE.reset();
	m_tCE.reset();
	m_active = false;
}

}//CForge
