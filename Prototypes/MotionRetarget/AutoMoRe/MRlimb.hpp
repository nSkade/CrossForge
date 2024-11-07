#pragma once

#include "IMoRe.hpp"

#include <Prototypes/MotionRetarget/IK/IKChain.hpp>
#include <Prototypes/MotionRetarget/CharEntity.hpp>
#include <Prototypes/MotionRetarget/IK/IKController.hpp>

namespace CForge {
using namespace Eigen;
	
class MRlimb : IMoRe {
public:
	//TODOf(skade) limb matching
	//TODO(skade) 
	/*
	 * @param corr source to target chain correspondence
	*/
	void initialize(std::shared_ptr<CharEntity> source, std::shared_ptr<CharEntity> target, std::vector<int> corr) {
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
	void update() {
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

#if 0
			// imitate joint angles
			for (auto jt : ct.joints) {
				Vector3f jtPos = tCtrl->m_IKJoints[jt].posGlobal;

				SkeletalAnimationController::SkeletalJoint* cj = nullptr;
				float closestDist = std::numeric_limits<float>::max();
				// find closest joint to imitate angle
				for (auto* js : cs.joints) {
					float dist = (jtPos-sCtrl->m_IKJoints[js].posGlobal).norm(); //TODO(skade) from current pose, need rest pose global pos instead
					if (dist < closestDist) {
						closestDist = dist;
						cj = js;
					}
				}
				// set local rotation to joint rotation
				jt->LocalRotation = cj->LocalRotation;
			}
#endif
		}
	};
	void reset() {
		m_ikcorr.clear();
		m_sCE.reset();
		m_tCE.reset();
		m_active = false;
	}
	bool active() {return m_active;};
private:
	bool m_active = false;
	std::weak_ptr<CharEntity> m_sCE;
	std::weak_ptr<CharEntity> m_tCE;
	//Matrix4f sourceToTargetTrans; // transform matrix that maps source to target space //TODO(skade)

	// limb correspondences
	// source -> target ik chains to retarget
	std::vector<int> m_ikcorr;
};

}//CForge
