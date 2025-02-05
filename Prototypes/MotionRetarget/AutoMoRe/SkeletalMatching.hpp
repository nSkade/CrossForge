#include <Prototypes/MotionRetarget/IK/IKController.hpp>

namespace CForge {
using namespace Eigen;

class SkeletalMatcher {
public:
	bool injectiveOnly = true;
	std::vector<int> m_corr; // correspondences of last matched pair

	// parameters
	float m_wRootPos = .1f;
	float m_wEefPos  = .1f;
	float m_wDir     = 2.f;
	float m_wMeanPos = 0.f;

	void reset() { m_corr.clear(); };

	//TODO(skade) incorporate injective only in extra function
	// compute matching with all chains and take one with highest score
	//	std::map<int,bool> used; // for injective matching

	void skelMatch(IKController* csc, IKController* ctc) {

		std::vector<int> ret;
		for (auto& jct : ctc->m_ikArmature.m_jointChains) {
			float bestScore = 0.;
			int bestIdx = 0;
			for (int i = 0; i < csc->m_ikArmature.m_jointChains.size(); ++i) {
				auto& jcs = csc->m_ikArmature.m_jointChains[i];

				// positional scores
				float rootPos = 1./(1.+(csc->m_IKJoints[jcs.joints.back()].posGlobal - ctc->m_IKJoints[jct.joints.back()].posGlobal).norm());
				float eefPos = 1./(1.+(csc->m_IKJoints[jcs.joints.front()].posGlobal - ctc->m_IKJoints[jct.joints.front()].posGlobal).norm());
				
				// directional score
				Vector3f dirT = ctc->m_IKJoints[jct.joints.front()].posGlobal - ctc->m_IKJoints[jct.joints.back()].posGlobal;
				Vector3f dirS = csc->m_IKJoints[jcs.joints.front()].posGlobal - csc->m_IKJoints[jcs.joints.back()].posGlobal;
				float dotS = dirT.dot(dirS);

				float meanPos;
				{
					Vector3f csmp = Vector3f::Zero();
					Vector3f ctmp = Vector3f::Zero();
					for (auto j : jcs.joints)
						csmp += csc->m_IKJoints[j].posGlobal;
					csmp /= jcs.joints.size();
					for (auto j : jct.joints)
						ctmp += ctc->m_IKJoints[j].posGlobal;
					ctmp /= jct.joints.size();
					meanPos = 1./(1.+ (csmp-ctmp).norm());
				}
				
				float score = m_wRootPos * rootPos
				            + m_wEefPos * eefPos
				            + m_wDir * dotS
				            + m_wMeanPos * meanPos;

				if (score > bestScore) {
					bestScore = score;
					bestIdx = i;
				}
			}
			ret.push_back(bestIdx);
		}
		
		m_corr = ret;
	};
};

}//CForge
