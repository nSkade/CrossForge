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

	//TODOff(skade) incorporate injective only in extra function
	// compute matching with all chains and take one with highest score
	//	std::map<int,bool> used; // for injective matching

	void skelMatch(IKController* csc, IKController* ctc);

	/**
	 * @brief uses hungarian algorithm to optimal match chains globally
	*/
	void skelMatchInj(IKController* csc, IKController* ctc);
private:
	// Helper function to calculate the score between two joint chains
	double calculateScore(IKController* csc, const IKChain& jcs, IKController* ctc, const IKChain& jct);
};

}//CForge
