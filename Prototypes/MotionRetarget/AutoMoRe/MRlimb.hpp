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
	void initialize(std::shared_ptr<CharEntity> source, std::shared_ptr<CharEntity> target, std::vector<int> corr);
	void update();
	void reset();
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
