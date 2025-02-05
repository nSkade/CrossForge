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

	bool m_imitiateAngle = true;
	bool m_copy_rootPos = true;
	float m_scale_rootPos = 1.;
	bool m_copy_rootRot = true;

	std::vector<float> m_scale_limbs;

	std::vector<std::shared_ptr<IKTarget>> m_targets;
	std::weak_ptr<CharEntity> m_sCE;
	std::weak_ptr<CharEntity> m_tCE;
private:
	Vector3f m_src_rootPos;
	Vector3f m_tar_rootPos;
	std::vector<float> m_src_limbLen;
	std::vector<float> m_tar_limbLen;
	int jointIndexingFunc(int tarIdx, IKChain& cs, IKChain& ct);
	bool m_active = false;
	//Matrix4f sourceToTargetTrans; // transform matrix that maps source to target space //TODO(skade)

	// limb correspondences
	// source -> target ik chains to retarget
	std::vector<int> m_ikcorr;
};

}//CForge
