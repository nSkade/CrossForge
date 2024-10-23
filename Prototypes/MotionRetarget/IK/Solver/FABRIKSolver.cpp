#include "FABRIKSolver.hpp"
#include <Prototypes/MotionRetarget/IK/IKController.hpp>

namespace CForge {

void IKSfabrik::solve(std::string segmentName, IKController* pController) {
	//std::vector<IKController::SkeletalJoint*>& Chain = pController->m_jointChains.at(segmentName).joints;
	std::vector<IKController::SkeletalJoint*>& Chain = pController->getIKChain(segmentName)->joints;
	//IKTarget* target = pController->m_jointChains.at(segmentName).target;
	IKTarget* target = pController->getIKChain(segmentName)->target.lock().get();
	if (!target)
		return;

	//TODOff(skade) make member for per chain solver to avoid memory alloc?
	fbrkPoints.clear(); // global position for fabrik calculation
	fbrkPoints.reserve(Chain.size()); //TODOff(skade)
	for (uint32_t i = 0; i < Chain.size(); ++i)
		fbrkPoints.push_back(pController->m_IKJoints[Chain[i]].posGlobal);

	std::vector<float> fbrkLen(Chain.size()); // joint lengths
	float totalChainLength = 0.;

	// compute chain lengths, root joint has length to next joint
	fbrkLen[0] = 0.f; // eef has no length
	for (uint32_t i=1;i<Chain.size();++i) {
		fbrkLen[i] = (pController->m_IKJoints[Chain[i-1]].posGlobal - pController->m_IKJoints[Chain[i]].posGlobal).norm();
		totalChainLength += fbrkLen[i];
	}

	// original root position, needs to be restored on end
	Vector3f rootOrigPos = pController->m_IKJoints[Chain.back()].posGlobal;
	Vector3f rootToTarget = target->pos - rootOrigPos;
	
	if (rootToTarget.norm() >= totalChainLength) {
		// target unreachable, chain is straight line
		// root already set
		for (int32_t i = Chain.size()-2; i >= 0; --i) {
			Vector3f ofst = rootToTarget.normalized() * fbrkLen[i+1];
			fbrkPoints[i] = ofst + fbrkPoints[i+1];
		}
	}
	else {
		// target reachable
		Vector3f prevPos = Vector3f(FLT_MAX,FLT_MAX,FLT_MAX);
		for (uint32_t iter=0;iter<m_MaxIterations;++iter) {
			Vector3f eefToTar = target->pos - fbrkPoints[0];

			if (eefToTar.norm() < m_thresholdDist)
				break;
			if ((eefToTar-prevPos).norm() < m_thresholdPosChange)
				break;
			prevPos = eefToTar;

			// Forward reach
			fbrkPoints[0] = target->pos;
			for (uint32_t i=1;i<Chain.size();++i) {
				Vector3f line = fbrkPoints[i] - fbrkPoints[i-1];
				line.normalize();

				Vector3f newPos = fbrkPoints[i-1] + (line*fbrkLen[i]);
				fbrkPoints[i] = newPos;
			}

			// Backward reach
			fbrkPoints.back() = rootOrigPos;
			for (int32_t i=Chain.size()-2;i>=0;--i) {
				Vector3f line = fbrkPoints[i] - fbrkPoints[i+1];
				line.normalize();

				Vector3f newPos = fbrkPoints[i+1] + (line*fbrkLen[i+1]);
				fbrkPoints[i] = newPos;
			}
		}
	}

	backwardKinematics(segmentName,pController,fbrkPoints);
}

void IKSfabrik::backwardKinematics(std::string segmentName, IKController* pController, const std::vector<Vector3f>& fbrkPoints) {
	//std::vector<IKController::SkeletalJoint*>& Chain = pController->m_jointChains.at(segmentName).joints;
	std::vector<IKController::SkeletalJoint*>& Chain = pController->getIKChain(segmentName)->joints;
	//IKTarget* target = pController->m_jointChains.at(segmentName).target;
	IKTarget* target = pController->getIKChain(segmentName)->target.lock().get();
	if (!target)
		return;

	for (int32_t i = Chain.size() - 1; i > 0; --i) {
		// angle axis for every fabrik point
		IKJoint& ikJ = pController->m_IKJoints[Chain[i]];

		// destination
		Vector3f destvec = fbrkPoints[i-1] - ikJ.posGlobal;
		destvec.normalize();

		// current dir
		Vector3f curvec = pController->m_IKJoints[Chain[i-1]].posGlobal - ikJ.posGlobal;
		curvec.normalize();

		float curDotDest = curvec.dot(destvec);
		if (curDotDest > 1.)
			curDotDest = 1.;
		else if (curDotDest < -1.)
			curDotDest = -1.;
		float angle = std::acos(curDotDest);
		
		Quaternionf globToLoc = ikJ.rotGlobal.inverse();
		Vector3f rotAx = curvec.cross(destvec);
		rotAx.normalize();
		Vector3f rotAxLoc = globToLoc*rotAx;

		Quaternionf rot(AngleAxis(angle,rotAxLoc));
		//TODOff(skade)
		if (std::abs(angle) < FLT_EPSILON)
			rot = Quaternionf::Identity();
		
		Chain[i]->LocalRotation = Chain[i]->LocalRotation * rot;
		Chain[i]->LocalRotation.normalize();

		pController->forwardKinematics(Chain[i]);
	}
}

}//CForge
