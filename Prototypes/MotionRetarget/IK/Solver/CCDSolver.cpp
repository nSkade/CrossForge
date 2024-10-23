#include "CCDSolver.hpp"
#include <Prototypes/MotionRetarget/IK/IKController.hpp>

namespace CForge {

//template<IKSccd::Type type>
void IKSccd::solve(std::string segmentName, IKController* pController) {
	std::vector<IKController::SkeletalJoint*>& Chain = pController->getIKChain(segmentName)->joints;
	IKTarget* target = pController->getIKChain(segmentName)->target.lock().get();
	if (!target)
		return;

	Vector3f lastEFpos;
	IKJoint& eef = pController->m_IKJoints[Chain[0]];

	for (int32_t i = 0; i < m_MaxIterations; ++i) {
		lastEFpos = eef.posGlobal;

		// check for termination -> condition: end-effector has reached the targets position and orientation
		float DistError = (lastEFpos-target->pos).norm();
		if (DistError <= m_thresholdDist)
			return;

		// Backward CCD
		int32_t k = 1;
		if (m_type == FORWARD)
			k = Chain.size()-1;

		bool fwd = m_type == FORWARD;
		for (; fwd ? k >= 1 :  k < Chain.size(); fwd ? --k : ++k) {
			// start at base joint
			IKController::SkeletalJoint* pCurrent = Chain[k];
			IKJoint& pCurrentIK = pController->m_IKJoints[pCurrent];

			// calculate rotation axis
			// joint position to end effector
			Vector3f jpToEE = eef.posGlobal - pCurrentIK.posGlobal;
			jpToEE.normalize();
			// joint position to target position
			Vector3f jpToTar = target->pos - pCurrentIK.posGlobal;
			jpToTar.normalize();

			// rotation angle
			//float theta = std::acos(jpToEE.dot(jpToTar));
			float theta = std::acos(jpToEE.dot(jpToTar));
			if (std::abs(theta) < FLT_EPSILON || std::isnan(theta)) //TODOff(skade) why theta sometimes nan?
				continue;

			// vector we rotate around
			Vector3f rotVec = jpToEE.cross(jpToTar);
			rotVec.normalize();

			Quaternionf globToLoc = pCurrentIK.rotGlobal.inverse();
			globToLoc.normalize();
			
			Vector3f rotVecLocal = globToLoc*rotVec;
			rotVecLocal.normalize();
			
			//Quaternionf GlobalIncrement = Quaternionf(AngleAxis(theta,rotVec));
			//Quaternionf NewGlobalRotation = GlobalIncrement * pCurrentIK.rotGlobal;
			//NewGlobalRotation.normalize();
			
			// transform new global rotation to new local rotation
			//Quaternionf NewLocalRotation = pController->m_IKJoints[m_Joints[pCurrent->Parent]]->rotGlobal.conjugate() * NewGlobalRotation;

			Quaternionf NewLocalRotation = Quaternionf(AngleAxis(theta,rotVecLocal));
			NewLocalRotation.normalize();

			//TODOf(skade)
			//// constrain new local rotation if joint is not unconstrained
			//if (pCurrentIK.pLimits != nullptr)
			//	NewLocalRotation = pCurrentIK.pLimits->constrain(NewLocalRotation);

			// apply new local rotation to joint
			pCurrent->LocalRotation = pCurrent->LocalRotation * NewLocalRotation;
			pCurrent->LocalRotation.normalize();

			// update kinematic chain
			pController->forwardKinematics(pCurrent);
		}//for[each joint in chain]

		float PosChangeError = (eef.posGlobal - lastEFpos).norm();
		if (PosChangeError < m_thresholdPosChange)
			return;
	}//for[m_MaxIterations]
}//solve

}//CForge
