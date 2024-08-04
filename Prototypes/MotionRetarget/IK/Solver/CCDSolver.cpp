#include "CCDSolver.hpp"
#include <Prototypes/MotionRetarget/IKController.hpp>

namespace CForge {

//template<IKSolverCCD::CCDtype type>
void IKSolverCCD::solve(std::string segmentName, IKController* pController) {
	//std::vector<IKController::SkeletalJoint*>& Chain = pController->m_jointChains.at(segmentName).joints;
	std::vector<IKController::SkeletalJoint*>& Chain = pController->getIKChain(segmentName)->joints;
	//IKTarget* target = pController->m_jointChains.at(segmentName).target;
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

		//TODO(skade) implement Forward CCD
		//for (int32_t k = Chain.size()-1; k >= 0; --k) {
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
			if (std::abs(theta) < FLT_EPSILON || std::isnan(theta)) //TODO(skade) why theta sometimes nan?
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

			//TODO(skade)
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
//TODO(skade) remove
//template void IKSolverCCD::solve<IKSolverCCD::BACKWARD>(std::string segmentName, IKController* pController);
//template void IKSolverCCD::solve<IKSolverCCD::FORWARD>(std::string segmentName, IKController* pController);

//struct EndEffectorData { // X corresponds to entry from end-effector to root
//	Eigen::Matrix3Xf EEPosLocal;      // current local joint positions, applied onto Controller
//	Eigen::Matrix3Xf EEPosGlobal;     // current global joint positions
//	Eigen::Matrix3Xf TargetPosGlobal; // target global joint positions
//};

//TODO(skade) rewrite
//void IKController::rotateGaze(void) {
//	Vector3f EPos = m_pHead->pEndEffectorData->EEPosGlobal.col(0);
//	Vector3f TPos = m_pHead->pEndEffectorData->TargetPosGlobal.col(0);
//	Vector3f CurrentDir = (EPos - m_pHead->posGlobal).normalized();
//	Vector3f TargetDir = (TPos - m_pHead->posGlobal).normalized();
//
//	if (std::abs(1.0f - CurrentDir.dot(TargetDir) > 1e-6f)) {
//		// compute unconstrained global rotation to align both directional vectors in world space
//		Quaternionf GlobalIncrement;
//		GlobalIncrement.setFromTwoVectors(CurrentDir, TargetDir);
//		Quaternionf NewGlobalRotation = GlobalIncrement * m_pHead->rotGlobal;
//		
//		// transform new global rotation to new local rotation
//		Quaternionf NewLocalRotation = (m_pHead == m_pRoot) ? NewGlobalRotation : m_pHead->pParent->rotGlobal.conjugate() * NewGlobalRotation;
//		NewLocalRotation.normalize();
//
//		// constrain new local rotation if joint is not unconstrained
//		if (m_pHead->pLimits != nullptr) NewLocalRotation = m_pHead->pLimits->constrain(NewLocalRotation);
//
//		// apply new local rotation to joint 
//		m_pHead->LocalRotation = NewLocalRotation;
//
//		// compute new global joint rotation and apply to gaze direction
//		forwardKinematics(m_pHead);
//	}
//}//rotateGaze

//void IKController::ikCCDglobal(const std::string segmentName) {
//	std::vector<SkeletalJoint*>& Chain = m_jointChains.at(segmentName).joints;
//	EndEffectorData* pEffData = m_IKJoints[Chain[0]]->pEndEffectorData;
//	Matrix3Xf LastEndEffectorPoints;
//
//	for (int32_t i = 0; i < m_MaxIterations; ++i) {
//		LastEndEffectorPoints = pEffData->EEPosGlobal;
//
//		for (int32_t k = 0; k < Chain.size(); ++k) {
//			SkeletalJoint* pCurrent = Chain[k];
//			IKJoint* pCurrentIK = m_IKJoints[pCurrent];
//
//			// compute unconstrained global rotation that best aligns position and orientation of end effector with desired target values
//			Quaternionf GlobalIncrement = computeUnconstrainedGlobalRotation(pCurrentIK, pEffData);
//			Quaternionf NewGlobalRotation = GlobalIncrement * pCurrentIK->rotGlobal;
//			
//			// transform new global rotation to new local rotation
//			Quaternionf NewLocalRotation;
//			if (pCurrent == m_pRoot)
//				NewLocalRotation = NewGlobalRotation;
//			else
//				NewLocalRotation = m_IKJoints[m_Joints[pCurrent->Parent]]->rotGlobal.conjugate() * NewGlobalRotation;
//
//			NewLocalRotation.normalize();
//
//			//TODO(skade)
//			// constrain new local rotation if joint is not unconstrained
//			//if (pCurrentIK->pLimits != nullptr)
//			//	NewLocalRotation = pCurrentIK->pLimits->constrain(NewLocalRotation);
//
//			// apply new local rotation to joint
//			pCurrent->LocalRotation = NewLocalRotation;
//
//			// update kinematic chain
//			forwardKinematics(pCurrent);
//		}//for[each joint in chain]
//
//		// check for termination -> condition: end-effector has reached the targets position and orientation
//		Matrix3Xf EffectorTargetDiff = pEffData->TargetPosGlobal - pEffData->EEPosGlobal;
//		float DistError = EffectorTargetDiff.cwiseProduct(EffectorTargetDiff).sum() / float(EffectorTargetDiff.cols());
//		if (DistError < m_thresholdDist)
//			return;
//
//		Matrix3Xf EffectorPosDiff = pEffData->EEPosGlobal - LastEndEffectorPoints;
//		float PosChangeError = EffectorPosDiff.cwiseProduct(EffectorPosDiff).sum() / float(EffectorPosDiff.cols());
//		if (PosChangeError < m_thresholdPosChange)
//			return;
//	}//for[m_MaxIterations]
//}//ikCCDglobal

//TODOf(skade) target for every joint
//Quaternionf IKSolverCCD::computeUnconstrainedGlobalRotation(IKJoint* pJoint, IKController::EndEffectorData* pEffData) {
//
//	//TODO(skade) doesnt simply rotate joint to target, but takes whole chain into considertaiton
//	           // every joint has its own target, find best rotation to reduce error
//
//	//TODO: combine points of multiple end effectors and targets into 2 point clouds to compute CCD rotation for multiple end effectors?
//	Matrix3Xf EndEffectorPoints = pEffData->EEPosGlobal.colwise() - pJoint->GlobalPosition; // current local joint position
//	Matrix3Xf TargetPoints = pEffData->TargetPosGlobal.colwise() - pJoint->GlobalPosition;  // target local joint position
//
//#if 1
//	// compute matrix W
//	Matrix3f W = TargetPoints * EndEffectorPoints.transpose(); // only take first 3 points from eef to root
//
//	// compute singular value decomposition
//	JacobiSVD<Matrix3f> SVD(W, ComputeFullU | ComputeFullV);
//	Matrix3f U = SVD.matrixU();
//	Matrix3f V = SVD.matrixV();
//
//
//	// compute rotation
//	Matrix3f R = U * V.transpose();
//	Quaternionf GlobalRotation(R);
//	GlobalRotation.normalize();
//#else
//	// compute rotation using quaternion characteristic polynomial from: "Closed-form solution of absolute orientation using unit quaternions." - Berthold K. P. Horn, 1987
//	// https://web.stanford.edu/class/cs273/refs/Absolute-OPT.pdf
//
//	//
//	//			0	1	2
//	//		0	Sxx	Sxy	Sxz
//	// S =	1	Syx	Syy	Syz
//	//		2	Szx	Szy	Szz
//	//		
//	Matrix3f S = EndEffectorPoints * TargetPoints.transpose(); // only take first 3 points from eff to root
//
//	//
//	// N = 
//	//		Sxx + Syy + Szz		Syz - Szy			 Szx - Sxz			 Sxy - Syx
//	//		Syz - Szy			Sxx - Syy - Szz		 Sxy + Syx			 Szx + Sxz
//	//		Szx - Sxz			Sxy + Syx			-Sxx + Syy - Szz	 Syz + Szy
//	//		Sxy - Syx			Szx + Sxz			 Syz + Szy			-Sxx - Syy + Szz
//	//
//	Matrix4f N = Matrix4f::Zero();
//
//	N(0, 0) = S(0, 0) + S(1, 1) + S(2, 2);  //  Sxx + Syy + Szz
//	N(0, 1) = S(1, 2) - S(2, 1);            //  Syz - Szy
//	N(0, 2) = S(2, 0) - S(0, 2);            //  Szx - Sxz
//	N(0, 3) = S(0, 1) - S(1, 0);            //  Sxy - Syx
//	
//	N(1, 0) = N(0, 1);                      //  Syz - Szy
//	N(1, 1) = S(0, 0) - S(1, 1) - S(2, 2);  //  Sxx - Syy - Szz
//	N(1, 2) = S(0, 1) + S(1, 0);            //  Sxy + Syx
//	N(1, 3) = S(2, 0) + S(0, 2);            //  Szx + Sxz
//	
//	N(2, 0) = N(0, 2);                      //  Szx - Sxz
//	N(2, 1) = N(1, 2);                      //  Sxy + Syx
//	N(2, 2) = -S(0, 0) + S(1, 1) - S(2, 2); // -Sxx + Syy - Szz
//	N(2, 3) = S(1, 2) + S(2, 1);            //  Syz + Szy
//	
//	N(3, 0) = N(0, 3);                      //  Sxy - Syx
//	N(3, 1) = N(1, 3);                      //  Szx + Sxz
//	N(3, 2) = N(2, 3);                      //  Syz + Szy
//	N(3, 3) = -S(0, 0) - S(1, 1) + S(2, 2); // -Sxx - Syy + Szz
//	
//	Eigen::SelfAdjointEigenSolver<Matrix4f> Solver(4);
//	Solver.compute(N);
//	Vector4f BiggestEVec = Solver.eigenvectors().col(3); // last column of eigenvectors() matrix contains eigenvector of largest eigenvalue;
//	                                                     // that's supposed to equal the desired rotation quaternion
//	Quaternionf GlobalRotation = Quaternionf(BiggestEVec(0), BiggestEVec(1), BiggestEVec(2), BiggestEVec(3));
//#endif
//	return GlobalRotation;
//}//computeUnconstrainedGlobalRotation

}//CForge

