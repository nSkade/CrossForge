#include "CCDSolver.hpp"
#include <Prototypes/MotionRetarget/IKController.hpp>

namespace CForge {

template<IKSolverCCD::CCDtype type>
void IKSolverCCD::solve(std::string segmentName, IKController* pController) {
	std::vector<IKController::SkeletalJoint*>& Chain = pController->m_JointChains.at(segmentName).joints;
	IKTarget* target = pController->m_JointChains.at(segmentName).target;
	Vector3f lastEFpos;
	IKJoint* eef = pController->m_IKJoints[Chain[0]];

	for (int32_t i = 0; i < m_MaxIterations; ++i) {
		lastEFpos = eef->posGlobal;

		// check for termination -> condition: end-effector has reached the targets position and orientation
		float DistError = (lastEFpos-target->pos).norm();
		if (DistError <= m_thresholdDist)
			return;

		//TODO(skade) implement Forward CCD
		//for (int32_t k = Chain.size()-1; k >= 0; --k) {
		// Backward CCD
		int32_t k = 1;
		if (type == FORWARD)
			k = Chain.size()-1;

		bool fwd = type == FORWARD;
		for (; fwd ? k >= 1 :  k < Chain.size(); fwd ? --k : ++k) {
			// start at base joint
			IKController::SkeletalJoint* pCurrent = Chain[k];
			IKJoint* pCurrentIK = pController->m_IKJoints[pCurrent];

			// calculate rotation axis
			// joint position to end effector
			Vector3f jpToEE = eef->posGlobal - pCurrentIK->posGlobal;
			jpToEE.normalize();
			// joint position to target position
			Vector3f jpToTar = target->pos - pCurrentIK->posGlobal;
			jpToTar.normalize();

			// rotation angle
			//float theta = std::acos(jpToEE.dot(jpToTar));
			float theta = std::acos(jpToEE.dot(jpToTar));
			if (std::abs(theta) < FLT_EPSILON || std::isnan(theta)) //TODO(skade) why theta sometimes nan?
				continue;

			// vector we rotate around
			Vector3f rotVec = jpToEE.cross(jpToTar);
			rotVec.normalize();

			Quaternionf globToLoc = pCurrentIK->rotGlobal.inverse();
			globToLoc.normalize();
			
			Vector3f rotVecLocal = globToLoc*rotVec;
			rotVecLocal.normalize();
			
			//Quaternionf GlobalIncrement = Quaternionf(AngleAxis(theta,rotVec));
			//Quaternionf NewGlobalRotation = GlobalIncrement * pCurrentIK->rotGlobal;
			//NewGlobalRotation.normalize();
			
			// transform new global rotation to new local rotation
			//Quaternionf NewLocalRotation = pController->m_IKJoints[m_Joints[pCurrent->Parent]]->rotGlobal.conjugate() * NewGlobalRotation;

			Quaternionf NewLocalRotation = Quaternionf(AngleAxis(theta,rotVecLocal));
			NewLocalRotation.normalize();

			//TODO(skade)
			//// constrain new local rotation if joint is not unconstrained
			//if (pCurrentIK->pLimits != nullptr)
			//	NewLocalRotation = pCurrentIK->pLimits->constrain(NewLocalRotation);

			// apply new local rotation to joint
			pCurrent->LocalRotation = pCurrent->LocalRotation * NewLocalRotation;
			pCurrent->LocalRotation.normalize();

			// update kinematic chain
			pController->forwardKinematics(pCurrent);
		}//for[each joint in chain]

		float PosChangeError = (eef->posGlobal - lastEFpos).norm();
		if (PosChangeError < m_thresholdPosChange)
			return;
	}//for[m_MaxIterations]
}//solve
template void IKSolverCCD::solve<IKSolverCCD::BACKWARD>(std::string segmentName, IKController* pController);
template void IKSolverCCD::solve<IKSolverCCD::FORWARD>(std::string segmentName, IKController* pController);

//Quaternionf IKSolverCCD::computeUnconstrainedGlobalRotation(IKJoint* pJoint, IKController::EndEffectorData* pEffData) {
//
//	//TODO(skade) doesnt rotate joint to target, but takes whole chain into considertaiton
//
//	//TODO: combine points of multiple end effectors and targets into 2 point clouds to compute CCD rotation for multiple end effectors?
//	Matrix3Xf EndEffectorPoints = pEffData->EEPosGlobal.colwise() - pJoint->GlobalPosition; // current local joint position
//	Matrix3Xf TargetPoints = pEffData->TargetPosGlobal.colwise() - pJoint->GlobalPosition;  // target local joint position
//
//#if 0
//	// compute matrix W
//	Matrix3f W = TargetPoints * EndEffectorPoints.transpose();
//
//	// compute singular value decomposition
//	JacobiSVD<Matrix3f> SVD(W, ComputeFullU | ComputeFullV);
//	Matrix3f U = SVD.matrixU();
//	Matrix3f V = SVD.matrixV();
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
//	Matrix3f S = EndEffectorPoints * TargetPoints.transpose();
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

