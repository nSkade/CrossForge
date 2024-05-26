#include "CCDSolver.hpp"
#include <Prototypes/MotionRetarget/IKController.h>

namespace CForge {

void IKSolverCCD::ikCCD(const std::string segmentName) {
	std::vector<SkeletalJoint*>& Chain = m_JointChains.at(segmentName).joints;
	EndEffectorData* pEffData = m_IKJoints[Chain.front()]->pEndEffectorData;
	Matrix3Xf LastEndEffectorPoints;

	for (int32_t i = 0; i < m_MaxIterations; ++i) {
		LastEndEffectorPoints = pEffData->EEPosGlobal;

		// check for termination -> condition: end-effector has reached the targets position and orientation
		Matrix3Xf EffectorTargetDiff = pEffData->TargetPosGlobal - pEffData->EEPosGlobal;
		float DistError = EffectorTargetDiff.cwiseProduct(EffectorTargetDiff).sum() / float(EffectorTargetDiff.cols());
		if (DistError < m_thresholdDist)
			return;

		//TODO(skade) implement Forward CCD
		// Backward CCD
		//for (int32_t k = Chain.size()-1; k >= 0; --k) {
		for (int32_t k = 0; k < Chain.size(); ++k) {
			// start at base joint
			SkeletalJoint* pCurrent = Chain[k];
			IKJoint* pCurrentIK = m_IKJoints[pCurrent];

			// calculate rotation axis
			// joint position to end effector
			Eigen::Vector3f jpToEE = pEffData->EEPosGlobal.col(0)-pCurrentIK->GlobalPosition;
			// joint position to target position
			Eigen::Vector3f jpToTar = pEffData->TargetPosGlobal.col(0)-pCurrentIK->GlobalPosition;

			// rotation angle
			float theta = std::acos(jpToEE.dot(jpToTar));
			if (std::abs(theta) < FLT_EPSILON)
				continue;

			// vector we rotate around
			Eigen::Vector3f rotVec = jpToEE.cross(jpToTar);
			rotVec.normalize();

			Quaternionf GlobalIncrement = Quaternionf(Eigen::AngleAxis(theta,rotVec));

			Quaternionf NewGlobalRotation = GlobalIncrement * pCurrentIK->GlobalRotation;
			
			// transform new global rotation to new local rotation
			Quaternionf NewLocalRotation;
			if (pCurrent == m_pRoot)
				NewLocalRotation = NewGlobalRotation;
			else
				NewLocalRotation = m_IKJoints[m_Joints[pCurrent->Parent]]->GlobalRotation.conjugate() * NewGlobalRotation;

			NewLocalRotation.normalize();

			// constrain new local rotation if joint is not unconstrained
			if (pCurrentIK->pLimits != nullptr)
				NewLocalRotation = pCurrentIK->pLimits->constrain(NewLocalRotation);

			// apply new local rotation to joint
			pCurrent->LocalRotation = NewLocalRotation;

			// update kinematic chain
			forwardKinematics(pCurrent);
		}//for[each joint in chain]

		Matrix3Xf EffectorPosDiff = pEffData->EEPosGlobal - LastEndEffectorPoints;
		float PosChangeError = EffectorPosDiff.cwiseProduct(EffectorPosDiff).sum() / float(EffectorPosDiff.cols());
		if (PosChangeError < m_thresholdPosChange)
			return;
	}//for[m_MaxIterations]
}//ikCCD

Quaternionf IKSolverCCD::computeUnconstrainedGlobalRotation(IKJoint* pJoint, IKController::EndEffectorData* pEffData) {

	//TODO(skade) doesnt rotate joint to target, but takes whole chain into considertaiton

	//TODO: combine points of multiple end effectors and targets into 2 point clouds to compute CCD rotation for multiple end effectors?
	Matrix3Xf EndEffectorPoints = pEffData->EEPosGlobal.colwise() - pJoint->GlobalPosition; // current local joint position
	Matrix3Xf TargetPoints = pEffData->TargetPosGlobal.colwise() - pJoint->GlobalPosition;  // target local joint position

#if 0
	// compute matrix W
	Matrix3f W = TargetPoints * EndEffectorPoints.transpose();

	// compute singular value decomposition
	JacobiSVD<Matrix3f> SVD(W, ComputeFullU | ComputeFullV);
	Matrix3f U = SVD.matrixU();
	Matrix3f V = SVD.matrixV();

	// compute rotation
	Matrix3f R = U * V.transpose();
	Quaternionf GlobalRotation(R);
	GlobalRotation.normalize();
#else
	// compute rotation using quaternion characteristic polynomial from: "Closed-form solution of absolute orientation using unit quaternions." - Berthold K. P. Horn, 1987
	// https://web.stanford.edu/class/cs273/refs/Absolute-OPT.pdf

	//
	//			0	1	2
	//		0	Sxx	Sxy	Sxz
	// S =	1	Syx	Syy	Syz
	//		2	Szx	Szy	Szz
	//		
	Matrix3f S = EndEffectorPoints * TargetPoints.transpose();

	//
	// N = 
	//		Sxx + Syy + Szz		Syz - Szy			 Szx - Sxz			 Sxy - Syx
	//		Syz - Szy			Sxx - Syy - Szz		 Sxy + Syx			 Szx + Sxz
	//		Szx - Sxz			Sxy + Syx			-Sxx + Syy - Szz	 Syz + Szy
	//		Sxy - Syx			Szx + Sxz			 Syz + Szy			-Sxx - Syy + Szz
	//
	Matrix4f N = Matrix4f::Zero();

	N(0, 0) = S(0, 0) + S(1, 1) + S(2, 2);  //  Sxx + Syy + Szz
	N(0, 1) = S(1, 2) - S(2, 1);            //  Syz - Szy
	N(0, 2) = S(2, 0) - S(0, 2);            //  Szx - Sxz
	N(0, 3) = S(0, 1) - S(1, 0);            //  Sxy - Syx
	
	N(1, 0) = N(0, 1);                      //  Syz - Szy
	N(1, 1) = S(0, 0) - S(1, 1) - S(2, 2);  //  Sxx - Syy - Szz
	N(1, 2) = S(0, 1) + S(1, 0);            //  Sxy + Syx
	N(1, 3) = S(2, 0) + S(0, 2);            //  Szx + Sxz
	
	N(2, 0) = N(0, 2);                      //  Szx - Sxz
	N(2, 1) = N(1, 2);                      //  Sxy + Syx
	N(2, 2) = -S(0, 0) + S(1, 1) - S(2, 2); // -Sxx + Syy - Szz
	N(2, 3) = S(1, 2) + S(2, 1);            //  Syz + Szy
	
	N(3, 0) = N(0, 3);                      //  Sxy - Syx
	N(3, 1) = N(1, 3);                      //  Szx + Sxz
	N(3, 2) = N(2, 3);                      //  Syz + Szy
	N(3, 3) = -S(0, 0) - S(1, 1) + S(2, 2); // -Sxx - Syy + Szz
	
	Eigen::SelfAdjointEigenSolver<Matrix4f> Solver(4);
	Solver.compute(N);
	Vector4f BiggestEVec = Solver.eigenvectors().col(3); // last column of eigenvectors() matrix contains eigenvector of largest eigenvalue;
	                                                     // that's supposed to equal the desired rotation quaternion
	Quaternionf GlobalRotation = Quaternionf(BiggestEVec(0), BiggestEVec(1), BiggestEVec(2), BiggestEVec(3));
#endif
	return GlobalRotation;
}//computeUnconstrainedGlobalRotation

}//CForge

