#include "JacInvSolver.hpp"
#include <Prototypes/MotionRetarget/IK/IKController.hpp>

//#include <iostream> // dbg

#include <Prototypes/MotionRetarget/CMN/EigenFWD.hpp>

namespace CForge {
using namespace Eigen;

void IKSjacInv::solve(std::string segmentName, IKController* pController) {
	std::vector<IKController::SkeletalJoint*>& Chain = pController->getIKChain(segmentName)->joints;
	IKTarget* target = pController->getIKChain(segmentName)->target.lock().get();
	if (!target)
		return;

	Vector3f lastEFpos;
	IKJoint& eef = pController->m_IKJoints[Chain[0]];

	// target reachable?
	float totalChainLength = 0.;
	for (uint32_t i=1;i<Chain.size();++i) {
		float len = (pController->m_IKJoints[Chain[i-1]].posGlobal - pController->m_IKJoints[Chain[i]].posGlobal).norm();
		totalChainLength += len;
	}

	Vector3f rootPos = pController->m_IKJoints[Chain.back()].posGlobal;
	Vector3f rootToTar = target->pos - rootPos;
	float rootToTarLen = rootToTar.norm();

	Vector3f targetPos = target->pos;
	if (totalChainLength < rootToTarLen) {
		// project target to reachable length
		targetPos = totalChainLength * rootToTar.normalized() + rootPos;
	}

	for (uint32_t i = 0; i < m_MaxIterations; ++i) {
		lastEFpos = eef.posGlobal;
		// check for termination -> condition: end-effector has reached the targets position and orientation
		float DistError = (lastEFpos-targetPos).norm();
		if (DistError <= m_thresholdDist)
			return;

		MatrixXd jac = calculateJacobianNumerical(segmentName,pController);
		VectorXd diff = (targetPos-eef.posGlobal).cast<double>();

		MatrixXd jacI;
		switch (m_type)
		{
		case TRANSPOSE: { // transpose
			jacI = jac.transpose() * diff;
		}
			break;
		case SVD: { // svd
			// not stable when multiple joints align
			jacI = EigenFWD::JacobiSVDSolve(jac,diff);
			//TODOfff(skade)
			//jacI = EigenFWD::FullPivLUSolve(jac,diff);
		}
			break;
		default:
		case DLS: { // damped least squares
			auto dls = DampedLeastSquare(jac);
			jacI = dls * diff;
		}
			break;
		}

		// print matrix dim for testing
		//std::cout << "jac" << std::endl;
		//std::cout << jac.rows() << " x " << jac.cols() << std::endl;
		////std::cout << "svd" << std::endl;
		////std::cout << svd.rows() << " x " << svd.cols() << std::endl;
		////std::cout << "diff" << std::endl;
		////std::cout << diff.rows() << " x " << diff.cols() << std::endl;
		//std::cout << "jacI" << std::endl;
		//std::cout << jacI.rows() << " x " << jacI.cols() << std::endl;
		//std::cout << jacI << std::endl;

		// apply solution rotations to joint rotations
		for (uint32_t j = 0; j < Chain.size(); ++j) {
			float dx = jacI(j*3+0);
			float dy = jacI(j*3+1);
			float dz = jacI(j*3+2);

			// global delta
			Quaternionf rotD = Quaternionf(AngleAxisf(dx,Vector3f::UnitX()))
			                 * Quaternionf(AngleAxisf(dy,Vector3f::UnitY()))
			                 * Quaternionf(AngleAxisf(dz,Vector3f::UnitZ()));
			rotD.normalize();

			// not usable with svd
			//TODOff(skade) add delta as parameter
			//{// rotate by dist error
			//	AngleAxisf rotDaa = AngleAxisf(rotD);
			//	rotDaa.angle() *= (DistError + 1.)*10.; //.1; //TODOff(skade)
			//	rotD = Quaternionf(rotDaa);
			//	rotD.normalize();
			//}

			Chain[j]->LocalRotation = rotD * Chain[j]->LocalRotation;
			Chain[j]->LocalRotation.normalize();
		}
		
		pController->forwardKinematics();
	
		float PosChangeError = (eef.posGlobal - lastEFpos).norm();
		if (PosChangeError < m_thresholdPosChange)
			return;
	}
}

MatrixXd IKSjacInv::DampedLeastSquare(MatrixXd jac) {
	float damping = m_dlsDamping;
	int eefCount = 1; //TODO(skade) multiple endeff
	MatrixXd jt = jac.transpose();
	MatrixXd temp = jac * jt + (damping*damping) * MatrixXd::Identity(eefCount*3,eefCount*3);
	MatrixXd ret = jt * temp.inverse();
	return ret;
}

//TODO(skade) implement for multiple endeffectors for use with OMR
MatrixXd IKSjacInv::calculateJacobianNumerical(std::string segmentName, IKController* pController) {
	std::vector<IKController::SkeletalJoint*>& chain = pController->getIKChain(segmentName)->joints;
	IKTarget* target = pController->getIKChain(segmentName)->target.lock().get();
	uint32_t dim = 3;
	MatrixXd jacobian(dim, chain.size()*dim);

	for (int i = 0; i < chain.size(); ++i) {
		for (int j = 0; j < dim; ++j) {
		
			// xyz endeff position rate change on angle change
			Vector3f element = Vector3f::Zero();

#if 0		// forwardKinematics method, naive slower
			Vector3f eefPos = pController->m_IKJoints[chain[0]].posGlobal;
			auto origRot = chain[i]->LocalRotation;

			// rotate in dim by delta
			float delta = 0.01f; //TODO(skade) j -> eff norm instead of delta?
			switch (j)
			{
			case 0:
				chain[i]->LocalRotation = Quaternionf(AngleAxisf(delta,Vector3f::UnitX())) * chain[i]->LocalRotation;
			case 1:
				chain[i]->LocalRotation = Quaternionf(AngleAxisf(delta,Vector3f::UnitY())) * chain[i]->LocalRotation;
			case 2:
				chain[i]->LocalRotation = Quaternionf(AngleAxisf(delta,Vector3f::UnitZ())) * chain[i]->LocalRotation;
			default:
				break;
			}
			
			pController->forwardKinematics();
			element = pController->m_IKJoints[chain[0]].posGlobal - eefPos;
			element.normalize();

			// undo rotation
			chain[i]->LocalRotation = origRot;
			pController->forwardKinematics();

#else		// cross product tangent calculation, more performant,
			// this exploits the fact that all children of a joint are just a rigid structure on rotation
			
			// rotation has higher effect, depending on distance
			Vector3f toeef = pController->m_IKJoints[chain[0]].posGlobal - pController->m_IKJoints[chain[i]].posGlobal;

			// joint space defined by parent transformation
			Matrix3f jointSpace = Matrix3f::Identity();
			if (chain[i]->Parent != -1)
				jointSpace = pController->m_IKJoints[pController->getBone(chain[i]->Parent)].rotGlobal.matrix();

			// cross product to get the tangent vector, direction in which the point moves on rotation
			// of the corresponding axis
			element = jointSpace.col(j).cross(toeef);
#endif
			for (uint32_t k = 0; k < 3; ++k) {
				jacobian(k,i*dim+j) = element(k);
			}
		}
	}
	return jacobian;
}

}//CForge
