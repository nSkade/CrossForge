#include "JacInvSolver.hpp"
#include <Prototypes/MotionRetarget/IK/IKController.hpp>

#include <iostream> //TODO(skade) remove

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
			Eigen::JacobiSVD<MatrixXd> svd(jac, ComputeThinU | ComputeThinV);
			jacI = svd.solve(diff);
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
			Quaternionf rotD = AngleAxisf(dx,Vector3f::UnitX())
			                 * AngleAxisf(dy,Vector3f::UnitY())
			                 * AngleAxisf(dz,Vector3f::UnitZ());
			rotD.normalize();

			//// rotate by dist error
			//AngleAxisf rotDaa = AngleAxisf(rotD);
			//rotDaa.angle() *= .01; // DistError;
			//rotD = Quaternionf(rotDaa);
			//rotD.normalize();

			//TODOm(skade)
			//// get parent global rotatios
			//Quaternionf rotParGlb;
			//if (Chain[j]->Parent != -1)
			//	rotParGlb = pController->m_IKJoints[pController->getBone(Chain[j]->Parent)].rotGlobal;
			//auto rotGlb = pController->m_IKJoints[Chain[j]].rotGlobal;
			//Chain[j]->LocalRotation = (rotD * rotGlb).normalized() * rotParGlb;

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

/**
 * @brief calculates Jacobian matrix of chain regarding influence on end effector,
 *        the jacobian looks as follows:
 * for every joint axis write change on each dim on endeffector
 *              joint 1 x                      joint 1 y                     joint 1 z
 *    eef1 x  | {angle dx, angle dy, angle dz, angle dx, angle dy, angle dz, angle dx, angle dy, angle dz,
 *    eff1 y  | {angle dx, angle dy, angle dz, angle dx, angle dy, angle dz, angle dx, angle dy, angle dz,
 *    eff1 z  | {angle dx, angle dy, angle dz, angle dx, angle dy, angle dz, angle dx, angle dy, angle dz,
 *    eff2 x  v ,...}
 *          all joints
*/
MatrixXd IKSjacInv::calculateJacobianNumerical(std::string segmentName, IKController* pController) {
	std::vector<IKController::SkeletalJoint*>& chain = pController->getIKChain(segmentName)->joints;
	IKTarget* target = pController->getIKChain(segmentName)->target.lock().get();
	uint32_t dim = 3;
	MatrixXd jacobian(dim, chain.size()*dim);

	for (int i = 0; i < chain.size(); ++i) {
		for (int j = 0; j < dim; ++j) {
			Matrix3f jointSpace = chain[i]->LocalRotation.matrix();
			Vector3f toeef = (pController->m_IKJoints[chain[0]].posGlobal - pController->m_IKJoints[chain[i]].posGlobal);
			Vector3f element = jointSpace.col(j).cross(toeef);
			 //TODOm(skade) row oder col?
			// (a,b) * (1) = (a)
			// (c,d) * (0) = (c)
			for (uint32_t k = 0; k < 3; ++k) {
				jacobian(k,i*dim+j) = element(k);
			}
		}
	}
	return jacobian;
}

}//CForge
