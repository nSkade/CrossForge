#include "EigenFWD.hpp"

namespace EigenFWD {
using namespace Eigen;

Quaternionf FromTwoVectors(Vector3f a, Vector3f b) {
	return Quaternionf::FromTwoVectors(a,b);
}

MatrixXd JacobiSVDSolve(MatrixXd jac, Vector3d diff) {
	Eigen::JacobiSVD<MatrixXd> svd(jac, ComputeThinU | ComputeThinV);
	return svd.solve(diff);
}

MatrixXd FullPivLUSolve(MatrixXd jac, Vector3d diff) {
	Eigen::FullPivLU<MatrixXd> fplu(jac);
	return fplu.solve(diff);
}

}//EigenFWD
