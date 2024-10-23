#pragma once

/**
 * @brief forwarding Eigen templates to reduce compile time.
*/
namespace EigenFWD {
using namespace Eigen;

/**
 * @brief functions to test Eigen Object for any nan values. Useful in asserts.
*/
template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
return ((x.array() == x.array())).all();
}
template<typename Derived>
inline bool is_nan(const Eigen::QuaternionBase<Derived>& x)
{
return is_nan(x.vec());
}

Quaternionf FromTwoVectors(Vector3f a, Vector3f b);

//TODO(skade) VectorXd for OMR?
MatrixXd JacobiSVDSolve(MatrixXd jac, Vector3d diff);

MatrixXd FullPivLUSolve(MatrixXd jac, Vector3d diff);

}//EigenFWD
