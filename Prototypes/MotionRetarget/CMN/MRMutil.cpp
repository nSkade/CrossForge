#include "MRMutil.hpp"

namespace CForge {
using namespace Eigen;

/**
 * @brief various (lin algebra) utilities
*/
namespace MRMutil {

Matrix4f buildTransformation(ISceneGraphNode& sgn) {
	Vector3f p,s; Quaternionf r;
	sgn.buildTansformation(&p,&r,&s);
	return CForgeMath::translationMatrix(p)*CForgeMath::rotationMatrix(r)*CForgeMath::scaleMatrix(s);
}

void deconstructMatrix(Matrix4f t, Vector3f* pos, Quaternionf* rot, Vector3f* scale) {
	*pos = t.block<3,1>(0,3);
	*scale = Vector3f(t.block<3,1>(0,0).norm(),
	                  t.block<3,1>(0,1).norm(),
	                  t.block<3,1>(0,2).norm());
	Matrix3f rotScale;
	rotScale.row(0) = *scale;
	rotScale.row(1) = *scale;
	rotScale.row(2) = *scale;
	*rot = Quaternionf(t.block<3,3>(0,0).cwiseQuotient(rotScale));
}
}//MRMutils
}//CForge
