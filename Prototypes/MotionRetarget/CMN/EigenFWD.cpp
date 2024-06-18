#include "EigenFWD.hpp"

namespace EigenFWD {
using namespace Eigen;

Quaternionf FromTwoVectors(Vector3f a, Vector3f b) {
	return Quaternionf::FromTwoVectors(a,b);
}

}//EigenFWD
