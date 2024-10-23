#pragma once
#include <crossforge/Graphics/SceneGraph/ISceneGraphNode.h>
#include <crossforge/Math/CForgeMath.h>

namespace CForge {
using namespace Eigen;

/**
 * @brief various (lin algebra) utilities
*/
namespace MRMutil {

Matrix4f buildTransformation(ISceneGraphNode& sgn);
void deconstructMatrix(Matrix4f t, Vector3f* pos, Quaternionf* rot, Vector3f* scale);

}//MRMutil

}//CForge
