#pragma once

#include "Picking.hpp"
#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>
#include <crossforge/Math/CForgeMath.h>

namespace CForge {

struct JointPickable : public IPickable {
	void pckMove(const Matrix4f& trans) {
		if (!pJoint) return;
		pJoint->LocalPosition = trans.block<3,1>(3,0);
		pJoint->LocalRotation = Quaternionf(trans.block<3,3>(0,0));
		pJoint->LocalScale = Vector3f(trans.coeff(0,0),trans.coeff(1,1),trans.coeff(2,2));
	};

	Matrix4f pckTrans() {
		if (!pJoint) return Matrix4f::Identity();
		auto s = CForgeMath::scaleMatrix(pJoint->LocalScale);
		auto r = CForgeMath::rotationMatrix(pJoint->LocalRotation);
		auto t = CForgeMath::translationMatrix(pJoint->LocalPosition);

		return t*r*s;
	};
	const BoundingVolume& pckBV() {
		return bv;
	}
	BoundingVolume bv; //TODO(skade)
	SkeletalAnimationController::SkeletalJoint* pJoint;
};

}//CForge
