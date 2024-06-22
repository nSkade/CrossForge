#pragma once

#include <Prototypes/MotionRetarget/UI/Picking.hpp>

namespace CForge {
using namespace Eigen;

//TODO(skade)
struct IKTarget : public IPickable {
	void pckMove(const Matrix4f& trans) {
		pos = trans.block<3,1>(0,3);
	}
	Matrix4f pckTrans() {
		Matrix4f r = Matrix4f::Identity();
		r.block<3,1>(0,3) = pos;
		return r;
	}
	Matrix4f pckTransGuizmo() {
		return pckTrans();
	}
	Matrix4f pckTransPickin() {
		return pckTrans();
	}
	const BoundingVolume& pckBV() {
		return bv;
	}

	BoundingVolume bv; //TODO(skade)

	Vector3f pos;
	float influence = 1.; //TODO(skade)
};

}//CForge
