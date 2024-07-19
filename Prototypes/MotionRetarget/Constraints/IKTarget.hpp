#pragma once

#include <Prototypes/MotionRetarget/Animation/Picking.hpp>

namespace CForge {
using namespace Eigen;

//TODO(skade)
struct IKTarget : public IPickable {
	void update(Matrix4f sgnT) {
		m_sgnT = sgnT;
	}

	void pckMove(const Matrix4f& trans) {
		pos = (m_sgnT.inverse() * trans).block<3,1>(0,3);
	}
	Matrix4f pckTrans() {
		Matrix4f r = Matrix4f::Identity();
		r.block<3,1>(0,3) = pos;
		r = m_sgnT * r;
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
	Matrix4f m_sgnT = Matrix4f::Identity();

	Vector3f pos;
	float influence = 1.; //TODO(skade)
};

}//CForge
