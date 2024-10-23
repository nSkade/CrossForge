#pragma once

#include <Prototypes/MotionRetarget/CMN/Picking.hpp>

namespace CForge {
using namespace Eigen;

struct IKTarget : public IPickable {

	IKTarget(std::string name, BoundingVolume bv) {
		this->bv = bv;
		this->name = name;
	}

	/**
	 * @param sgnT Transform Matrix of corresponding actor geo SGN
	*/
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

	BoundingVolume bv;
	Matrix4f m_sgnT = Matrix4f::Identity();
	std::string name;

	Vector3f pos; // global position
};

}//CForge
