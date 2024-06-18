#pragma once

#include "Picking.hpp"
#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>
#include <crossforge/Math/CForgeMath.h>

#include <crossforge/AssetIO/SAssetIO.h>
#include <crossforge/Graphics/Actors/StaticActor.h>
//#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>

namespace CForge {

struct JointPickableMesh {
	EigenMesh eigenMesh;
	StaticActor actor;
	BoundingVolume bv;

	JointPickableMesh() { 
		T3DMesh<float> M;
		SAssetIO::load("MyAssets/ccd-ik/joint.obj", &M); //TODO(skade) use primitive shape factory instead
		eigenMesh = EigenMesh(M);
		actor.init(&M);
		M.computeAxisAlignedBoundingBox();
		bv.init(M.aabb());
	};
};

struct JointPickable : public IPickable {
	JointPickable(JointPickableMesh* pMesh, SkeletalAnimationController::SkeletalJoint* pJoint) {
		this->pMesh = pMesh;
		this->pJoint = pJoint;
	};

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
		return pMesh->bv;
	}
	EigenMesh* pckEigenMesh() {
		return &(pMesh->eigenMesh);
	};

	SkeletalAnimationController::SkeletalJoint* pJoint;
	JointPickableMesh* pMesh;
};

}//CForge
