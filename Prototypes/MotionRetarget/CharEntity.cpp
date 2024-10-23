#include "CharEntity.hpp"

#include "CMN/MRMutil.hpp"

namespace CForge {

void CharEntity::pckMove(const Matrix4f& trans) {
	Vector3f p,s; Quaternionf r;
	MRMutil::deconstructMatrix(trans,&p,&r,&s);
	sgn.position(p);
	sgn.rotation(r);
	sgn.scale(s);
}
Matrix4f CharEntity::pckTransGuizmo() {
	return MRMutil::buildTransformation(sgn);
} // used for guizmo update
Matrix4f CharEntity::pckTransPickin() {
	return MRMutil::buildTransformation(sgn);
} // used for picking evaluation
const BoundingVolume& CharEntity::pckBV() {
	return bv; 
}

void CharEntity::init(SGNTransformation* sgnRoot) {
	mesh.computePerVertexNormals(); //TODOff(skade) remove
	if (mesh.rootBone()) {
		controller = std::make_unique<IKController>();
		controller->init(&mesh);

		for (uint32_t i = 0; i < mesh.skeletalAnimationCount(); ++i) {
			if (mesh.getSkeletalAnimation(i)->Keyframes[0]->ID != -1)
				controller->addAnimationData(mesh.getSkeletalAnimation(i));
		}
		actor = std::make_unique<IKSkeletalActor>();
		actor->init(&mesh,controller.get());

		//TODOff(skade) into function?
		sgn.init(sgnRoot,actor.get());
	}
	else {
		actorStatic = std::make_unique<StaticActor>();
		actorStatic->init(&mesh);
		isStatic = true;

		sgn.init(sgnRoot,actorStatic.get());
	}

	// set bounding volume
	mesh.computeAxisAlignedBoundingBox();
	Box aabb = mesh.aabb();
	bv.init(aabb);
}

void CharEntity::removeArmature(SGNTransformation* sgnRoot) {
	mesh.clearSkeleton();
	mesh.clearSkeletalAnimations();
	controller.reset();
	actor.reset();
	init(sgnRoot);
}

void CharEntity::applyTransformToMesh(SGNTransformation* sgnRoot) {
	Matrix4f t = MRMutil::buildTransformation(sgn);

	// apply transfrom on all vertices
	for (uint32_t i = 0; i < mesh.vertexCount(); ++i) {
		Vector4f v;
		v.block<3,1>(0,0) = mesh.vertex(i);
		v.w() = 1.;
		v = t*v;
		mesh.vertex(i) = v.block<3,1>(0,0);
	}

	//TODOfff(skade) morph support
	//mesh.addMorphTarget

	Vector3f pos, scale;
	Quaternionf rot;
	MRMutil::deconstructMatrix(t,&pos,&rot,&scale);

	for (uint32_t i = 0; i < mesh.boneCount(); ++i) {
		// apply scale to pos only
		Matrix4f t2 = CForgeMath::translationMatrix(pos);
		t2.block<3,3>(0,0) = rot.toRotationMatrix();

		mesh.getBone(i)->InvBindPoseMatrix = mesh.getBone(i)->InvBindPoseMatrix * t2.inverse();
		Vector3f ibpPos = mesh.getBone(i)->InvBindPoseMatrix.block<3,1>(0,3);
		ibpPos = ibpPos.cwiseProduct(scale); //TODO(skade) correct?
		mesh.getBone(i)->InvBindPoseMatrix.block<3,1>(0,3) = ibpPos;
	}

	//TODO(skade) animations
	for (uint32_t i = 0; i < mesh.skeletalAnimationCount(); ++i) {
		int32_t rootBoneID = mesh.rootBone()->ID;
		auto anim = mesh.getSkeletalAnimation(i);
		auto kf = anim->Keyframes[rootBoneID];
		
		//TODO(skade) scale not ideal? apply all scale to pos instead?
		for (uint32_t k = 0; k < kf->Scalings.size(); ++k) {
			kf->Scalings[k] = scale.cwiseProduct(kf->Scalings[k]);
		}

		for (uint32_t k = 0; k < kf->Rotations.size(); ++k) {
			kf->Rotations[k] = rot * kf->Rotations[k];
		}
		//TODOf(skade) correct?
		for (uint32_t k = 0; k < kf->Positions.size(); ++k) {
			kf->Positions[k] = scale.cwiseProduct(kf->Positions[k]);
			kf->Positions[k] = pos + rot * kf->Positions[k];
		}
	}

	init(sgnRoot);
}

void CharEntity::updateRestpose(SGNTransformation* sgnRoot) {
	if(!actor)
		return;

	// apply current pose to mesh data
	for (uint32_t i=0;i<mesh.vertexCount();++i) {
		mesh.vertex(i) = actor->transformVertex(i);
	}

	// forwardKinematics to get updated global pos and rot in m_IKJoints
	controller->forwardKinematics(controller->getRoot());

	for (uint32_t i=0;i<mesh.boneCount();++i) {
		auto* b = mesh.getBone(i);

		//TODOff(skade) bad, assumes mesh idx == controller idx
		IKJoint ikj = controller->m_IKJoints[controller->getBone(i)];

		// get current global position and rotation
		Vector3f pos = ikj.posGlobal;
		Quaternionf rot = ikj.rotGlobal;

		Matrix4f bindPose = Matrix4f::Identity();
		bindPose.block<3,1>(0,3) = pos;
		bindPose.block<3,3>(0,0) = rot.toRotationMatrix();
		b->InvBindPoseMatrix = bindPose.inverse();
	}

	//TODOf(skade) update animations

	init(sgnRoot);
}

}//CForge
