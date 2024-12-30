#include "CharEntity.hpp"

#include "CMN/MRMutil.hpp"

#include <fstream>#
#include <crossforge/Core/SLogger.h>

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
		sgn.m_enableCulling = false;
		isStatic = false;
	}
	else {
		actorStatic = std::make_unique<StaticActor>();
		actorStatic->init(&mesh);

		sgn.init(sgnRoot,actorStatic.get());
		sgn.m_enableCulling = false;
		isStatic = true;
	}

	// set bounding volume
	mesh.computeAxisAlignedBoundingBox();
	Box aabb = mesh.aabb();
	bv.init(aabb);

	// load armature if one is bound
	try {
		parseArmature();
	}
	catch (...) {
		SLogger::log("error occured curing parsing ik armature, deleting armature");
		armatureInfo.limbs.clear();
	}
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
		ibpPos = ibpPos.cwiseProduct(scale); //TODOff(skade) seems correct, but I am not sure
		mesh.getBone(i)->InvBindPoseMatrix.block<3,1>(0,3) = ibpPos;
	}

	for (uint32_t i = 0; i < mesh.skeletalAnimationCount(); ++i) {
		int32_t rootBoneID = mesh.rootBone()->ID;
		auto anim = mesh.getSkeletalAnimation(i);
		auto kfr = anim->Keyframes[rootBoneID];
		
		for (uint32_t k = 0; k < kfr->Rotations.size(); ++k) {
			kfr->Rotations[k] = rot * kfr->Rotations[k];
		}
		for (uint32_t j = 0; j < anim->Keyframes.size(); ++j) {
			auto kf = anim->Keyframes[j];
			for (uint32_t k = 0; k < kf->Positions.size(); ++k) {
				kf->Positions[k] = scale.cwiseProduct(kf->Positions[k]);
			}
		}
		for (uint32_t k = 0; k < kfr->Positions.size(); ++k) {
			kfr->Positions[k] = pos + rot*kfr->Positions[k];
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


void CharEntity::importArmature(std::filesystem::path path) {
	armatureInfo.limbs.clear();
	std::ifstream f(path);
	const nlohmann::json configData = nlohmann::json::parse(f);
	auto StructureData = configData.at("SkeletonStructure");

	for (auto it : StructureData.items()) {
		if(it.value().contains("Root") && it.value().contains("EndEffector"))
			armatureInfo.limbs.push_back({it.key(),
			                              it.value().at("Root").get<std::string>(),
			                              it.value().at("EndEffector").get<std::string>()});
	}
}

void CharEntity::exportArmature(std::filesystem::path path) {
	nlohmann::json configData;

	std::ofstream f(path);
	if (f.is_open()) {
		for (auto l : armatureInfo.limbs) {
			nlohmann::json limbData = { {"Root", l.startJoint}, {"EndEffector", l.endJoint} };
			configData["SkeletonStructure"][l.name] = limbData;
		}
		f << std::setw(4) << configData << std::endl;
	}
}

void CharEntity::autoCreateTargets() {
	if (auto ctrl = controller.get()) {
		ctrl->initTargetPoints();
	}
}

void CharEntity::autoCreateArmature() {

	if (auto ctrl = controller.get()) {
		std::map<std::string,std::vector<SkeletalAnimationController::SkeletalJoint*>> ikc;

		std::function<void(SkeletalAnimationController::SkeletalJoint* pJoint, std::string name)> propagate;
		propagate = [&](SkeletalAnimationController::SkeletalJoint* pJoint, std::string name) {
			
			// end current chain and add all childs as new chains
			int cc = pJoint->Children.size();
			// add joint to chain
			if (name != "")
				ikc[name].insert(ikc[name].begin(),pJoint);
			if (cc > 1) {
				for (uint32_t i = 0; i < cc; ++i) {
					auto c =ctrl->getBone(pJoint->Children[i]);
					propagate(c,c->Name);
				}
			} else if (cc == 1) { // add to current chain
				auto c =ctrl->getBone(pJoint->Children[0]);
				propagate(c,name);
			} //else //(cc == 0)  // end effector nothing to do
		};
		propagate(ctrl->getRoot(),"");

		ctrl->m_ikArmature.m_jointChains.clear();
		for (auto& [k,v] : ikc) {
			IKChain nc;
			nc.name = k;
			nc.joints = v;
			ctrl->m_ikArmature.m_jointChains.emplace_back(std::move(nc));
		}
	}
}

}//CForge
