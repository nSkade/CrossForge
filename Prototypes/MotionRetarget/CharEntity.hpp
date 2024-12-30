#pragma once

#include "CMN/Picking.hpp"
#include "IK/IKSkeletalActor.hpp"
#include "IK/IKController.hpp"

#include <crossforge/AssetIO/T3DMesh.hpp>
#include <crossforge/Graphics/SceneGraph/SGNGeometry.h>
#include <crossforge/Graphics/SceneGraph/SGNTransformation.h>

namespace CForge {
using namespace Eigen;

/**
 * @brief compacts info regarding single character
*/
struct CharEntity : public IPickable {
	// actor
	//TODOff(skade) find best way to determine if actor is static or skeletal
	//TODOff(skade) add skeletal actor without mesh data
	bool isStatic = false;
	std::unique_ptr<IKSkeletalActor> actor;
	std::unique_ptr<StaticActor> actorStatic;

	// animation only
	std::unique_ptr<IKController> controller;
	int animIdx = 0;
	int animFrameCurr = 0;
	SkeletalAnimationController::Animation* pAnimCurr = nullptr;

	bool m_IKCupdate = false;
	bool m_IKCupdateSingle = false;
	int m_animAutoplay = false;

	// common
	std::string name;
	T3DMesh<float> mesh;
	SGNGeometry sgn;
	void init(SGNTransformation* sgnRoot);

	BoundingVolume bv; // mesh bounding volume
	float visibility = 1.;

	//TODO(skade) cesman test,
	std::string armatureFilepath = "";

	// exportable armature
	//TODO(skade) function to get ConfigData from existing m_ikArmature chains
	struct ArmatureInfo {
		struct Chain {
			std::string name;
			std::string startJoint; // root of chain
			std::string endJoint;   // end effector
		};
		std::vector<Chain> limbs;
		//TODO(skade) joint limits
	} armatureInfo;
	void parseArmature() {
		if (controller) {
			for (ArmatureInfo::Chain c : armatureInfo.limbs)
				controller->buildKinematicChain(c.name,c.startJoint,c.endJoint);
		}
	}
	void extractArmature() {
		armatureInfo.limbs.clear();
		if (controller) {
			for (auto& jc : controller->m_ikArmature.m_jointChains)
				armatureInfo.limbs.push_back({jc.name,jc.joints.back()->Name,jc.joints[0]->Name});
		}
	}
	void importArmature(std::filesystem::path path);
	void exportArmature(std::filesystem::path path);

	void autoCreateTargets();
	void autoCreateArmature();

	// Picking bindings
	void pckMove(const Matrix4f& trans);
	Matrix4f pckTransGuizmo(); // used for guizmo update
	Matrix4f pckTransPickin(); // used for picking evaluation
	const BoundingVolume& pckBV();

	// tools //TODOff(skade) move outside
	void applyTransformToMesh(SGNTransformation* sgnRoot);
	void removeArmature(SGNTransformation* sgnRoot);
	void updateRestpose(SGNTransformation* sgnRoot);
};

}//CForge
