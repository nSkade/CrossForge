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

	// common
	std::string name;
	T3DMesh<float> mesh;
	SGNGeometry sgn;
	void init(SGNTransformation* sgnRoot);

	BoundingVolume bv; // mesh bounding volume
	bool visible = true;

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
