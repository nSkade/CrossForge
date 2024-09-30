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
	std::string name;
	T3DMesh<float> mesh;
	SGNGeometry sgn;

	//TODO(skade) find best way to determine if actor is static or skeletal
	std::unique_ptr<IKSkeletalActor> actor;
	bool isStatic = false;
	std::unique_ptr<StaticActor> actorStatic;

	std::unique_ptr<IKController> controller;

	BoundingVolume bv; // mesh bounding volume
	bool visible = true;

	int animIdx = 0;
	int animFrameCurr = 0;
	SkeletalAnimationController::Animation* pAnimCurr = nullptr;

	// Picking bindings
	void pckMove(const Matrix4f& trans);
	Matrix4f pckTransGuizmo(); // used for guizmo update
	Matrix4f pckTransPickin(); // used for picking evaluation
	const BoundingVolume& pckBV();

	void init(SGNTransformation* sgnRoot);
	void applyTransformToMesh(SGNTransformation* sgnRoot);
	void removeArmature(SGNTransformation* sgnRoot);

	void updateRestpose(SGNTransformation* sgnRoot);
};

}//CForge
