#pragma once

#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>

#include <Prototypes/MotionRetarget/UI/Picking.hpp>

#include <crossforge/Math/CForgeMath.h>
#include <crossforge/AssetIO/SAssetIO.h>
#include <crossforge/Graphics/Actors/StaticActor.h>
//#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>

#include <Prototypes/MotionRetarget/CMN/EigenFWD.hpp>

namespace CForge {
using namespace Eigen;
class IKController;
class RenderDevice;

struct JointPickableMesh {
	EigenMesh eigenMesh;
	StaticActor actor;
	StaticActor actorSel; // inverted normals
	BoundingVolume bv;

	JointPickableMesh();
};

class JointPickable : public IPickable {
public:
	void update(Matrix4f sgnT);
	void render(RenderDevice* pRD);

	JointPickable(JointPickableMesh* pMesh, SkeletalAnimationController::SkeletalJoint* pJoint, IKController* pIKC)
	              : m_pMesh(pMesh), m_pJoint(pJoint), m_pIKC(pIKC) {};

	void pckMove(const Matrix4f& trans);

	//TODO(skade) consider global transform of underlying 
	Matrix4f pckTransPickin() {
		return m_transform;
	};
	Matrix4f pckTransGuizmo() {
		return m_transformGuizmo;
	};
	const BoundingVolume& pckBV() {
		return m_pMesh->bv;
	}
	EigenMesh* pckEigenMesh() {
		return &(m_pMesh->eigenMesh);
	};

	void pckSelect() { m_picked = true; };
	void pckDeselect() { m_picked = false; };
private:
	bool m_picked = false;
	Matrix4f m_transform; // global space transform of joint
	Matrix4f m_transformGuizmo; // global space transform of joint
	Matrix4f m_fromPar; // parent joint transform
	Matrix4f m_sgnT; // parent sgn transform of SkeletalActor
	SkeletalAnimationController::SkeletalJoint* m_pJoint;
	IKController* m_pIKC; //TODO(skade) remove refs
	JointPickableMesh* m_pMesh;
};

}//CForge
