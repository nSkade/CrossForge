#pragma once

#include <crossforge/Graphics/Controller/SkeletalAnimationController.h>

#include <Prototypes/MotionRetarget/CMN/Picking.hpp>

#include <crossforge/Math/CForgeMath.h>
#include <crossforge/Graphics/Actors/StaticActor.h>
//#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>

#include <Prototypes/MotionRetarget/CMN/EigenFWD.hpp>

namespace CForge {
using namespace Eigen;
class IKController;
class RenderDevice;

struct JointPickableMesh {
	EigenMesh eigenMesh;
	BoundingVolume bv;
	T3DMesh<float> mesh;
	JointPickableMesh();
};

class JointPickable : public IPickable {
public:
	void init() {
		//TODOfff(skade) mesh data for every joint in vram, inefficient
		actor.init(&m_pJPMesh->mesh);
		actorSel.init(&m_pJPMesh->mesh);
		actorSel.material(0)->color(Vector4f(227./255,142./255,48./255,1.));
	};
	void update(Matrix4f sgnT);
	void render(RenderDevice* pRD);

	JointPickable(JointPickableMesh* pMesh, SkeletalAnimationController::SkeletalJoint* pJoint, IKController* pIKC)
	              : m_pJPMesh(pMesh), m_pJoint(pJoint), m_pIKC(pIKC) {};

	void pckMove(const Matrix4f& trans);

	Matrix4f pckTransPickin() {
		return m_transform;
	};
	Matrix4f pckTransGuizmo() {
		return m_transformGuizmo;
	};
	const BoundingVolume& pckBV() {
		return m_pJPMesh->bv;
	}
	EigenMesh* pckEigenMesh() {
		return &(m_pJPMesh->eigenMesh);
	};

	void pckSelect() { m_highlight = true; };
	void pckDeselect() { m_highlight = false; };
	JointPickableMesh* m_pJPMesh;
	bool m_highlight = false;
	
	// visualizer
	bool highlightBehind = true;
	void setOpacity(float opacity);
	float getOpacity();

	void restoreColor() { colorSelect = colorSelect0; };

	Vector4f colorSelect = Vector4f(227./255,142./255,48./255,1.);
	Vector4f colorSelect0 = Vector4f(227./255,142./255,48./255,1.); // standard color for reset
	SkeletalAnimationController::SkeletalJoint* m_pJoint;
private:
	Matrix4f m_transform; // global space transform of joint
	Matrix4f m_transformGuizmo; // global space transform of joint
	Matrix4f m_fromPar; // parent joint transform
	Matrix4f m_sgnT; // parent sgn transform of SkeletalActor
	IKController* m_pIKC; //TODOfff(skade) smartptr

	// visualizer
	StaticActor actor;
	StaticActor actorSel; // inverted normals
};

}//CForge
