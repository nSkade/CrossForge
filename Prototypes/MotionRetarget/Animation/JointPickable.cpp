#include "JointPickable.hpp"

#include <Prototypes/MotionRetarget/IK/IKController.hpp>
#include <crossforge/Graphics/RenderDevice.h>
#include <crossforge/AssetIO/SAssetIO.h>

//TODOfff(skade) use primitiveShapeFectory to generate joint mesh
//#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>

#include <glad/glad.h>

namespace CForge {
using namespace Eigen;

JointPickableMesh::JointPickableMesh() {
	SAssetIO::load("Assets/joint.obj", &mesh); //TODOfff(skade) use primitive shape factory instead
	eigenMesh = EigenMesh(mesh);
	mesh.getMaterial(0)->Color.w() = .75;
	mesh.computeAxisAlignedBoundingBox();
	bv.init(mesh.aabb());
}

void JointPickable::setOpacity(float opacity) {
	Vector4f color = actor.material(0)->color();
	color.w() = opacity;
	actor.material(0)->color(color);
}
float JointPickable::getOpacity() {
	return actor.material(0)->color().w();
}

void JointPickable::update(Matrix4f sgnT) {
	m_sgnT = sgnT;

	const Matrix4f R = CForgeMath::rotationMatrix(m_pJoint->LocalRotation);
	const Matrix4f T = CForgeMath::translationMatrix(m_pJoint->LocalPosition);
	const Matrix4f S = CForgeMath::scaleMatrix(m_pJoint->LocalScale);
	Matrix4f JointTransform = T * R * S;

	m_fromPar = Matrix4f::Identity();
	if (m_pJoint->Parent != -1)
		m_fromPar = m_pIKC->getBone(m_pJoint->Parent)->SkinningMatrix
		          * m_pIKC->getBone(m_pJoint->Parent)->OffsetMatrix.inverse();
	Matrix4f GlobalTransform = m_fromPar * JointTransform;

	Vector3f BoneVec; // vector to next bone
	if (m_pJoint->Children.size() > 0)
		BoneVec = m_pIKC->getBone(m_pJoint->Children[0])->LocalPosition;
	else
		BoneVec = m_pJoint->LocalPosition;
	float Length = BoneVec.norm(); // length to next bone
	
	Quaternionf LR = EigenFWD::FromTwoVectors(Vector3f::UnitX(), BoneVec.normalized()); // obj Joint points to +x axis

	m_transform = m_sgnT * GlobalTransform * CForgeMath::rotationMatrix(LR) * CForgeMath::scaleMatrix(Vector3f(Length,Length,Length));
	if (!m_highlight)
		m_transformGuizmo = m_sgnT * GlobalTransform;
}
void JointPickable::pckMove(const Matrix4f& trans) {
	if (!m_pJoint) return;

	// extract new rotations from matrix
	//TODOfff(skade) unify place with init rest pose
	            //function that computes pos scale rot from mat4
	// 
	Matrix4f t = m_fromPar.inverse() * m_sgnT.inverse() * trans; //TODOf(skade) order correct?
	Vector3f pos = t.block<3,1>(0,3);
	Vector3f scale = Vector3f(t.block<3,1>(0,0).norm(),
	                          t.block<3,1>(0,1).norm(),
	                          t.block<3,1>(0,2).norm());
	Matrix3f rotScale;
	rotScale.row(0) = scale;
	rotScale.row(1) = scale;
	rotScale.row(2) = scale;
	Quaternionf rot = Quaternionf(t.block<3,3>(0,0).cwiseQuotient(rotScale));

	m_pJoint->LocalPosition = pos;
	m_pJoint->LocalScale = scale; //m_pJoint->LocalScale;
	m_pJoint->LocalRotation = rot; //m_pJoint->LocalRotation;
	m_pJoint->LocalRotation.normalize();
};
void JointPickable::render(RenderDevice* pRD) {
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	if (highlightBehind || !m_highlight) {
		pRD->modelUBO()->modelMatrix(m_transform);
		actor.render(pRD,Eigen::Quaternionf::Identity(),Eigen::Vector3f(),Eigen::Vector3f(1.f,1.f,1.f));
	}
	if (m_highlight) {
		glCullFace(GL_FRONT);
		pRD->modelUBO()->modelMatrix(m_transform * CForgeMath::scaleMatrix(Vector3f(1.,1.3,1.3)));
		actorSel.material(0)->color(colorSelect);
		actorSel.render(pRD, Eigen::Quaternionf::Identity(),Eigen::Vector3f(),Eigen::Vector3f(1.f,1.f,1.f));
		glCullFace(GL_BACK);
	}
	glDisable(GL_BLEND);
}

}//CForge
