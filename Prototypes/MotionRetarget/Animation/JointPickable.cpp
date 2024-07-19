#include "JointPickable.hpp"

#include <Prototypes/MotionRetarget/IKController.hpp>
#include <crossforge/Graphics/RenderDevice.h>
#include <crossforge/AssetIO/SAssetIO.h>

#include <glad/glad.h>

namespace CForge {
using namespace Eigen;

JointPickableMesh::JointPickableMesh() {
	T3DMesh<float> mesh;
	SAssetIO::load("MyAssets/ccd-ik/joint.obj", &mesh); //TODO(skade) use primitive shape factory instead
	eigenMesh = EigenMesh(mesh);
	mesh.getMaterial(0)->Color.w() = .75;
	mesh.computeAxisAlignedBoundingBox();
	bv.init(mesh.aabb());

	actor.init(&mesh);

	mesh.getMaterial(0)->Color = Vector4f(227./255,142./255,48./255,1.);
	actorSel.init(&mesh);
}

void JointPickableMesh::setOpacity(float opacity) {
	Vector4f color = actor.material(0)->color();
	color.w() = opacity;
	actor.material(0)->color(color);
}
float JointPickableMesh::getOpacity() {
	return actor.material(0)->color().w();
}

void JointPickable::update(Matrix4f sgnT) {
	m_sgnT = sgnT;

	//TODO(skade) appply sg node transform
	const Matrix4f R = CForgeMath::rotationMatrix(m_pJoint->LocalRotation);
	const Matrix4f T = CForgeMath::translationMatrix(m_pJoint->LocalPosition);
	const Matrix4f S = CForgeMath::scaleMatrix(m_pJoint->LocalScale);
	Matrix4f JointTransform = T * R * S;

	m_fromPar = Matrix4f::Identity();
	if (m_pJoint->Parent != -1)
		m_fromPar = m_pIKC->getBone(m_pJoint->Parent)->SkinningMatrix
		          * m_pIKC->getBone(m_pJoint->Parent)->OffsetMatrix.inverse();
	Matrix4f LocalTransform = m_fromPar * JointTransform;

	Vector3f BoneVec; // vector to next bone
	if (m_pJoint->Children.size() > 0)
		BoneVec = m_pIKC->getBone(m_pJoint->Children[0])->LocalPosition;
	else
		BoneVec = m_pJoint->LocalPosition;
	float Length = BoneVec.norm(); // length to next bone
	
	Quaternionf LR = EigenFWD::FromTwoVectors(Vector3f::UnitX(), BoneVec.normalized()); // obj Joint points to +x axis

	//TODO sgnT
	m_transform = m_sgnT * LocalTransform * CForgeMath::rotationMatrix(LR) * CForgeMath::scaleMatrix(Vector3f(Length,Length,Length));
	if (!m_picked)
		m_transformGuizmo = m_sgnT * LocalTransform;
}
void JointPickable::pckMove(const Matrix4f& trans) {
	if (!m_pJoint) return;

	// extract new rotations from matrix
	//TODO(skade) unify place with init rest pose
	            //function that computes pos scale rot from mat4
	// 
	Matrix4f t = m_fromPar.inverse() * m_sgnT.inverse() * trans; //TODO(skade) order correct
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
	pRD->modelUBO()->modelMatrix(m_transform);
	m_pMesh->actor.render(pRD,
						Eigen::Quaternionf::Identity(),Eigen::Vector3f(),Eigen::Vector3f(1.f,1.f,1.f));
	glDisable(GL_BLEND);
	if (m_picked) {
		glCullFace(GL_FRONT);
		pRD->modelUBO()->modelMatrix(m_transform * CForgeMath::scaleMatrix(Vector3f(1.,1.3,1.3)));
		m_pMesh->actorSel.render(pRD,
							Eigen::Quaternionf::Identity(),Eigen::Vector3f(),Eigen::Vector3f(1.f,1.f,1.f));
		glCullFace(GL_BACK);
	}
}

}//CForge
