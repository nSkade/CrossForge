#include "Picking.hpp"

#include <crossforge/Graphics/GLWindow.h>
#include <crossforge/Graphics/Camera/VirtualCamera.h>

namespace CForge {
using namespace Eigen;

Picker::Picker(GLWindow* pWin, VirtualCamera* pCam)
	: m_pWin(pWin), m_pCam(pCam) {
	
}

void Picker::rayCast(Vector3f* ro, Vector3f* rd) {
	Vector4f Viewport = Vector4f(0.0f, 0.0f, float(m_pWin->width()), float(m_pWin->height())); 
	Vector2f CursorPos = Vector2f(m_pWin->mouse()->position().x(), Viewport(3) - m_pWin->mouse()->position().y());
	Matrix4f View = m_pCam->cameraMatrix();
	Matrix4f Projection = m_pCam->projectionMatrix();
	
	igl::unproject_ray(CursorPos, View, Projection, Viewport, *ro, *rd);
	rd->normalize();
}

void Picker::pick(std::vector<std::weak_ptr<IPickable>> objects) {
	
	Vector3f ro0, rd0;
	rayCast(&ro0, &rd0);
	std::weak_ptr<IPickable> pPick;
	Matrix4f t = Matrix4f::Identity();

	for (int32_t i = 0; i < objects.size(); ++i) {
		std::shared_ptr<IPickable> pPobj = objects[i].lock();

		float T0 = m_pCam->nearPlane();
		float T1 = m_pCam->farPlane();
		float TMin, TMax; // minimum and maximum of interval of overlap within [T0, T1] -> not actually used here, but required by function
		const BoundingVolume& bv = pPobj->pckBV();
		bool hit = false;

		// simply apply obj Transform my inverse transforming ray pos
		Matrix4f objTrans = pPobj->pckTransPickin();
		objTrans = objTrans.inverse();

		Vector4f ro04; ro04 << ro0,1.;
		Vector3f ro = (objTrans*Vector4f(ro04)).head<3>();
		Vector4f rd04; rd04 << rd0,0.;
		Vector3f rd = (objTrans*Vector4f(rd04)).head<3>();

		switch (bv.type()) {
		case BoundingVolume::TYPE_AABB: {
			AlignedBox3f aabb = AlignedBox3f(bv.aabb().min(), bv.aabb().max());
			hit = igl::ray_box_intersect(ro, rd, aabb, T0, T1, TMin, TMax);
		} break;
		case BoundingVolume::TYPE_SPHERE: {
			Vector3d c = bv.boundingSphere().center().cast<double>();
			float r = bv.boundingSphere().radius();
			Vector3d rod = ro.cast<double>();
			Vector3d rdd = rd.cast<double>();
			hit = igl::ray_sphere_intersect(rod,rdd,c,r,T0,T1);
		} break;
		default:
			break;
		}

		//TODOf(skade) check for distance in case on screenspace overlap of multiple aabb
		if (hit) {
			//TODOf(skade) implement mesh intersection test
			EigenMesh* pMesh = pPobj->pckEigenMesh();
			if (pMesh) {
				igl::Hit iglHit;
				hit = igl::ray_mesh_intersect(ro,rd,pMesh->getDV(),pMesh->getDF(),iglHit);
				//if (!hit)
				//	continue;
			}
			pPick = pPobj;
			break;
		}
	}
	if (!pPick.expired()) {
		m_guizmoMat = pPick.lock().get()->pckTransGuizmo();

		if (!m_pLastPick.expired())
			m_pLastPick.lock().get()->pckDeselect();
		pPick.lock().get()->pckSelect();

		m_pLastPick = pPick;
		m_pCurrPick = pPick;
	}
	else {
		// deselect only after pick was miss again
		if (m_pCurrPick.expired()) {
			if (!m_pLastPick.expired())
				m_pLastPick.lock().get()->pckDeselect();
			m_pLastPick.reset();
		}
		m_pCurrPick.reset();
	}
}

void Picker::update(Matrix4f trans) {
	if (m_pLastPick.expired())
		return;

	m_guizmoMat = trans;
	m_pLastPick.lock().get()->pckMove(trans);
}

}//CForge
