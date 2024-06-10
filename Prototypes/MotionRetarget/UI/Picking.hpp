#pragma once

#include "../CMN/EigenMesh.hpp"
#include <crossforge/Math/BoundingVolume.h>

namespace CForge {
using namespace Eigen;

class GLWindow;
class VirtualCamera;

/**
 * @brief Interface for a Pickable Object.
*/
class IPickable {
public:
	virtual void pckSelect() {};
	virtual void pckDeselect() {};
	virtual void pckMove(const Matrix4f& trans) = 0;

	/**
	 * @brief World space transform
	*/
	virtual Matrix4f pckTrans() = 0;
	virtual const BoundingVolume& pckBV() = 0;
	virtual EigenMesh* pckEigenMesh() { return nullptr; };
private:
};

/**
 * @brief Checks and Handles Picked Objects.
*/
class Picker {
public:
	Picker(GLWindow* pWin, VirtualCamera* pCam);
	void pick(std::vector<std::weak_ptr<IPickable>> objects);
	void update(Matrix4f trans);
	IPickable* getLastPick() {
		if (!m_pLastPick.expired())
			return m_pLastPick.lock().get();
		return nullptr;
	};
	Matrix4f m_guizmoMat = Matrix4f::Identity();
private:
	void rayCast(Vector3f* ro, Vector3f* rd);
	GLWindow* m_pWin;
	VirtualCamera* m_pCam;
	//TODO(skade) unsafe, object might get destroyed
	std::weak_ptr<IPickable> m_pLastPick; // picked object
	std::weak_ptr<IPickable> m_pCurrPick; // last clicked object
	//Vector3f m_DragStart = Vector3f::Zero(); //TODO(skade) remove
};

}//CForge
