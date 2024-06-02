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
	void pick(std::vector<IPickable*> objects);
	void update(Matrix4f trans);
	IPickable* getLastPick() { return m_pLastPick; };
	Matrix4f m_guizmoMat = Matrix4f::Identity();
private:
	void rayCast(Vector3f* ro, Vector3f* rd);
	GLWindow* m_pWin;
	VirtualCamera* m_pCam;
	//TODO(skade) unsafe, object might get destroyed
	IPickable* m_pLastPick = nullptr; // picked object
	IPickable* m_pCurrPick = nullptr; // last clicked object
	//Vector3f m_DragStart = Vector3f::Zero(); //TODO(skade) remove
};

}//CForge
