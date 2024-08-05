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
	virtual Matrix4f pckTransGuizmo() = 0; // used for guizmo update
	virtual Matrix4f pckTransPickin() = 0; // used for picking evaluation
	virtual const BoundingVolume& pckBV() = 0;
	virtual EigenMesh* pckEigenMesh() { return nullptr; };
};

/**
 * @brief Checks and Handles Picked Objects.
*/
class Picker {
public:
	Picker(GLWindow* pWin, VirtualCamera* pCam);
	void pick(std::vector<std::weak_ptr<IPickable>> objects);
	void forcePick(std::weak_ptr<IPickable> pick);
	void start();
	void resolve();
	void reset();
	void update(Matrix4f trans);
	std::weak_ptr<IPickable> getLastPick() {
		return m_pLastPick;
	};
	std::weak_ptr<IPickable> getCurrPick() {
		return m_pCurrPick;
	};
	Matrix4f m_guizmoMat = Matrix4f::Identity();
private:
	void rayCast(Vector3f* ro, Vector3f* rd);
	GLWindow* m_pWin;
	VirtualCamera* m_pCam;
	std::weak_ptr<IPickable> m_pLastPick; // picked object
	std::weak_ptr<IPickable> m_pCurrPick; // last clicked object
	std::weak_ptr<IPickable> m_pPick; // last clicked object
};

}//CForge
