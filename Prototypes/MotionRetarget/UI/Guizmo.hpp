#pragma once

namespace CForge {
using namespace Eigen;
class VirtualCamera;

class Guizmo {
public:
	void setMat(Matrix4f* mat);
	void active(bool active);
	void setCam(const VirtualCamera* cam);
	void render();
	void renderOptions();
	bool m_manipulated = false;
	bool m_renderDebugCube = false;
private:
	Matrix4f* m_pMat = nullptr;
	bool m_objectPicked = false;
	const VirtualCamera* m_pCam = nullptr;
	ImGuizmo::MODE m_CurrentGizmoMode = ImGuizmo::WORLD;
	ImGuizmo::OPERATION m_CurrentGizmoOperation = ImGuizmo::TRANSLATE;
	bool m_useSnap = false;
	Vector3f m_snapTrans = Vector3f(1.f,1.f,1.f);
	Vector3f m_snapRot = Vector3f(1.f,1.f,1.f);
	Vector3f m_snapScale = Vector3f(.2f,.2f,.2f);
};

}//CForge
