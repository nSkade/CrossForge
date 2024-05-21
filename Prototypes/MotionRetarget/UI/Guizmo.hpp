#pragma once
#include <crossforge/Graphics/Camera/VirtualCamera.h>

namespace CForge {
	class Guizmo {
	public:
		void setMat(Eigen::Matrix4f* mat);
		void active(bool active);
		void setCam(const VirtualCamera* cam);
		void render();
		void renderOptions();
		bool m_manipulated = false;
	private:
		Eigen::Matrix4f* m_pMat = nullptr;
		bool m_objectPicked = false;
		const VirtualCamera* m_pCam = nullptr;
		ImGuizmo::MODE m_CurrentGizmoMode = ImGuizmo::LOCAL;
		ImGuizmo::OPERATION m_CurrentGizmoOperation = ImGuizmo::TRANSLATE;
		bool m_useSnap = false;
		Eigen::Vector3f m_snapTrans = Eigen::Vector3f(1.f,1.f,1.f);
		Eigen::Vector3f m_snapRot = Eigen::Vector3f(1.f,1.f,1.f);
		Eigen::Vector3f m_snapScale = Eigen::Vector3f(.2f,.2f,.2f);
		bool m_renderDebugCube = false;
	};
}//CForge
