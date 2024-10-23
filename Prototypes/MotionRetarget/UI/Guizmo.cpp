#include "Guizmo.hpp"

#include <crossforge/Graphics/Camera/VirtualCamera.h>

namespace CForge {
	void Guizmo::setMat(Eigen::Matrix4f* mat) {
		m_pMat = mat;
	}
	void Guizmo::active(bool active) {
		m_objectPicked = active;
	}
	void Guizmo::setCam(const VirtualCamera* cam) {
		m_pCam = cam;
	}
	void Guizmo::render() {
		if (!m_pCam)
			return;
		m_manipulated = false;
		float* cameraView = m_pCam->cameraMatrix().data();
		float* cameraProjection = m_pCam->projectionMatrix().data();
		
		if (ImGui::IsKeyPressed(ImGuiKey_T))
			m_CurrentGizmoOperation = ImGuizmo::TRANSLATE;
		if (ImGui::IsKeyPressed(ImGuiKey_R))
			m_CurrentGizmoOperation = ImGuizmo::ROTATE;
		if (ImGui::IsKeyPressed(ImGuiKey_Y))
			m_CurrentGizmoOperation = ImGuizmo::SCALE;
		
		Eigen::Vector3f snap;
		if (m_useSnap) {
			switch (m_CurrentGizmoOperation)
			{
			case ImGuizmo::TRANSLATE:
				snap = m_snapTrans;
				break;
			case ImGuizmo::ROTATE:
				snap = m_snapRot;
				break;
			case ImGuizmo::SCALE:
				snap = m_snapScale;
				break;
			}
		}

		if (m_objectPicked && m_pMat) {
			if (m_renderDebugCube)
				ImGuizmo::DrawCubes(cameraView, cameraProjection, (*m_pMat).data(), 1);
			m_manipulated = ImGuizmo::Manipulate(cameraView, cameraProjection, m_CurrentGizmoOperation, m_CurrentGizmoMode,
								(*m_pMat).data(),NULL,m_useSnap ? snap.data() : NULL);
			//TODOfff(skade) ,boundSizing ? bounds : NULL, boundSizingSnap ? boundsSnap : NULL);
		}
	}
	void Guizmo::renderOptions() {
		if (ImGui::RadioButton("Translate (t)", m_CurrentGizmoOperation == ImGuizmo::TRANSLATE))
			m_CurrentGizmoOperation = ImGuizmo::TRANSLATE;
		ImGui::SameLine();
		if (ImGui::RadioButton("Rotate (r)", m_CurrentGizmoOperation == ImGuizmo::ROTATE))
			m_CurrentGizmoOperation = ImGuizmo::ROTATE;
		ImGui::SameLine();
		if (ImGui::RadioButton("Scale (y)", m_CurrentGizmoOperation == ImGuizmo::SCALE))
			m_CurrentGizmoOperation = ImGuizmo::SCALE;

		if (m_objectPicked && m_pMat) {
			float matrixTranslation[3], matrixRotation[3], matrixScale[3];
			ImGuizmo::DecomposeMatrixToComponents((*m_pMat).data(), matrixTranslation, matrixRotation, matrixScale);
			ImGui::DragFloat3("Tr", matrixTranslation,0.01);
			ImGui::DragFloat3("Rt", matrixRotation,0.1);
			ImGui::DragFloat3("Sc", matrixScale,0.01);
			ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, (*m_pMat).data());
		}

		ImGui::Checkbox("useSnap", &m_useSnap);
		if (m_useSnap) {
			ImGui::SameLine();
			switch (m_CurrentGizmoOperation)
			{
			case ImGuizmo::TRANSLATE:
				ImGui::DragFloat3("Snap", m_snapTrans.data(),0.01);
				break;
			case ImGuizmo::ROTATE:
				ImGui::DragFloat("Angle Snap", m_snapRot.data(),0.1);
				break;
			case ImGuizmo::SCALE:
				ImGui::DragFloat("Scale Snap", m_snapScale.data(),0.01);
				break;
			}
		}

		if (m_CurrentGizmoOperation != ImGuizmo::SCALE)
		{
			if (ImGui::RadioButton("Local", m_CurrentGizmoMode == ImGuizmo::LOCAL))
				m_CurrentGizmoMode = ImGuizmo::LOCAL;
			ImGui::SameLine();
			if (ImGui::RadioButton("World", m_CurrentGizmoMode == ImGuizmo::WORLD))
				m_CurrentGizmoMode = ImGuizmo::WORLD;
		}
	}
}//CForge
