#pragma once

#include <imgui.h>

namespace CForge {
using namespace Eigen;

class ViewManipulate {
public:
	void render(Matrix4f* cameraView, float camDistance) {
		//ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(m_viewManipulateRight - 128, m_viewManipulateTop), m_size, 0x10101010);
		ImGui::Begin("ViewManipulate");
		// invisible button to avoid drag and drop on window region
		ImGui::InvisibleButton("inv",ImGui::GetContentRegionAvail());
		float border = ImGui::GetStyle().FramePadding.y + ImGui::GetFontSize();
		m_size = ImGui::GetWindowSize()-ImVec2(0,border*2);
		m_pos = ImGui::GetWindowPos()+ImVec2(0,border);
		ImGui::End();
		ImGuizmo::ViewManipulate(cameraView->data(), camDistance, m_pos, m_size, 0x00000000);
	}

	/**
	 * @brief checks whether pos is inside viewManipulate
	*/
	bool isInside(Vector2i pos) {
		return pos[0] > m_pos[0] && pos[0] < (m_pos[0]+m_size[0])
		    && pos[1] > m_pos[1] && pos[1] < (m_pos[1]+m_size[1]);
	}
private:
	ImVec2 m_size = ImVec2(128,128);
	ImVec2 m_pos = ImVec2(0,0);
};

}//CForge
