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
		ImVec2 region = ImGui::GetContentRegionAvail();
		if (region.x > 0 && region.y > 0)
			ImGui::InvisibleButton("inv",ImGui::GetContentRegionAvail());
		float border = ImGui::GetStyle().FramePadding.y + ImGui::GetFontSize();
		m_size = ImGui::GetWindowSize()-ImVec2(0,border*2);
		m_pos = ImGui::GetWindowPos()+ImVec2(0,border);
		ImGui::End();

		if (m_active)
			ImGuizmo::ViewManipulate(cameraView->data(), camDistance, m_pos, m_size, 0x00000000);
	}

	/**
	 * @brief checks whether pos is inside viewManipulate
	*/
	bool isInside(Vector2f pos) {
		return pos[0] > m_pos[0] && pos[0] < (m_pos[0]+m_size[0])
		    && pos[1] > m_pos[1] && pos[1] < (m_pos[1]+m_size[1]);
	}

	void update(Vector2f mousePos, bool LMBdown, bool active) {
		if (!LMBdown)
			m_startMousePos = mousePos;
		bool inside =  m_startMousePos.x() >= m_pos.x && m_startMousePos.x() <= m_pos.x + m_size.x;
		     inside &= m_startMousePos.y() >= m_pos.y && m_startMousePos.y() <= m_pos.y + m_size.y;
		bool insideCurr =  mousePos.x() >= m_pos.x && mousePos.x() <= m_pos.x + m_size.x;
		     insideCurr &= mousePos.y() >= m_pos.y && mousePos.y() <= m_pos.y + m_size.y;
		if (insideCurr)
			m_active = inside;
		else
			m_active = true;
		m_active &= active;
	}
private:
	Vector2f m_startMousePos;
	bool m_active = true;
	ImVec2 m_size = ImVec2(128,128);
	ImVec2 m_pos = ImVec2(0,0);
};

}//CForge
