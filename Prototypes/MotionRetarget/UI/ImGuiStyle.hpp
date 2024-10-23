#pragma once

// https://gist.github.com/dougbinks/8089b4bbaccaaf6fa204236978d165a9#file-imguiutils-h-L9-L93
inline void SetupImGuiStyle(bool is_dark_style, float alpha_threshold) {
	//Use a ternary operator
	is_dark_style ? ImGui::StyleColorsDark() : ImGui::StyleColorsLight();

	ImGuiStyle& style = ImGui::GetStyle();

	// Adjusts the alpha values of the ImGui colors based on the alpha threshold.
	for (int i = 0; i < ImGuiCol_COUNT; i++) {
		const auto color_id = static_cast<ImGuiCol>(i);
		auto& color = style.Colors[i];
		if (color.w < alpha_threshold || color_id == ImGuiCol_FrameBg || color_id == ImGuiCol_WindowBg || color_id == ImGuiCol_ChildBg)
			color.w *= alpha_threshold;
	}

	// Sets the border sizes and rounding.
	style.ChildBorderSize = 1.0f;
	style.FrameBorderSize = 0.0f;
	style.PopupBorderSize = 1.0f;
	style.WindowBorderSize = 0.0f;
	style.FrameRounding = 3.0f;
	style.Alpha = 1.0f;
}
