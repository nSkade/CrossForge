#pragma once

#include <Prototypes/GUI/ImGuiUtility.h>
#include <Prototypes/MotionRetarget/IKController.hpp>

namespace CForge {

//TODO(skade)
class IKImGui {
	public:
		IKImGui();
		~IKImGui();

		void render();
	private:

	IKController* m_ikc;
	friend IKController;
};

}//CForge
