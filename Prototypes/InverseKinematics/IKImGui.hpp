#pragma once
#include <Prototypes/GUI/ImGuiUtility.h>
#include "InverseKinematicsController.h"

namespace CForge {

class IKImGui {
	public:
		IKImGui();
		~IKImGui();

		void render();
	private:

	InverseKinematicsController* m_ikc;
	friend InverseKinematicsController;
};

}//CForge
