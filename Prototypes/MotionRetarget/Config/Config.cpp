#include "TypeSerializers.hpp"

#include <crossforge/Graphics/VirtualCamera.h>

namespace CForge {

void Config::store(const VirtualCamera* object) {
	m_ConfigData["VirtualCamera"] = object->cameraMatrix();
}
void Config::load(VirtualCamera* object) {
	if (m_ConfigData.contains("VirtualCamera")) {
		Eigen::Matrix4f m = m_ConfigData["VirtualCamera"];
		object->cameraMatrix(m);
	}
}

}//CForge
