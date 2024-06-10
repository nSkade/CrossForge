#include "TypeSerializers.hpp"

#include <crossforge/Graphics/Camera/VirtualCamera.h>
#include <crossforge/Graphics/GLWindow.h>

namespace CForge {
using namespace Eigen;

void Config::store(const VirtualCamera& object) {
	m_ConfigData["VirtualCamera"] = object.cameraMatrix();
}
void Config::load(VirtualCamera* object) {
	if (m_ConfigData.contains("VirtualCamera")) {
		Matrix4f m = m_ConfigData["VirtualCamera"];
		object->cameraMatrix(m);
	}
}

void Config::store(const GLWindow& object) {
	m_ConfigData["GLWindow"]["width"] = object.size().x();
	m_ConfigData["GLWindow"]["height"] = object.size().y();
	m_ConfigData["GLWindow"]["posx"] = object.position().x();
	m_ConfigData["GLWindow"]["posy"] = object.position().y();
}
void Config::load(GLWindow* object) {
	if (m_ConfigData.contains("GLWindow")) {
		object->size(m_ConfigData["GLWindow"]["width"],m_ConfigData["GLWindow"]["height"]);
		object->position(m_ConfigData["GLWindow"]["posx"],m_ConfigData["GLWindow"]["posy"]);
	}
}

}//CForge
