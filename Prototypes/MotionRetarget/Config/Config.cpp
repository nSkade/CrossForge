#include "TypeSerializers.hpp"

#include <crossforge/Graphics/Camera/VirtualCamera.h>
#include <crossforge/Graphics/GLWindow.h>

namespace CForge {
using namespace Eigen;

void Config::store(const VirtualCamera& object, std::string name) {
	m_ConfigData[name] = object.cameraMatrix();
}

void Config::store(const VirtualCamera& object) {
	store(object,"VirtualCamera");
}

void Config::load(VirtualCamera* object, std::string name) {
	if (m_ConfigData.contains(name)) {
		Matrix4f m = m_ConfigData[name];
		object->cameraMatrix(m);
	}
}

void Config::load(VirtualCamera* object) {
	load(object,"VirtualCamera");
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

void Config::store(std::string name, const Eigen::Vector3f& object) {
	// Store as JSON array [x, y, z]
	m_ConfigData[name] = nlohmann::json::array({object.x(), object.y(), object.z()});
}

void Config::load(std::string name, Eigen::Vector3f* object) {
	const auto& arr = m_ConfigData[name];
	object->x() = arr[0].get<float>();
	object->y() = arr[1].get<float>();
	object->z() = arr[2].get<float>();
}

}//CForge
