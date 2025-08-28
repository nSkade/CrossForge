#pragma once

#include <nlohmann/json.hpp>

namespace CForge {
class VirtualCamera;
class GLWindow;

/**
 * @brief Config class for parsing various Types.
 */
class Config {
public:
	void store(const VirtualCamera& object, std::string name);
	void store(const VirtualCamera& object);
	void load(VirtualCamera* object, std::string name);
	void load(VirtualCamera* object);
	void store(const GLWindow& object);
	void load(GLWindow* object); 

	void store(std::string name, const Eigen::Vector3f& object);
	void load(std::string name, Eigen::Vector3f* object); 

public:
	void baseStore();
	void baseLoad();

	// template functions to load and store simple types
	template <typename T>
	void store(std::string name, const T& value) {
		m_ConfigData[name] = value;
	}
	
	template <typename T>
	void load(std::string name, T* value) {
		if (m_ConfigData.contains(name)) {
			(*value) = m_ConfigData[name];
		}
	}

private:
	std::string ConfigFilepath = "CFconfig.json";
	nlohmann::json m_ConfigData;
};

}//CForge
