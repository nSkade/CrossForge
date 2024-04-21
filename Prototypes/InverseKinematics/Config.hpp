#pragma once

#include <nlohmann/json.hpp>

#include <crossforge/Graphics/VirtualCamera.h>

namespace CForge {

/**
 * @brief Config class for parsing various Types.
 */
class Config {
public:
	void baseStore();
	void baseLoad();

	void store(const VirtualCamera& object);
	void load(VirtualCamera* object);

private:
	std::string ConfigFilepath = "CFconfig.json";
	nlohmann::json m_ConfigData;
};

}//CForge

