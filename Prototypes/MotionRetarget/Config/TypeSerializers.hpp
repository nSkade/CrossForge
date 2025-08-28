#pragma once

#include "Config.hpp"
#include <typeinfo>
#include <fstream>

// Matrix4f adl_serializer
namespace nlohmann {
template <>
struct adl_serializer<Eigen::Matrix4f> {
	static void to_json(json& j, const Eigen::Matrix4f& m) {
		j = std::vector<float>(m.data(), m.data() + 16);
	}
	static void from_json(const json& j, Eigen::Matrix4f& m) {
		std::vector<float> d = j.get<std::vector<float>>();
		assert(d.size() == 16);
		std::copy(d.begin(), d.end(), m.data());
	}
};
}//nlohmann

namespace CForge {

void Config::baseStore() {
	std::ofstream out(ConfigFilepath);
	out << std::setw(4) << m_ConfigData << std::endl;
	out.close();
}

void Config::baseLoad() {
	std::ifstream in(ConfigFilepath);
	if (in.is_open())
		m_ConfigData = nlohmann::json::parse(in);
	in.close();
}

}//CForge
