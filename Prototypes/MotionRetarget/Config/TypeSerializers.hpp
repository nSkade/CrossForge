#pragma once

#include "Config.hpp"
#include <typeinfo>
#include <fstream>

namespace nlohmann {

template <>
struct adl_serializer<Eigen::Matrix4f> {
	static void to_json(json& j, const Eigen::Matrix4f& v) {
		j = std::vector<float>(v.data(),v.data()+16);
	}
	static void from_json(const json& j, Eigen::Matrix4f& v) {
		std::vector<float> d = j;
		assert(d.size() == 16);

		std::copy(d.data(),d.data()+16,v.data());
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
