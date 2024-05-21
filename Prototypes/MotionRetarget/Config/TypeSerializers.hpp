#include "Config.hpp"
#include <typeinfo>
#include <fstream>

namespace nlohmann {

template <>
struct adl_serializer<Eigen::Matrix4f> {
	static void to_json(json& j, const Eigen::Matrix4f& v) {
		std::vector<float> d;
		for (uint32_t i=0;i<16;++i)
			d.push_back(v.data()[i]);
		j = d;
	}
	static void from_json(const json& j, Eigen::Matrix4f& v) {
		std::vector<float> d = j;
		for (uint32_t i=0;i<16;++i)
			v.data()[i] = d[i];
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

