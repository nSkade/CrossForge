#pragma once

#include "IAutoRigger.hpp"

namespace CForge {
using namespace Eigen;

struct ARrignetOptions {
	float bandwidth = 0.0429;
	float threshold = 1e-5;
	bool parseOutputOnly = false;
};

class ARrignet : IAutoRigger<ARrignetOptions> {
public:
	// path to anaconda installation, folder which should contain _conda.exe
	std::string condaPath;//"C:/Users/Admin/miniconda3/";

	// path to rignet root, folder which should contain quick_start.py
	std::string rignetPath;//"\"C:/Users/Admin/Desktop/BA Shared/5. Autorigging/RigNet\"";

	void rig(T3DMesh<float>* mesh, ARrignetOptions options);
};

}//CForge
