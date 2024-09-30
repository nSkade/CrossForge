# pragma once

#include <crossforge/Core/CForgeObject.h>
#include <crossforge/Math/CForgeMath.h>
#include <vector>
#include <fstream>
#include <crossforge/Utility/CForgeUtility.h>

namespace CForge
{
	class objImportExport{
	public:
		objImportExport(void);
		~objImportExport(void);

		//TODO(skade) clearer names

		// ?
		static void readObjFile(std::string fileName, std::vector<Eigen::Vector3f> &vertices, std::vector<T3DMesh<float>::Face> &indices, std::vector<Eigen::Matrix<float, 3, 1>>& uvs);

		// import
		static void storeInMesh(std::string fileName, T3DMesh<float>* mesh);

		// export
		static void exportAsObjFile(std::string fileName, T3DMesh<float>* mesh);
		static void exportSubmeshesAsObjFiles(std::vector<std::string> filenames, T3DMesh<float>* mesh);
	};
	
} // namespace CForge
