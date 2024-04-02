# pragma once

#include <crossforge/Core/CForgeObject.h>
#include <crossforge/Math/CForgeMath.h>
#include <vector>
#include <fstream>
#include <crossforge/Utility/CForgeUtility.h>

namespace CForge
{
    class objImport{
    public:
        objImport(void);
        ~objImport(void);

        static void readObjFile(std::string fileName, std::vector<Eigen::Vector3f> &vertices, std::vector<T3DMesh<float>::Face> &indices);
        static void storeInMesh(std::string fileName, T3DMesh<float>* mesh);
    };
    
} // namespace CForge
