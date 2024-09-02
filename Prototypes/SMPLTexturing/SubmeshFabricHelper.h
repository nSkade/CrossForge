#pragma once

#include "json/json.h"
#include <vector>
#include <crossforge/Core/CForgeObject.h>
#include <crossforge/Utility/CForgeUtility.h>

namespace CForge
{
    struct FabricHelper{
        int id; 
        T3DMesh<float>::Submesh *submesh;
        bool isSkin = false; 
        
        // after evaluating the python nn we will have to fill this
        std::string fabricName; 
        double proability; 
    }; 

    class SubmeshFabricHelper{
    public:
        // need a function for reading the json then and filling the fabric helper
        static std::vector<FabricHelper> readFabricHelper(std::string filename);
    
    }; // SubmeshFabricHelper

} // namespace CForge
