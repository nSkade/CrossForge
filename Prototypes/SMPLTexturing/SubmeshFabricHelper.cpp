#include "SubmeshFabricHelper.h"
#include <iostream>
#include <fstream>

namespace CForge {
    
    // need a function for reading the json then and filling the fabric helper
    std::vector<FabricHelper> SubmeshFabricHelper::readFabricHelper(std::string filename){
        std::vector<FabricHelper> fabricHelpers; 
        
        std::ifstream file(filename);
        Json::Reader reader;
        Json::Value root;
        bool parsingSuccessful = reader.parse(file, root);

        if (!parsingSuccessful) {
            throw CForgeExcept("Failed to parse JSON");
		}
        int countJsonElements = root.size();
        for(int i = 0; i < countJsonElements; i++){
            FabricHelper fabricHelper;
            fabricHelper.id = root[i]["id"].asInt();
            fabricHelper.fabricName = root[i]["label"].asString();
            fabricHelper.proability = root[i]["probability"].asDouble();
            fabricHelpers.push_back(fabricHelper);
        }

        return fabricHelpers;
    }

} // namespace CForge