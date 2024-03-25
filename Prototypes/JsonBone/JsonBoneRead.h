# pragma once

#include <crossforge/Core/CForgeObject.h>
#include "json/json.h"
#include <vector>
#include "crossforge/AssetIO/T3DMesh.hpp"

namespace CForge
{
    struct JsonBone{
        int id;
        std::string name; 
        int parent;
        Eigen::Vector3f position;
        std::vector<float> vertexWeights;
    }; //JsonBone

    class BuildBones{
    public:
        BuildBones(void);
        ~BuildBones(void);

        static void readJsonFile(const std::string &fileName, std::vector<JsonBone> &bones); 
        static void constructBoneHierarchy(std::vector<JsonBone> &bonesJson, std::vector<T3DMesh<float>::Bone*> &pBones);
        static Eigen::Matrix4f getOffsetMatrix(Eigen::Vector3f position);
        static void getVertexInfluenceAndWeight(JsonBone Bone, std::vector<int32_t> &VertexInfluences, std::vector<float> &VertexWeights);
        static std::vector<T3DMesh<float>::Bone*> getBones(const std::string &fileName); 
    };
    
} // namespace CForge
