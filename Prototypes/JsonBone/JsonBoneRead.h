# pragma once

#include <crossforge/Core/CForgeObject.h>
#include "json/json.h"
#include <vector>
#include "crossforge/AssetIO/T3DMesh.hpp"
#include <crossforge/Math/CForgeMath.h>

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
        static void constructBoneHierarchy(std::vector<JsonBone> &bonesJson, std::vector<T3DMesh<float>::Bone*> &pBones, std::vector<std::vector<int>> &correspondences);
        static Eigen::Matrix4f getOffsetMatrix(Eigen::Vector3f position);
        static void getVertexInfluenceAndWeight(JsonBone Bone, std::vector<int32_t> &VertexInfluences, std::vector<float> &VertexWeights);
        static void getAllVertexInfluenceAndWeight(JsonBone Bone, std::vector<int32_t> &VertexInfluences, std::vector<float> &VertexWeights); 
        static std::vector<T3DMesh<float>::Bone*> getBones(const std::string &fileName);
        static std::vector<T3DMesh<float>::Bone*> getBones(const std::string &filenName, std::vector<std::vector<int>> &correspondences); 
        static void readJsonFileExtractSkinningWeights(const std::string &fileName, std::vector<std::vector<int32_t>>& VertexInfluences, std::vector<std::vector<float>>& VertexWeights); 
    };
    
} // namespace CForge
