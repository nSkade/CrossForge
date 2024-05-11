#include <iostream>
#include <fstream>
#include "JsonBoneRead.h"

namespace CForge{

    std::vector<T3DMesh<float>::Bone*> BuildBones::getBones(const std::string &fileName){
        std::vector<JsonBone> bonesJson;
        std::vector<T3DMesh<float>::Bone*> pBones;
        BuildBones::readJsonFile(fileName, bonesJson);
        BuildBones::constructBoneHierarchy(bonesJson, pBones);
        return pBones;
    }

    std::vector<T3DMesh<float>::Bone*> BuildBones::getBones(const std::string &fileName, std::vector<std::vector<int>> &correspondences){
        std::vector<JsonBone> bonesJson;
        std::vector<T3DMesh<float>::Bone*> pBones;
        BuildBones::readJsonFile(fileName, bonesJson);
        
        // TODO: construct new constructBoneHierarchy were you have corespondences
        BuildBones::constructBoneHierarchy(bonesJson, pBones, correspondences);
        
        return pBones;
    }

    // create function that just returns the skinning weights for futher use 
    // just needs to be a valid json file with the skinning weights, the positions do not matter
    void BuildBones::readJsonFileExtractSkinningWeights(const std::string &fileName, std::vector<std::vector<int32_t>>& VertexInfluences, std::vector<std::vector<float>>& VertexWeights){
        std::vector<JsonBone> bones; 
        readJsonFile(fileName, bones);

        VertexInfluences.clear(); 
        VertexWeights.clear(); 

        for(int i = 0; i < bones.size(); i++){
            std::vector<int32_t> influence; 
            std::vector<float> weight; 
            getAllVertexInfluenceAndWeight(bones[i], influence, weight);

            VertexInfluences.push_back(influence);
            VertexWeights.push_back(weight);  
        }
    }
    
    
    void BuildBones::readJsonFile(const std::string &fileName, std::vector<JsonBone> &bones){
        std::ifstream file(fileName);
        Json::Reader reader;
        Json::Value root;
        bool parsingSuccessful = reader.parse(file, root);

        if (!parsingSuccessful) {
            throw CForgeExcept("Failed to parse JSON");
		}
        bones.clear(); 
        int countJsonElements = root.size();

        for (size_t i = 0; i < countJsonElements; i++)
        {
            std::string id = std::to_string(i);
            const Json::Value element = root[id];
            JsonBone bone;

            bone.id = i; 
            bone.name = element["name"].asString();
            bone.parent = element["parent_id"].asInt();
            bone.position = Eigen::Vector3f(element["position"][0].asFloat(), 
                                            element["position"][1].asFloat(), 
                                            element["position"][2].asFloat());

            const Json::Value vertexWeights = element["vertex_weight"];
            for(int j = 0; j < vertexWeights.size(); j++){
                bone.vertexWeights.push_back(vertexWeights[j].asFloat());
            }

            bones.push_back(bone);
        }
    }

    // Construc the bone hierarchy for a given mesh
    void BuildBones::constructBoneHierarchy(std::vector<JsonBone> &bonesJson, std::vector<T3DMesh<float>::Bone*> &pBones){
        pBones.clear();
        
        for (size_t i = 0; i < bonesJson.size(); i++){
            pBones.push_back(new T3DMesh<float>::Bone());
            Eigen::Matrix4f offsetMatrix = BuildBones::getOffsetMatrix(bonesJson[i].position);
            
            pBones[i]->ID = bonesJson[i].id;
            pBones[i]->Name = bonesJson[i].name;
            pBones[i]->Position = bonesJson[i].position;
            pBones[i]->OffsetMatrix = offsetMatrix;

            // for(int j = 0; j < bonesJson[i].vertexWeights.size(); j++){
            //     pBones[i]->VertexInfluences.push_back(j);
            // }
            // pBones[i]->VertexWeights = bonesJson[i].vertexWeights;

            std::vector<int32_t> influence; 
			std::vector<float> weights; 
            getVertexInfluenceAndWeight(bonesJson[i], influence, weights);
            pBones[i]->VertexInfluences = influence;
            pBones[i]->VertexWeights = weights; 
        }

        // set parents and children
        for (size_t i = 0; i < bonesJson.size(); i++){
            if(bonesJson[i].parent != -1){
                pBones[i]->pParent = pBones[bonesJson[i].parent];
                pBones[bonesJson[i].parent]->Children.push_back(pBones[i]);
            }
            else{
                pBones[i]->pParent = nullptr;
            }
        }
    }

    // there are still many values that are zero, so we need to filter them out
    // the problem then would be that we do not have the indicies of the vertices so we need to store them as well
    void BuildBones::getVertexInfluenceAndWeight(JsonBone Bone, std::vector<int32_t>& VertexInfluences, std::vector<float>& VertexWeights){
        // theoretical can also implement a threshold between 0 and 1; if small then the threshold then ignore
        VertexInfluences.clear();
        VertexWeights.clear();

        for(int i = 0; i < Bone.vertexWeights.size(); i++){
            if(Bone.vertexWeights[i] != 0.0f){
                VertexInfluences.push_back(i);
                VertexWeights.push_back(Bone.vertexWeights[i]);
            }
        }
    }

     void BuildBones::getAllVertexInfluenceAndWeight(JsonBone Bone, std::vector<int32_t>& VertexInfluences, std::vector<float>& VertexWeights){
        // theoretical can also implement a threshold between 0 and 1; if small then the threshold then ignore
        VertexInfluences.clear();
        VertexWeights.clear();

        for(int i = 0; i < Bone.vertexWeights.size(); i++){
            VertexInfluences.push_back(i);
            VertexWeights.push_back(Bone.vertexWeights[i]);
        }
    }

    // in order to construct the bone hierarchy, we need to calculate the offset matrix
    // this is done by inverting the translation matrix (according to Mick)
    Eigen::Matrix4f BuildBones::getOffsetMatrix(Eigen::Vector3f position){
        Eigen::Matrix4f offsetMatrix = CForgeMath::translationMatrix(position);

        //Eigen::Matrix4f offsetMatrix = Eigen::Matrix4f::Identity();
        // offsetMatrix.block<3,1>(0,3) = position; 
        return offsetMatrix.inverse(); 
        //offsetMatrix = offsetMatrix.inverse(); 
    }

    // Construc the bone hierarchy for a given mesh
    void BuildBones::constructBoneHierarchy(std::vector<JsonBone> &bonesJson, std::vector<T3DMesh<float>::Bone*> &pBones, std::vector<std::vector<int>> &correspondences){
        pBones.clear();
        
        for (size_t i = 0; i < bonesJson.size(); i++){
            pBones.push_back(new T3DMesh<float>::Bone());
            Eigen::Matrix4f offsetMatrix = BuildBones::getOffsetMatrix(bonesJson[i].position);
            
            pBones[i]->ID = bonesJson[i].id;
            pBones[i]->Name = bonesJson[i].name;
            pBones[i]->Position = bonesJson[i].position;
            pBones[i]->OffsetMatrix = offsetMatrix;

            std::vector<int32_t> influence; 
			std::vector<float> weights; 
            getVertexInfluenceAndWeight(bonesJson[i], influence, weights);

            // now adjust the influences and weights for the other obj model
            std::vector<int32_t> proxyInfluence;
            std::vector<float> proxyWeights;
            for(int i = 0; i < influence.size(); i++){
                std::vector<int> tmpProxyInfluence = correspondences[influence[i]]; 
                float tmpProxyWeight = weights[i]; 

                for(int j = 0; j < tmpProxyInfluence.size(); j++){
                    proxyInfluence.push_back(tmpProxyInfluence[j]);
                    proxyWeights.push_back(tmpProxyWeight); 
                }

            }

            pBones[i]->VertexInfluences = proxyInfluence;
            pBones[i]->VertexWeights = proxyWeights; 
        }

        // set parents and children
        for (size_t i = 0; i < bonesJson.size(); i++){
            if(bonesJson[i].parent != -1){
                pBones[i]->pParent = pBones[bonesJson[i].parent];
                pBones[bonesJson[i].parent]->Children.push_back(pBones[i]);
            }
            else{
                pBones[i]->pParent = nullptr;
            }
        }
    }

}// namespace CForge