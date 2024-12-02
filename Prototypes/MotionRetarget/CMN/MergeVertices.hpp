#pragma once

#include <crossforge/AssetIO/T3DMesh.hpp>

namespace CForge {

/**
 * @brief returns first vertex id of redundant vertex
*/
uint32_t getMatchingVertex(uint32_t RedundantVertexID, std::vector<std::pair<uint32_t, uint32_t>> *pRedundantVertices);

/**
 * @brief merges Vertices of Mesh
 * @return mapping from old mesh vert to reduced vert
*/
std::vector<uint32_t> mergeRedundantVertices(T3DMesh<float>* pMesh);

}//CForge
