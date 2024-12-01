#pragma once

#include <crossforge/AssetIO/T3DMesh.hpp>

namespace CForge {

uint32_t getMatchingVertex(uint32_t RedundantVertexID, std::vector<std::pair<uint32_t, uint32_t>> *pRedundantVertices);

std::map<uint32_t, std::vector<uint32_t>> mergeRedundantVertices(T3DMesh<float>* pMesh);

}//CForge
