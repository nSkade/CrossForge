#pragma once

#include <vector>
#include <crossforge/Core/CForgeObject.h>
#include <crossforge/Utility/CForgeUtility.h>

namespace CForge
{
    struct FaceSubmesh {
        int faceIndex; 
        int submeshIndex;    
    }; 

    class MeshConnection{
    public:
        static std::vector<int32_t> getAdjacentVertices(T3DMesh<float>* pMesh, int vertexIndex);
        static std::vector<int32_t> getEdgesSubmesh(T3DMesh<float>* mesh, int submeshIndex, std::vector<std::vector<int32_t>> *ignoreList = nullptr); 
        static std::vector<int32_t> getNeighboringFacesSubmesh(T3DMesh<float>* mesh, int submeshIndex, int faceIndex);
        static std::vector<int32_t> getAdjacentFacesFromVertex(T3DMesh<float>* pMesh, int submeshIndex, int vertexIndex);
        static std::vector<FaceSubmesh> getAdjacentFacesFromVertex(T3DMesh<float>* pMesh, int vertexIndex); 
        static std::vector<int32_t> getAdjacentFacesSubmesh(T3DMesh<float>* pMesh, int submeshIndex, int faceIndex);
        static std::vector<FaceSubmesh> getAdjacentFaces(T3DMesh<float>* pMesh, int submeshIndex, int faceIndex); 
        static std::vector<T3DMesh<float>::Submesh*> getAdjacentSubmeshes(T3DMesh<float>* pMesh, int submeshIndex);
        static std::vector<std::vector<int32_t>> getAllBorders(T3DMesh<float>* pMesh);
        static bool shareEdge(T3DMesh<float>::Face f1, T3DMesh<float>::Face f2);
        static std::vector<int32_t> getIsolatedFacesSubmesh(T3DMesh<float>* pMesh, int submeshIndex);
        static float getAreaFace(T3DMesh<float>* pMesh, int submeshIndex, int faceIndex); 
        static float getAreaSubmesh(T3DMesh<float>* pMesh, int submeshIndex);
        static std::vector<T3DMesh<float>::Face> getIsolatedFacesWholeMesh(T3DMesh<float>* pMesh);
        static bool areSameFace(T3DMesh<float>::Face f1, T3DMesh<float>::Face f2);
        static int getFaceIndex(T3DMesh<float>* pMesh, int submeshIndex, T3DMesh<float>::Face face);
        static T3DMesh<float> splitMeshFromProxy(T3DMesh<float>* pMeshOrg, T3DMesh<float>* pMeshRegions, std::vector<std::vector<int32_t>> vertexMap);
        static void copyMesh(T3DMesh<float>* pMeshOrg, T3DMesh<float>* pMeshCopy);
    protected:
        static bool vertexOfFace(T3DMesh<float>::Face face, int vertexIndex); 
    }; // MeshConnection
}