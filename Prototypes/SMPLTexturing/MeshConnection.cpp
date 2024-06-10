#include "MeshConnection.h"
#include <unordered_set>
#include <algorithm>
#include <iostream>

namespace CForge{
    std::vector<int32_t> MeshConnection::getAdjacentVertices(T3DMesh<float>* pMesh, int vertexIndex){
        if(pMesh == nullptr || vertexIndex >= pMesh->vertexCount()) throw CForgeExcept("Nullpointer or out of range");
        
        std::vector<int32_t> Rval; 
        std::unordered_set<int> s; 
        
        // get every face and test if the vertex belongs to the face, if it does, then add to the vector    
        for(int subCount = 0; subCount < pMesh->submeshCount(); subCount++){
            T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(subCount); 
            for(int faceCount = 0; faceCount < sub->Faces.size(); faceCount++){
                T3DMesh<float>::Face face = sub->Faces[faceCount]; 
                if(vertexOfFace(face, vertexIndex)){
                    s.insert(face.Vertices[0]); s.insert(face.Vertices[1]), s.insert(face.Vertices[2]);
                }
            }//faces
        }// submesh

        // unordered_set to vec
        Rval.assign(s.begin(), s.end()); 

        // we do not want to have the vertex index in this vector
        Rval.erase(std::remove(Rval.begin(), Rval.end(), vertexIndex), Rval.end()); 
        
        // same result as the python variant
        return Rval; 
    }//getAdjacentVertices

    std::vector<int32_t> MeshConnection::getEdgesSubmesh(T3DMesh<float>* mesh, int submeshIndex, std::vector<std::vector<int32_t>>* ignoreList){
        if(mesh == nullptr || submeshIndex >= mesh->submeshCount() || submeshIndex < 0) throw CForgeExcept("Nullpointer or Index out of range");
        T3DMesh<float>::Submesh *sub = mesh->getSubmesh(submeshIndex);

        // if triangle does not have 3 neighbors, then it is an edge
        // if it has 3 neighbors, then it is not an edge - can put it into ignore list
        std::vector<int32_t> Rval;
        // std::vector<std::vector<int32_t>> ignoreList;

        if(ignoreList == nullptr) ignoreList = new std::vector<std::vector<int32_t>>; 

        for(int i = 0; i < sub->Faces.size(); i++){
            ignoreList->push_back(std::vector<int32_t>());
        }

        for(int i = 0; i < sub->Faces.size(); i++){
            // if we checked already beforehand, then we can skip
            if(ignoreList->at(i).size() == 3) continue;

            T3DMesh<float>::Face f = sub->Faces[i];
            std::vector<int32_t> adjFaces = getAdjacentFacesSubmesh(mesh, submeshIndex, i);
            if(adjFaces.size() < 3){
                Rval.push_back(i);
            }else{
                // basically we create a graph, where we ignore the faces that are already checked
                // need to check whether the face is already in the ignore list
                for(int j = 0; j < adjFaces.size(); j++){
                    int cnt1 = std::count(ignoreList->at(i).begin(), ignoreList->at(i).end(), adjFaces[j]);
                    int cnt2 = std::count(ignoreList->at(adjFaces[j]).begin(), ignoreList->at(adjFaces[j]).end(), i);

                    if(cnt1 == 0) ignoreList->at(i).push_back(adjFaces[j]);
                    if(cnt2 == 0) ignoreList->at(adjFaces[j]).push_back(i);
                }
            }
        }

        return Rval; 
    }

    std::vector<int32_t> MeshConnection::getNeighboringFacesSubmesh(T3DMesh<float>* pMesh, int submeshIndex, int faceIndex){
        // there are two different definitions of Adjacent: if they share an edge or just
        // a common vertex - the second one is also needed for the first one
        
        if(pMesh == nullptr || submeshIndex >= pMesh->submeshCount() || submeshIndex < 0) throw CForgeExcept("Nullpointer or Index out of range"); 
        
        // get triangles that share at leat one common vertex
        std::vector<int32_t> Rval; 
        T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(submeshIndex); 
        if(faceIndex > sub->Faces.size() || faceIndex < 0) throw CForgeExcept("faceIndex out of bounce");  

        // get the vertices of the face
        T3DMesh<float>::Face f = sub->Faces[faceIndex];
        for(int i = 0; i < 3; i++){
            std::vector<int32_t> adjFaces = getAdjacentFacesFromVertex(pMesh, submeshIndex, f.Vertices[i]); 
            for(int j = 0; j < adjFaces.size(); j++){
                // if the face is not already in the vector, then add it
                if(std::find(Rval.begin(), Rval.end(), adjFaces[j]) == Rval.end()) Rval.push_back(adjFaces[j]); 
            }
        }//for 3

        return Rval;
    }

    std::vector<int32_t> MeshConnection::getAdjacentFacesSubmesh(T3DMesh<float>* pMesh, int submeshIndex, int faceIndex){
        // difference to getNeighboringFacesSubmesh: we want to have the faces that share an edge
        
        if(pMesh == nullptr || submeshIndex >= pMesh->submeshCount() 
        || submeshIndex < 0) throw CForgeExcept("Submesh is a nullpointer or submesh out of range");

        std::vector<int32_t> Rval;
        T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(submeshIndex);
        if(faceIndex > sub->Faces.size() || faceIndex < 0) throw CForgeExcept("faceIndex out of bounce");

        // get all the faces that share at least one edge and sort them out afterwards
        T3DMesh<float>::Face face = sub->Faces[faceIndex];
        std::vector<int32_t> adjFaces = getNeighboringFacesSubmesh(pMesh, submeshIndex, faceIndex);

        for(int i = 0; i < adjFaces.size(); i++){
            T3DMesh<float>::Face f = sub->Faces[adjFaces[i]]; 
            int count = 0; 
            for(int j = 0; j < 3; j++){
                for(int k = 0; k < 3; k++){
                    if(face.Vertices[j] == f.Vertices[k]) count++; 
                }
            }
            // if they share two vertices, then they share an edge
            if(count == 2) Rval.push_back(adjFaces[i]); 
        }

        return Rval;
    }

    std::vector<FaceSubmesh> MeshConnection::getAdjacentFaces(T3DMesh<float>* pMesh, int submeshIndex, int faceIndex){
        // here: test all faces -> beyond submeshes; but still, we need to know which face of a submesh
        // this is slower than the other two functions, but it is more general
        std::vector<int32_t> Rval;

        if(pMesh == nullptr || submeshIndex >= pMesh->submeshCount() 
        || submeshIndex < 0) throw CForgeExcept("Submesh is a nullpointer or submesh out of range");
        
        T3DMesh<float>::Face f = pMesh->getSubmesh(submeshIndex)->Faces[faceIndex];

        // get all faces 
        std::vector<FaceSubmesh> facesIndex; 
        std::vector<T3DMesh<float>::Face> faces;
        for(int i = 0; i < pMesh->submeshCount(); i++){
            T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(i); 
            for(int j = 0; j < sub->Faces.size(); j++){

                if(shareEdge(f, sub->Faces[j])){
                    FaceSubmesh fs;
                    fs.faceIndex = j;
                    fs.submeshIndex = i;
                    facesIndex.push_back(fs);
                }
            }
        }

        return facesIndex; 
    }

    std::vector<int32_t> MeshConnection::getAdjacentFacesFromVertex(T3DMesh<float>* pMesh, int submeshIndex, int vertexIndex){
        if(pMesh == nullptr || submeshIndex >= pMesh->submeshCount() 
        || pMesh->vertexCount() < vertexIndex || vertexIndex < 0  || submeshIndex < 0) throw CForgeExcept("Nullpointer or Index out of range");

        // which triangles of a *submesh* are associated with a vertex? 
        // this is mostly an internal function for the function: getNeighboringFacesSubmesh()
        std::vector<int32_t> Rval; 

        T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(submeshIndex); 

        for(int i = 0; i < sub->Faces.size(); i++){
            T3DMesh<float>::Face f = sub->Faces[i]; 
            if(vertexOfFace(f, vertexIndex)) Rval.push_back(i); 
        }

        // problem: Rval can also be empty, bc the vertex is not part of the submesh but of another
        return Rval; 
    }

    std::vector<FaceSubmesh> MeshConnection::getAdjacentFacesFromVertex(T3DMesh<float>* pMesh, int vertexIndex){
        if(pMesh == nullptr || vertexIndex >= pMesh->vertexCount() || vertexIndex < 0) throw CForgeExcept("Nullpointer or Index out of range"); 

        std::vector<FaceSubmesh> Rval; 

        for(int subCount = 0; subCount < pMesh->submeshCount(); subCount++){
            T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(subCount); 
            for(int faceCount = 0; faceCount < sub->Faces.size(); faceCount++){
                T3DMesh<float>::Face face = sub->Faces[faceCount]; 
                if(vertexOfFace(face, vertexIndex)){
                    FaceSubmesh fs; 
                    fs.faceIndex = faceCount; 
                    fs.submeshIndex = subCount; 
                    Rval.push_back(fs); 
                }
            }//faces
        }// submesh

        return Rval;
    }

    std::vector<T3DMesh<float>::Submesh*> MeshConnection::getAdjacentSubmeshes(T3DMesh<float>* pMesh, int submeshIndex){
        std::vector<T3DMesh<float>::Submesh*> Rval;
        if(pMesh == nullptr || submeshIndex > pMesh->submeshCount() || submeshIndex < 0) throw CForgeExcept("Nullpointer or Index out of range");
        if(pMesh->submeshCount() == 1){
            Rval.push_back(pMesh->getSubmesh(0));
            return Rval;
        }
        // we can not do the same thing for two submeshes, bc maybe they are not connected

        std::vector<FaceSubmesh> allFaces; 
        for(int i = 0; i < pMesh->submeshCount(); i++){
            for(int j = 0; j < pMesh->getSubmesh(i)->Faces.size(); j++){
                FaceSubmesh fs; 
                fs.faceIndex = j; 
                fs.submeshIndex = i; 
                allFaces.push_back(fs); 
            }
        }

        // get border
        std::vector<int32_t> border = getEdgesSubmesh(pMesh, submeshIndex);

        return Rval;
    }

    std::vector<std::vector<int32_t>> MeshConnection::getAllBorders(T3DMesh<float>* pMesh){
        std::vector<std::vector<int32_t>> Rval; 
        if(pMesh == nullptr) throw CForgeExcept("Nullpointer");
        for(int i = 0; i < pMesh->submeshCount(); i++){
            Rval.push_back(getEdgesSubmesh(pMesh, i)); 
        }
        return Rval;
    }

    bool MeshConnection::vertexOfFace(T3DMesh<float>::Face face, int vertexIndex){
        bool Rval = false; 
        if(face.Vertices[0] == vertexIndex || face.Vertices[1] == vertexIndex || face.Vertices[2] == vertexIndex){
            Rval = true; 
        }
        return Rval; 
    }

    bool MeshConnection::shareEdge(T3DMesh<float>::Face f1, T3DMesh<float>::Face f2){
        // test if two faces share an edge
        bool Rval = false; 
        int count = 0; 
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                if(f1.Vertices[i] == f2.Vertices[j]) count++; 
            }
        }
        if(count == 2) Rval = true; 
        return Rval;
    }    

}//CForge