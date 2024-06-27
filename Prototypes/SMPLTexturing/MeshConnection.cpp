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

    std::vector<int32_t> MeshConnection::getIsolatedFacesSubmesh(T3DMesh<float>* pMesh, int submeshIndex){
        if(pMesh == nullptr || submeshIndex >= pMesh->submeshCount() || submeshIndex < 0) throw CForgeExcept("Nullpointer or Index out of range"); 

        std::vector<int32_t> Rval; 
        T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(submeshIndex); 

        for(int i = 0; i < sub->Faces.size(); i++){
            if(getAdjacentFacesSubmesh(pMesh, submeshIndex, i).size() == 0) Rval.push_back(i); 
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

    float MeshConnection::getAreaFace(T3DMesh<float>* pMesh, int submeshIndex, int faceIndex){
        if(pMesh == nullptr || submeshIndex >= pMesh->submeshCount() || submeshIndex < 0) throw CForgeExcept("Nullpointer or Index out of range"); 
        T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(submeshIndex); 
        if(faceIndex >= sub->Faces.size() || faceIndex < 0) throw CForgeExcept("faceIndex out of range"); 

        T3DMesh<float>::Face f = sub->Faces[faceIndex];
        Eigen::Vector3f v1 = pMesh->vertex(f.Vertices[0]);
        Eigen::Vector3f v2 = pMesh->vertex(f.Vertices[1]);
        Eigen::Vector3f v3 = pMesh->vertex(f.Vertices[2]);

        // get the area of the triangle
        Eigen::Vector3f e1 = v2 - v1;
        Eigen::Vector3f e2 = v3 - v1;
        float area = 0.5 * e1.cross(e2).norm();
        return area;
    }

    float MeshConnection::getAreaSubmesh(T3DMesh<float>* pMesh, int submeshIndex){
        if(pMesh == nullptr || submeshIndex >= pMesh->submeshCount() || submeshIndex < 0) throw CForgeExcept("Nullpointer or Index out of range"); 
        T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(submeshIndex); 

        float area = 0.0f; 
        for(int i = 0; i < sub->Faces.size(); i++){
            area += MeshConnection::getAreaFace(pMesh, submeshIndex, i); 
        }
        return area;
    }

    std::vector<T3DMesh<float>::Face> MeshConnection::getIsolatedFacesWholeMesh(T3DMesh<float>* pMesh){
        if(pMesh == nullptr) throw CForgeExcept("Nullpointer"); 
        std::vector<T3DMesh<float>::Face> Rval; 
        for(int i = 0; i < pMesh->submeshCount(); i++){
            std::vector<int32_t> isolatedFaces = getIsolatedFacesSubmesh(pMesh, i); 
            T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(i); 
            for(int j = 0; j < isolatedFaces.size(); j++){
                Rval.push_back(sub->Faces[isolatedFaces[j]]); 
            }
        }
        return Rval;
    }

    bool MeshConnection::areSameFace(T3DMesh<float>::Face f1, T3DMesh<float>::Face f2){
        // test if two faces are the same
        bool Rval = f1.Vertices[0] == f2.Vertices[0] && f1.Vertices[1] == f2.Vertices[1] && f1.Vertices[2] == f2.Vertices[2];
        return Rval;
    }

    int MeshConnection::getFaceIndex(T3DMesh<float>* pMesh, int submeshIndex, T3DMesh<float>::Face face){
        // get the index of a facevalue in a submesh
        int Rval = -1; 

        if(pMesh == nullptr || submeshIndex >= pMesh->submeshCount() || submeshIndex < 0) throw CForgeExcept("Nullpointer or Index out of range");
        T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(submeshIndex);
        for(int i = 0; i < sub->Faces.size(); i++){
            if(areSameFace(sub->Faces[i], face)){
                Rval = i; 
                break;
            }
        }
        return Rval;
    }

    T3DMesh<float> MeshConnection::splitMeshFromProxy(T3DMesh<float>* pMeshOrg, T3DMesh<float>* pMeshRegions, std::vector<std::vector<int32_t>> vertexMap){
        if(pMeshOrg == nullptr || pMeshRegions == nullptr) throw CForgeExcept("Nullpointer");
        T3DMesh<float> Rval;

        // create a new T3DMesh with the same vertices and texture coordinates
        // but split them up like the regionmesh

        // get verticies & texture vertices
        std::vector<Eigen::Vector3f> vertices; 
        for(int i = 0; i < pMeshOrg->vertexCount(); i++){
            vertices.push_back(pMeshOrg->vertex(i)); 
        }
        std::vector<Eigen::Vector3f> texturVerices; 
        for(int i = 0; i < pMeshOrg->textureCoordinatesCount(); i++){
            texturVerices.push_back(pMeshOrg->textureCoordinate(i)); 
        }
        Rval.vertices(&vertices);
        Rval.textureCoordinates(&texturVerices);  

        // a submesh can have multible materials which are empty, we just take the first none empty one
        // this should hold the texture itself
        int materialID = -1; 
        T3DMesh<float>::Material *textureMaterial;    
        for(int i = 0; i < pMeshOrg->materialCount(); i++){
            T3DMesh<float>::Material *matOrg = pMeshOrg->getMaterial(i);
            if(!matOrg->TexAlbedo.empty()){
                textureMaterial = pMeshOrg->getMaterial(i); 
                materialID = i; 
                break; 
            } 
        }

        // this is a necessary step, because later it is checked how many materials in total
        // the mesh has and it needs one per submesh
        for(int i = 0; i < pMeshRegions->submeshCount(); i++){
            Rval.addMaterial(textureMaterial, true); 
        }

        // submehes of pMeshRegions
        for(int i = 0; i < pMeshRegions->submeshCount(); i++){
            T3DMesh<float>::Submesh *subRegions = pMeshRegions->getSubmesh(i); 
            T3DMesh<float>::Submesh *newSub = new T3DMesh<float>::Submesh; 

            // transfer the faces - that means get a new face, 
            for(int faceIdx = 0; faceIdx < subRegions->Faces.size(); faceIdx++){
                T3DMesh<float>::Face f = subRegions->Faces[faceIdx]; 
                T3DMesh<float>::Face newFace; 
                for(int j = 0; j < 3; j++){
                    // vertex index of face
                    // can put 0 there because the vertices are the same
                    newFace.Vertices[j] = vertexMap[f.Vertices[j]][0]; 
                }
                newSub->Faces.push_back(newFace); 
            }
            newSub->Material = materialID; 
            Rval.addSubmesh(newSub, true); 
        }

        return Rval;
    }

    void MeshConnection::copyMesh(T3DMesh<float>* pMeshOrg, T3DMesh<float>* pMeshCopy){
        // a real copy function in the T3DMesh is missing 
        
        // we do not want to add anything but build it up from scratch
        pMeshCopy->clear();

        // copying the region mesh to m_RegionMesh
			std::vector<Eigen::Vector3f> vertices;
            std::vector<Eigen::Vector3f> textureCoordinates;
			std::vector<T3DMesh<float>::Face> faces;
			for(int i = 0; i < pMeshOrg->vertexCount(); i++){
				vertices.push_back(pMeshOrg->vertex(i));
			}
			pMeshCopy->vertices(&vertices);
            for(int i = 0; i < pMeshOrg->textureCoordinatesCount(); i++){
                textureCoordinates.push_back(pMeshOrg->textureCoordinate(i));
            }
            pMeshCopy->textureCoordinates(&textureCoordinates);
			for(int i = 0; i < pMeshOrg->submeshCount(); i++){
				T3DMesh<float>::Submesh *sub = pMeshOrg->getSubmesh(i); 
				std::vector<T3DMesh<float>::Face> faces;
				faces = sub->Faces;

                // through merging the mesh, it can be that the mesh has no faces
                if(faces.empty()) continue;

                pMeshCopy->addSubmesh(sub, true);	
			}
			for(int i = 0; i < pMeshOrg->materialCount(); i++){
				T3DMesh<float>::Material *mat = pMeshOrg->getMaterial(i); 
				pMeshCopy->addMaterial(mat, true);	
			}
			// copying end 

    }
    
}//CForge