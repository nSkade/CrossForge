#include "objImport.h"


namespace CForge{
    
    // this is just for reading a smplx file - it has just vertices and indices and comments
    void objImport::readObjFile(std::string fileName, std::vector<Eigen::Vector3f> &vertices, std::vector<T3DMesh<float>::Face> &indices){
        // read a file
        std::ifstream file(fileName);
        
        std::string line;

        // https://github.com/nSkade/einherjarEngine/blob/develop/src/Mesh.cpp
        // https://en.cppreference.com/w/cpp/io/basic_stringstream
        
        if(file.good()){
            while(std::getline(file, line)){
                std::stringstream ss(line);
                std::string type;
                ss >> type;
                if (type == "#") continue; // comment
                else if(type == "v"){
                    Eigen::Vector3f vertex;
                    ss >> vertex[0] >> vertex[1] >> vertex[2];
                    vertices.push_back(vertex);
                }
                else if(type == "f"){
                    T3DMesh<float>::Face face;
                    ss >> face.Vertices[0] >> face.Vertices[1] >> face.Vertices[2];
                    // obj files are 1-indexed so we need to subtract 1
                    face.Vertices[0] -= 1;
                    face.Vertices[1] -= 1;
                    face.Vertices[2] -= 1;
                    indices.push_back(face);
                }
            }
        }
        else{
            throw CForgeExcept("File not found");
        }

    }


   void objImport::storeInMesh(std::string fileName, T3DMesh<float>* mesh){
        std::vector<Eigen::Vector3f> vertices;
        std::vector<T3DMesh<float>::Face> indices; 
        objImport::readObjFile(fileName, vertices, indices);
        mesh->init(); 

        mesh->vertices(&vertices); 
        int materialID = 0; 
        
        // create submesh
        T3DMesh<float>::Submesh submesh;
        submesh.Material = materialID;
        submesh.Faces = indices;
        mesh->addSubmesh(&submesh, true);

        // create material
        T3DMesh<float>::Material material;
        material.init(); 
        material.ID = materialID; 
        
        mesh->addMaterial(&material, true);
    }

}// namespace CForge