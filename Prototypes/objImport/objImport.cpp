#include "objImport.h"
#include <crossforge/AssetIO/SAssetIO.h>
#include <iostream>


namespace CForge{
	
	// this is just for reading a smplx file - it has just vertices and indices and comments
	// texture are not completely supported

	void objImportExport::readObjFile(std::string fileName, std::vector<Eigen::Vector3f> &vertices, std::vector<T3DMesh<float>::Face> &indices, std::vector<Eigen::Matrix<float, 3, 1>>& uvs){
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
				else if(type == "vt"){
					// vertex textur, normally has x and y coordinate
					float x, y; 
					ss >> x , y; 
					uvs.push_back(Eigen::Vector3f(x, y, 0.0f)); 
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
				else{
					// not recognized line
					continue;
				}
			}
		}
		else{
			throw CForgeExcept("File not found");
		}

	}// readObjFile


   void objImportExport::storeInMesh(std::string fileName, T3DMesh<float>* mesh){
		std::vector<Eigen::Vector3f> vertices;
		std::vector<T3DMesh<float>::Face> indices; 
		std::vector<Eigen::Matrix<float, 3, 1>> uvs; 
		objImportExport::readObjFile(fileName, vertices, indices, uvs);
		mesh->init(); 

		mesh->vertices(&vertices);
		mesh->textureCoordinates(&uvs);  
		int materialID = 0; 
		
		// create submesh
		T3DMesh<float>::Submesh submesh;
		submesh.Material = materialID;
		submesh.Faces = indices;
		mesh->addSubmesh(&submesh, true);

		// create material
		T3DMesh<float>::Material material; 
		material.ID = materialID; 
		
		mesh->addMaterial(&material, true);
	}// storeInMesh

	void objImportExport::exportAsObjFile(std::string fileName, T3DMesh<float>* mesh){
		// export the mesh to an obj file

		// first check whether the mesh has vertices
		if(mesh->vertexCount() == 0 || mesh == nullptr) return; 

		// second check whether the filename ends with .obj
		std::string extension = ".obj";
		if(fileName.rfind(extension) != fileName.size() - extension.size()){
			std::cout<<"File name does not end with \".ob\"!"<<std::endl;
			return; 
		}

		std::ofstream file(fileName, std::ostream::out | std::ostream::trunc);
		std::string ret; 

		// write - with a comment
		ret.append("# Exported from CForge");
		
		std::vector<std::string> materialNames;
		for(int i = 0; i < mesh->submeshCount(); i++){
			int32_t submeshMaterial = mesh->getSubmesh(i)->Material; 
			T3DMesh<float>::Material *m =  mesh->getMaterial(submeshMaterial);
			
			if(m->TexAlbedo == ""){
				materialNames.push_back("");
			} else {
				// get rid of the "/" and the "." in the path
				size_t lastSlashPos = m->TexAlbedo.find_last_of("/");
				size_t dotPos = m->TexAlbedo.find_last_of(".");
				std::string path = m->TexAlbedo.substr(lastSlashPos + 1, dotPos - lastSlashPos - 1);
				materialNames.push_back(path);
				ret.append("\nmtllib " + path + ".mtl");
			}
		}

		// write vertices
		for(int i = 0; i < mesh->vertexCount(); i++){
			Eigen::Vector3f v = mesh->vertex(i);
			ret.append("\nv "   + std::to_string(v.x()) + " " 
								+ std::to_string(v.y()) + " " 
								+ std::to_string(v.z()));
		}	

		// export the vertex normals
		for(int i = 0; i < mesh->normalCount(); i++){
			Eigen::Vector3f vn = mesh->normal(i);
			ret.append("\nvn "	  + std::to_string(vn.x()) + " " 
									+ std::to_string(vn.y()) + " " 
									+ std::to_string(vn.z()));
		}

		// write the textur coordinates
		for(int i = 0; i < mesh->textureCoordinatesCount(); i++){
			Eigen::Vector3f vt = mesh->textureCoordinate(i); 
			ret.append("\nvt " + std::to_string(vt.x()) + " " + std::to_string(vt.y()));
		}

		// expor the faces - define material beforehand 
		for(int i = 0; i < mesh->submeshCount(); i++){
			T3DMesh<float>::Submesh *submesh = mesh->getSubmesh(i); 

			//if(materialNames[i] == ""){
			//	continue;
			//}
			//ret.append("\nusemtl " + materialNames[i]);
			
			for(int j = 0; j < submesh->Faces.size(); j++){
				T3DMesh<float>::Face face = submesh->Faces[j]; 
				
				if(mesh->textureCoordinatesCount() == 0){
					ret.append("\nf "	+ std::to_string(face.Vertices[0] + 1) + " " 
										+ std::to_string(face.Vertices[1] + 1) + " " 
										+ std::to_string(face.Vertices[2] + 1));
				} // if no texture coordinates
				else {
					ret.append("\nf "	+ std::to_string(face.Vertices[0] + 1)+"/"+std::to_string(face.Vertices[0] + 1) + " " 
										+ std::to_string(face.Vertices[1] + 1)+"/"+std::to_string(face.Vertices[1] + 1) + " " 
										+ std::to_string(face.Vertices[2] + 1)+"/"+std::to_string(face.Vertices[2] + 1));

				} // if texture coordinates
			} // for each face
		}// for each submesh
		
		ret.append("\n");
		file << ret;
		
	}// exportAsObjFile

	void objImportExport::exportSubmeshesAsObjFiles(std::vector<std::string> filenames, T3DMesh<float>* pMesh){
		// this funciton exports each submesh as a separate obj file
		if(pMesh == nullptr) throw CForgeExcept("Mesh is null");
		if(filenames.size() != pMesh->submeshCount()){
			filenames.clear();
			for(int i = 0; i < pMesh->submeshCount(); i++){
				filenames.push_back("submesh" + std::to_string(i) + ".obj");
			}
		} 
		
		std::vector<Eigen::Vector3f> vertices; 
		for(int i = 0; i < pMesh->vertexCount(); i++){
			vertices.push_back(pMesh->vertex(i));
		}
		std::vector<Eigen::Vector3f> textureCoordinates;
		for(int i = 0; i < pMesh->textureCoordinatesCount(); i++){
			textureCoordinates.push_back(pMesh->textureCoordinate(i));
		}

		T3DMesh<float> exportMesh; 
		for(int i = 0; i < pMesh->submeshCount(); i++){
			exportMesh.clear(); 
			exportMesh.vertices(&vertices);
			exportMesh.textureCoordinates(&textureCoordinates); 
			T3DMesh<float>::Submesh *submesh = pMesh->getSubmesh(i);
			std::vector<T3DMesh<float>::Face> faces = submesh->Faces;

			// if we merge the mesh beforehand it can be that the mesh has no
			// faces, so we just skip that mesh and proceed with the next one
			if(faces.size() == 0) continue;

			exportMesh.addSubmesh(submesh, true);

			T3DMesh<float>::Material *material = pMesh->getMaterial(submesh->Material);
			for(int i = 0; i < pMesh->submeshCount(); i++){
				exportMesh.addMaterial(material, true);
			}
			

			std::string filename = filenames[i];

			// there can still be a problem with the material of the mesh 
			// according to f3d: "material '' appears in OBJ but not MTL file?"
			SAssetIO::store(filename, &exportMesh);
		}
	}

}// namespace CForge