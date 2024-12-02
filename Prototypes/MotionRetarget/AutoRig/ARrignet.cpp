#include "ARrignet.hpp"

#include <Prototypes/objImport/objImport.h>

#include <filesystem>
#include <Prototypes/MotionRetarget/CMN/MergeVertices.hpp>

#include <Prototypes/MotionRetarget/CMN/EigenFWD.hpp>
#include <iostream> //TODOff(skade) SLogger

#include <Prototypes/MotionRetarget/CMN/MergeVertices.hpp>

namespace CForge {
using namespace Eigen;

void ARrignet::rig(T3DMesh<float>* mesh, ARrignetOptions options) {
	std::string objPath = "MyAssets/Cache/Rignet/";

	// merge vertices
	T3DMesh<float> mergedMesh = *mesh;

	// remove redundant vertices for rignet and create correspondences to assign weights to original mesh
	std::vector<uint32_t> vertCorr = mergeRedundantVertices(&mergedMesh);

	// compute inverted vertCorr, new vert to old verts
	std::map<uint32_t,std::vector<uint32_t>> vertCorrI;
	for (uint32_t i=0;i<vertCorr.size();++i)
		vertCorrI[vertCorr[i]].push_back(i);

	// delete cache folder
	if (!options.parseOutputOnly) {
		std::filesystem::remove_all(objPath);
		std::filesystem::create_directories(objPath);

		// export file for script
		objImportExport::exportAsObjFile(objPath+"mesh.obj", &mergedMesh);

		// assemble command

		//TODOfff(skade) linux support / crossplatform
		//TODOfff(skade) no space allowable?
		//std::string command = "\""+condaPath + "/Scripts/activate.bat\" " + "\""+condaPath+"/\"";
		std::string command = condaPath + "/Scripts/activate.bat " + condaPath;

		//std::filesystem::remove(std::filesystem::path(objPath));

		// commands
		command.append(" & conda activate rignet");
		command.append(" & conda info");

		std::string exePath = std::filesystem::current_path().string();

		command.append(" & cd \""+rignetPath + "/\"");
		command.append(" & python quick_start.py");

		// rignet arg
		command.append(" \"" + exePath + "/" + objPath + "\""); // input folder
		command.append(" mesh"); // filename
		command.append(" " + std::to_string(options.bandwidth)); // bandwidth
		command.append(" " + std::to_string(options.threshold)); // treshold

		//command.append(" & cd"); // cd is exe root

		std::cout << command << std::endl;

		//TODO(skade) option to not run script but parse cache instead
		// run script
		std::system(command.c_str());
	}

	// parse output of script
	struct RNrig {
		std::vector<std::pair<std::string,Vector3f>> joints;
		std::string root;

		// per joint name
		std::map<std::string,std::vector<std::pair<int,float>>> weights; // weights per joint
		std::map<std::string,std::vector<std::string>> hier; // hierarchy
	} rig;

	std::ifstream ifs(objPath+"/mesh_rig.txt");
	std::string line;
	while (std::getline(ifs,line)) {
		std::istringstream iss(line);
		std::string type; iss >> type;
		if (type == "joints") {
			std::string jointName;
			Vector3f v;
			iss >> jointName >> v[0] >> v[1] >> v[2];
			rig.joints.push_back({jointName,v});
		} else if (type == "root") {
			std::string rootName;
			iss >> rootName;
			rig.root = rootName;
		} else if (type == "skin") {
			int skinIndex;
			std::string jointName;
			float weight;
			iss >> skinIndex;
			while (iss >> jointName >> weight)
				rig.weights[jointName].push_back({skinIndex,weight});
		} else if (type == "hier") {
			std::string parent, child;
			iss >> parent >> child;
			rig.hier[parent].push_back(child);
		} else { // Handle unknown line type
			std::cout << "Unknown line type: " << line << std::endl;
			__debugbreak();
		}
	}

	ifs.close();

	std::vector<T3DMesh<float>::Bone*>* bones = new std::vector<T3DMesh<float>::Bone*>();
	std::vector<Vector3f> bonesPos;

	for (int i=0;i<rig.joints.size();++i) {
		auto& [name,pos] = rig.joints[i];
		T3DMesh<float>::Bone* b = new T3DMesh<float>::Bone();
		b->clear();

		b->ID = i;
		b->Name = name;

		std::vector<int32_t> influences;
		std::vector<float> weights;

		for(auto& [idx,w] : rig.weights[name]) {
			for (auto oIdx : vertCorrI[idx]) {
				influences.emplace_back(oIdx);
				weights.emplace_back(w);
			}
		}

		//// sort biggest weight first
		//std::vector<std::pair<int32_t,float>> pairs;
		//for (uint32_t i=0;i<influences.size();++i)
		//	pairs.push_back({influences[i],weights[i]});

		//std::sort(pairs.begin(), pairs.end(),
		//	[](const std::pair<int32_t, float>& a, const std::pair<int32_t, float>& b) {
		//	return a.second > b.second;});

		//influences.clear();
		//weights.clear();
		//for (auto [i,w] : pairs) {
		//	influences.push_back(i);
		//	weights.push_back(w);
		//}

		b->VertexInfluences = influences;
		b->VertexWeights = weights;

		bones->emplace_back(b);
		bonesPos.push_back(pos);
	}

	// create links
	for (int i=0;i<bones->size();++i) {
		auto& rChildren = rig.hier[bones->at(i)->Name];
		for (auto rChild : rChildren) {
			// find child pointer
			auto it = std::find_if(bones->begin(),bones->end(),
			          [&rChild](const T3DMesh<float>::Bone* b){ return b->Name == rChild; });
			if (it != bones->end()) {
				bones->at(i)->Children.push_back(*it);
				(*it)->pParent = bones->at(i);
			}
		}
	}

	// comp InvBindPose
	for (int i=0;i<bones->size();++i) {
		auto b = bones->at(i);
		Vector3f pos = bonesPos[i];

		//TODOf(skade) unify with JointPickable::update
		Quaternionf LR = Quaternionf::Identity();
		if (b->Children.size() > 0) {
			Vector3f BoneVec = bonesPos[b->Children[0]->ID] - pos;
			float Length = BoneVec.norm(); // length to next bone
			LR = EigenFWD::FromTwoVectors(Vector3f::UnitX(), BoneVec.normalized()); // obj Joint points to +x axis
		}
		LR.normalize();

		Matrix4f bindPose = Matrix4f::Identity();
		bindPose.block<3,1>(0,3) = pos;
		bindPose.block<3,3>(0,0) = LR.toRotationMatrix();
		b->InvBindPoseMatrix = bindPose.inverse();
	}

	// add rig to mesh
	mesh->bones(bones,false);
};

}//CForge
