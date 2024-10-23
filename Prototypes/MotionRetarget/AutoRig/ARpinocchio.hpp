#pragma once

#include "IAutoRigger.hpp"
#include <Thirdparty/Pinocchio/PinocchioTools.hpp>
#include <Thirdparty/Pinocchio/skeleton.h>

namespace nsPiT = nsPinocchioTools;

namespace CForge {
using namespace Eigen;

struct ARpinocchioOptions {

};

class ARpinocchio : IAutoRigger<ARpinocchioOptions> {
public:

	void rig(T3DMesh<float>* mesh, ARpinocchioOptions options) {
		nsPiT::CVScalingInfo cvsInfo;

		nsPinocchio::Skeleton skl = nsPinocchio::HumanSkeleton(); //TODOff(skade) other predefined skl in skeleton.h
		nsPinocchio::Mesh piM;// = new nsPinocchio::Mesh(); //("MyAssets/muppetshow/Model1.obj"); //TODOff(skade) test original
		nsPiT::convertMesh(*mesh, &piM);
		nsPinocchio::PinocchioOutput rig = nsPiT::autorig(skl, &piM);

		//TODOff(skade) fix bones, wrong
		std::vector<T3DMesh<float>::Bone*> bones;
		{ // create bone list
			for (auto [name, idx] : skl.jointNames) {
				bones.push_back(new T3DMesh<float>::Bone());
				T3DMesh<float>::Bone* b = bones.back();
				b->ID = idx;
				b->Name = name;

				//TODOff(skade) construct invBindPose mat
				nsPinocchio::Vector3 piPos = skl.fGraph().verts[idx];
				Vector3f pos = Vector3f(piPos[0],piPos[1],piPos[2]);
				b->InvBindPoseMatrix = Matrix4f::Identity();
				b->InvBindPoseMatrix.block<3,1>(0,3) = pos;
			}

			// assign parent and children
			for (auto [name, idx] : skl.jointNames) {
				// get bone
				T3DMesh<float>::Bone* b = nullptr;
				for (uint32_t i = 0; i < bones.size(); ++i) {
					if (bones[i]->ID == idx) {
						b = bones[i];
						break;
					}
				}
				assert(b);
				assert(name == b->Name);
				
				// assign parent
				int parIdx = skl.fPrev()[idx];
				for (uint32_t i = 0; i < bones.size(); ++i) {
					if (bones[i]->ID == parIdx) {
						b->pParent = bones[i];
						break;
					}
				}
				
				// find children
				for (auto [_,idx2] : skl.jointNames) {
					int parIdx2 = skl.fPrev()[idx2];
					if (parIdx2 == idx) {
						for (uint32_t i = 0; i < bones.size(); ++i) {
							if (bones[i]->ID == idx2) {
								b->Children.push_back(bones[i]);
								break;
							}
						}
					}
				}
			}
		}

		{ // create bones
			mesh->bones(&bones,false);
			mesh->clearSkeletalAnimations();
			for (uint32_t i = 0; i < mesh->boneCount(); i++) {
				mesh->getBone(i)->VertexInfluences = std::vector<int32_t>();
				mesh->getBone(i)->VertexWeights = std::vector<float>();
			}
		}

		if (rig.attachment)
			nsPiT::applyWeights(&skl, &piM, mesh, cvsInfo, rig, mesh->vertexCount());
		else
			std::cerr << "ARpinocchio::rig failed, rig contains no attachment." << std::endl; //TODOff(skade)

		// do not delete, we dont copy bones
		//for (uint32_t i=0;i<bones.size();++i)
		//	delete bones[i];
	};
};

}//CForge
