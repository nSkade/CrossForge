/*****************************************************************************\
*                                                                           *
* File(s): ExampleBAMaterial.hpp                                        			*
*                                                                           *
* Content: Example of two Meshes (Timon and SMPLX) where we want to         *
*          rotate Timon to SMPLX via ICP and hausdorff as close as possible *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann, Niclas Meyer                                      *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/

#pragma once 

#include "ExampleSceneBase.hpp"
#include <Prototypes/SMPLTexturing/ICP.h> 
#include <Prototypes/asicp/asicp.hxx>

#include "json/json.h"
#include <Prototypes/JsonBone/JsonBoneRead.h>
#include <Prototypes/objImport/objImport.h>
#include <Prototypes/PNGimport/pngImport.h>
#include <Prototypes/SMPLTexturing/ICP.h>
#include <Prototypes/SMPLTexturing/MeshConnection.h>

#include <iostream>
#include <fstream>
#include <algorithm>

using namespace Eigen;
using namespace std;

namespace CForge {

	class ExampleBAMaterials : public ExampleSceneBase {
	public:
		
		// these are all the regions
		enum BoneGroups{ 
			HEAD,
			TORSO,
			FOOT, 
			LEG, 
			ARM,
			HAND
		};

		struct BorderNeighborhood{
			int id; 
			BoneGroups group; 
			std::vector<std::vector<int32_t>> neighboringFaces;
			std::vector<int32_t> border; 
		}; 

		struct ParentChildBorder{
			std::vector<int32_t> parentBorder; 
			std::vector<int32_t> childBorder;
		}; 

		ExampleBAMaterials(void) {
			m_WindowTitle = "CrossForge Example - ICP";
			m_WinWidth = 1280;
			m_WinHeight = 720;
		}//Constructor

		~ExampleBAMaterials(void) {
			clear();
		}//Destructor

		void init() override{

			initWindowAndRenderDevice();
			initCameraAndLights();
			initFPSLabel();

            m_Cam.position(Vector3f(0.0f, 1.5f, 4.0f));

			// build scene graph
			m_RootSGN.init(nullptr);
			m_SG.init(&m_RootSGN);

			// load SMPLX model
			// SAssetIO::load("MyAssets/010.obj", &m_SMPLXMesh);
			objImportExport::storeInMesh("MyAssets/010.obj", &m_ProxyMesh);
			SAssetIO::load("MyAssets/Reko_Timo/test_own_export_from_python.obj", &m_SMPLXMesh); 

            m_SMPLXMesh.computePerVertexNormals();
			m_SMPLXMesh.computeAxisAlignedBoundingBox();

			m_ProxyMesh.computePerVertexNormals(); 
			m_ProxyMesh.computePerFaceNormals(); 

			// std::vector<Eigen::Vector3f>>* color; 
			// m_ProxyMesh.colors(); 
            m_Reconstruction.init(&m_SMPLXMesh);
			m_ReconstructionTransformSGN.init(&m_RootSGN); // Vector3f(0.0f, 0.0f, 0.0f)
			
            m_ReconstructionTransformSGN.translation(Vector3f(0.0f, 1.25f, 0.0f));
			m_ReconstructionTransformSGN.rotation(Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), Vector3f::UnitY())));
            m_ReconstructionSGN.init(&m_ReconstructionTransformSGN, &m_Reconstruction);

			const std::string fileName = "MyAssets/result.json"; 
			T3DMesh<float> regionMesh = getRegionalModel(fileName, &m_ProxyMesh);
			
			// copying the region mesh to m_RegionMesh
			std::vector<Vector3f> vertices;
			for(int i = 0; i < regionMesh.vertexCount(); i++){
				vertices.push_back(regionMesh.vertex(i));
			}
			m_RegionMesh.vertices(&vertices);
			for(int i = 0; i < regionMesh.submeshCount(); i++){
				T3DMesh<float>::Submesh *sub = regionMesh.getSubmesh(i); 
				m_RegionMesh.addSubmesh(sub, true);	
			}
			for(int i = 0; i < regionMesh.materialCount(); i++){
				T3DMesh<float>::Material *mat = regionMesh.getMaterial(i); 
				m_RegionMesh.addMaterial(mat, true);	
			}
			// copying end 
			
			m_RegionMesh.computePerVertexNormals(); 
			m_RegionMesh.computePerFaceNormals(); 

			m_RegionActor.init(&m_RegionMesh); 
			m_RegionActorTransformSGN.init(&m_RootSGN); 
			m_RegionActorSGN.init(&m_RegionActorTransformSGN, &m_RegionActor); 
			m_RegionActorTransformSGN.rotation(Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), Vector3f::UnitY())));
			m_RegionActorTransformSGN.translation(Vector3f(0.0f, -0.5f, 0.0f));

			// create help text
			LineOfText* pKeybindings = new LineOfText();
			pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Movement: (Shift) + W,A,S,D  | Rotation: LMB/RMB + Mouse | F1: Toggle help text | 1: ICP | shift+1: scaling ICP");
			m_HelpTexts.push_back(pKeybindings);
			m_DrawHelpTexts = true;
			m_ScreenshotExtension = "png";

			m_ClearSky.push_back("Assets/ExampleScenes/skybox/bluecloud_rt.jpg");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/bluecloud_lf.jpg");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/bluecloud_up.jpg");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/bluecloud_dn.jpg");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/bluecloud_ft.jpg");
			m_ClearSky.push_back("Assets/ExampleScenes/skybox/bluecloud_bk.jpg");

			m_Skybox.init(m_ClearSky[0], m_ClearSky[1], m_ClearSky[2], m_ClearSky[3], m_ClearSky[4], m_ClearSky[5]);

			// set initialize color adjustment values
			m_Skybox.brightness(1.15f);
			m_Skybox.contrast(1.1f);
			m_Skybox.saturation(1.2f);

			// create scene graph for the Skybox
			m_SkyboxTransSGN.init(nullptr);
			m_SkyboxGeomSGN.init(&m_SkyboxTransSGN, &m_Skybox);
			m_SkyboxSG.init(&m_SkyboxTransSGN);

			// std::string ErrorMsg;
			// if (0 != CForgeUtility::checkGLError(&ErrorMsg)) {
			// 	SLogger::log("OpenGL Error" + ErrorMsg, "PrimitiveFactoryTestScene", SLogger::LOGTYPE_ERROR);
			// } 

			// later for floatfill
			m_textur.readPNGFile("MyAssets/Reko_Timo/Timon_A_20k.png"); 
			m_correspondencies = countDoubleVerticesKDTree(&m_ProxyMesh, &m_SMPLXMesh, &m_correspondenciesNotInverted);

			//getNeigborhoodRegionMesh(&m_RegionMesh, &timon);

		}//initialize

		void clear(void) override{
			m_RenderWin.stopListening(this);
			if (nullptr != m_pShaderMan) m_pShaderMan->release();
			m_pShaderMan = nullptr;
		}//clear

		void mainLoop(void)override {
			m_RenderWin.update();
			m_SG.update(60.0f / m_FPS);

			defaultCameraUpdate(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse());

			m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
			m_RenderDev.activeCamera(const_cast<VirtualCamera*>(m_Sun.camera()));
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
			m_RenderDev.activeCamera(&m_Cam);
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
			// Skybox should be last thing to render
			m_SkyboxSG.render(&m_RenderDev);

			m_FPSLabel.render(&m_RenderDev);
			if (m_DrawHelpTexts) drawHelpTexts();

			m_RenderWin.swapBuffers();

			updateFPS();

			defaultKeyboardUpdate(m_RenderWin.keyboard());
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_1)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_1, Keyboard::KEY_RELEASED); 
				moveBorderMesh(&m_RegionMesh, BoneGroups::HEAD, BoneGroups::TORSO);
				m_RegionActor.init(&m_RegionMesh);
			}
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_2)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_2, Keyboard::KEY_RELEASED); 
				floatfill(&m_RegionMesh, &m_textur);
				m_RegionActor.init(&m_RegionMesh);
			}
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_J)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_J, Keyboard::KEY_RELEASED); 
                Eigen::Vector3f trans = m_ReconstructionTransformSGN.translation() + Eigen::Vector3f(0.125f, 0.0f, 0.0f);
                m_ReconstructionTransformSGN.translation(trans); 
            }
            if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_L)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_L, Keyboard::KEY_RELEASED);

                Eigen::Vector3f trans = m_ReconstructionTransformSGN.translation() - Eigen::Vector3f(0.125f, 0.0f, 0.0f);
                m_ReconstructionTransformSGN.translation(trans);
            }

		}//mainLoop

		void floatfill(T3DMesh<float> *pMesh, PNGImport *textur){
			if(pMesh->submeshCount() != m_allRegionTypes.size()) throw CForgeExcept("Region Mesh has not the same number of regions as the hard coded ones");
			if(pMesh == nullptr) throw CForgeExcept("Mesh is nullptr");
			if(textur == nullptr || !textur->isPictureLoaded()) throw CForgeExcept("Texture is nullptr or not existing");
			

			// 1. pick the neighboring regions
			for(int parentID = 0; parentID < m_NeighborshipOneWay.size(); parentID++){
				BoneGroups parentGroup = m_allRegionTypes[parentID];
				std::vector<BoneGroups> childVector = m_NeighborshipOneWay.at(parentGroup);

				// 1.1. get submesh of parent
				T3DMesh<float>::Submesh *subParent = pMesh->getSubmesh(parentID);

				for(int childID = 0; childID < childVector.size(); childID++){
					// 1.2. get submesh of child
					BoneGroups childGroup = childVector.at(childID);
					T3DMesh<float>::Submesh *subChild = pMesh->getSubmesh(childGroup);

					// 2. prepare the border of the parent and the child
					std::vector<std::vector<int32_t>> neighboringFacesParent, neighboringFacesChild;
					std::vector<int32_t> parentBorder, childBorder;
					BorderNeighborhood parent, child; 
					ParentChildBorder borderRegions;

					int counter = 0; 
					double colorDifference = 0, oldColorDifference = 0;
					
					do {
						// 2.1 get the border of the parent and the child
						parentBorder = MeshConnection::getEdgesSubmesh(pMesh, parentGroup, &neighboringFacesParent);
						childBorder = MeshConnection::getEdgesSubmesh(pMesh, childGroup, &neighboringFacesChild);
						parent = BorderNeighborhood{parentGroup, parentGroup, neighboringFacesParent, parentBorder};
						child = BorderNeighborhood{childGroup, childGroup, neighboringFacesChild, childBorder};
						borderRegions = detectCommonBorder(pMesh, &parent, &child);

						// 3. get the color of the borders
						oldColorDifference = colorDifference;
						colorDifference = colorDifferenceChildParent(pMesh, textur, &parent, &child, &borderRegions);

						// 4. if color under threshold -> add to parent and remove from child
						if(colorDifference < m_thresholdDeltaColor){
							addAndRemoveFacesMesh(pMesh, &parent, &child, &borderRegions);
							std::cout << "Parent: " << parentID << "| Child: " << childID << " | Color Difference: " << colorDifference << std::endl;
							counter++;
						}
						else{
							std::cout << "Color Difference too high: " << colorDifference << std::endl;
							counter = 0; 
						}

					// 5. rinse and repeat
					} while (colorDifference < m_thresholdDeltaColor && counter < m_maxFloatFillItteration);
					
				}// childID
			}// parentID

			// serach for isolated faces and add them to the according region

		}

		void moveBorderMesh(T3DMesh<float> *pMesh, BoneGroups parentID, BoneGroups childID){
			// just for testing - I've noticed that the border removal is not as seamless as I thought
			// idea is, to remove the border between the head and the torso and to add the torso border to the head

			if(pMesh->submeshCount() != m_allRegionTypes.size()) throw CForgeExcept("Region Mesh has not the same number of regions as the hard coded ones");
			if(pMesh == nullptr) throw CForgeExcept("Mesh is nullptr");

			// test with the head and torso -> they share a border
			// BoneGroups parentID = BoneGroups::HEAD, childID = BoneGroups::TORSO;

			std::vector<std::vector<int32_t>> neighboringFacesParent, neighboringFacesChild;
			std::vector<int32_t> parentBorder = MeshConnection::getEdgesSubmesh(pMesh, parentID, &neighboringFacesParent);
			std::vector<int32_t> childBorder = MeshConnection::getEdgesSubmesh(pMesh, childID, &neighboringFacesChild);
			BorderNeighborhood parent = BorderNeighborhood{parentID, parentID, neighboringFacesParent, parentBorder};    
			BorderNeighborhood child = BorderNeighborhood{childID, childID, neighboringFacesChild, childBorder};

			// get the common border
			ParentChildBorder borderRegions = detectCommonBorder(pMesh, &parent, &child);

			// remove the head border from the Mesh
			T3DMesh<float>::Submesh *subParent = pMesh->getSubmesh(parentID);
			T3DMesh<float>::Submesh *subChild = pMesh->getSubmesh(childID); 
			std::vector<T3DMesh<float>::Face> newFacesParent;
			std::vector<T3DMesh<float>::Face> newFacesChild;

			// push original faces and the torso faces to the newFacesParent
			for(int i = 0; i < subParent->Faces.size(); i++){
				newFacesParent.push_back(subParent->Faces[i]);
			}
			for(int i = 0; i < borderRegions.childBorder.size(); i++){
				T3DMesh<float>::Face f = subChild->Faces[borderRegions.childBorder.at(i)];
				newFacesParent.push_back(f);
			}
			subParent->Faces = newFacesParent;

			// push everything except the border to the newFacesChild
			for(int i = 0; i < subChild->Faces.size(); i++){
				bool inVec = std::find(borderRegions.childBorder.begin(), borderRegions.childBorder.end(), i) != borderRegions.childBorder.end();
				if(!inVec){
					newFacesChild.push_back(subChild->Faces[i]); 
				}
			}
			subChild->Faces = newFacesChild;
		}

		void getNeigborhoodRegionMesh(T3DMesh<float> *pMesh, PNGImport *textur){
			if(pMesh->submeshCount() != m_allRegionTypes.size()) throw CForgeExcept("Region Mesh has not the same number of regions as the hard coded ones");
			if(pMesh == nullptr) throw CForgeExcept("Mesh is nullptr");
			if(textur == nullptr || !textur->isPictureLoaded()) throw CForgeExcept("Texture is nullptr or not existing"); 

			// we want to get the neighborhood of the regions
			// i.e. which region is next to which one

			std::vector<BorderNeighborhood> borderNeighborhood;
			for(int i = 0; i < pMesh->submeshCount(); i++){
				std::vector<std::vector<int32_t>> neighboringFaces; 
				std::vector<int32_t> border =  MeshConnection::getEdgesSubmesh(pMesh, i, &neighboringFaces); 
				borderNeighborhood.push_back(BorderNeighborhood{i, m_allRegionTypes[i], neighboringFaces, border});
			}

			// compare every border triangle to the others -> which borders which?
			ParentChildBorder borderRegions; 
			for(int region = 0; region < m_Neighborship.size(); region++){
				std::vector<BoneGroups> neighbor = m_Neighborship.at(region);

				for(int border = 0; border < neighbor.size(); border++){
					// for every border...
					BorderNeighborhood *parent = &borderNeighborhood.at(region);
					BorderNeighborhood *child = &borderNeighborhood.at(neighbor.at(border));
					
					// // get the common border
					// borderRegions = detectCommonBorder(pMesh, parent, child);
					// // get the color of the borders
					// double colorDifference = 0; 
					
					// // colorDifferenceChildParent(pMesh, textur, parent, child, &borderRegions);

					// if(colorDifference < m_thresholdDeltaColor){
					// 	// add to the parent and do it again
					// 	// remove from child 
					// 	addAndRemoveFacesMesh(pMesh, parent, child, &borderRegions);
					// }

					
					int counterFloatFill = 0; 
					double colorDifference = 0;
					double oldColorDifference = 0; 
					do { //.do it once first and then check -> do while...
						// get the common border
						borderRegions = detectCommonBorder(pMesh, parent, child);
						if(borderRegions.childBorder.size() == 0 || borderRegions.parentBorder.size() == 0){
							std::cout << "No Common Border" << std::endl;
							break; 
						} 

						colorDifference = colorDifferenceChildParent(pMesh, textur, parent, child, &borderRegions);
						std::cout << "Child id: " << child->id << " Parent id: " << parent->id << "| Color Difference: " << colorDifference << std::endl;

						// add to the parent and do it again
						// remove from child 
						addAndRemoveFacesMesh(pMesh, parent, child, &borderRegions);

						
						if(m_thresholdDeltaColor * 1.5 < oldColorDifference + colorDifference){
							std::cout << "Color Difference is too high" << std::endl;
							break;
						}  
						oldColorDifference = colorDifference;

						// we know that the child's border is the new border of the parent - do not need to search
						// for(auto &element : borderRegions.childBorder){
						// 	parent->border.push_back(element);
						// }
						// but need to search for the child (or get it via the neighbroing faces)
						
						std::vector<int32_t> borderParent =  MeshConnection::getEdgesSubmesh(pMesh, parent->id);
						std::vector<int32_t> borderChild =  MeshConnection::getEdgesSubmesh(pMesh, child->id);
						parent->border = borderParent;  
						child->border = borderChild;
						// I need to do it at the regions not the indicies 
						// Ich mein, hier nehme ich einfach region und border, brauche aber an den stellen neighborhood(region) und neighborhood(border)
						// d.h. ich benutzte die völlig falschen submeshes! - deshalb funktioniert das nur einmal!

						// for(int i = 0; i < pMesh->submeshCount(); i++){
						// 	std::vector<std::vector<int32_t>> neighboringFaces; 
						// 	std::vector<int32_t> border =  MeshConnection::getEdgesSubmesh(pMesh, i, &neighboringFaces); 
						// 	borderNeighborhood.push_back(BorderNeighborhood{i, m_allRegionTypes[i], neighboringFaces, border});
						// }		
					} 
					while (colorDifference < m_thresholdDeltaColor || counterFloatFill < m_maxFloatFillItteration);
					

				}//border
			}//region  
		}
		
		void addAndRemoveFacesMesh(T3DMesh<float> *pMesh, BorderNeighborhood *parent, BorderNeighborhood *child, ParentChildBorder *borderRegions){
			// we want to add the faces from the child to the parent and remove them from the child
			// we also want to update the border of the parent and child

			T3DMesh<float>::Submesh *subParent = pMesh->getSubmesh(parent->id);
			T3DMesh<float>::Submesh *subChild = pMesh->getSubmesh(child->id);
			std::vector<T3DMesh<float>::Face> newFacesParent;
			std::vector<T3DMesh<float>::Face> newFacesChild;
			

			// push original faces and the torso faces to the newFacesParent
			for(int i = 0; i < subParent->Faces.size(); i++){
				newFacesParent.push_back(subParent->Faces[i]);
			}
			for(int i = 0; i < borderRegions->childBorder.size(); i++){
				T3DMesh<float>::Face f = subChild->Faces[borderRegions->childBorder.at(i)];
				newFacesParent.push_back(f);
			}
			subParent->Faces = newFacesParent;

			// push everything except the border to the newFacesChild
			for(int i = 0; i < subChild->Faces.size(); i++){
				bool inVec = std::find(borderRegions->childBorder.begin(), borderRegions->childBorder.end(), i) != borderRegions->childBorder.end();
				if(!inVec){
					newFacesChild.push_back(subChild->Faces[i]); 
				}
			}
			subChild->Faces = newFacesChild;
		}

		ParentChildBorder detectCommonBorder(T3DMesh<float> *pMesh, BorderNeighborhood *parent, BorderNeighborhood *child){
			// we want to detect the border between two regions (parent and child)
			
			ParentChildBorder Rval; 
			T3DMesh<float>::Submesh *parentSub = pMesh->getSubmesh(parent->id);
			T3DMesh<float>::Submesh *childSub  = pMesh->getSubmesh(child->id);
			
			// here we save the future border - so we can compare the color
			std::vector<int32_t> parentBorder; std::vector<int32_t> childBorder;

			// get the actual triangles which border each other
			// need to search through the faces of the submeshes - but we have the border
			for(int i = 0; i < parent->border.size(); i++){
				// this face we want to compare with the child
				T3DMesh<float>::Face fParent = parentSub->Faces[parent->border.at(i)];

				for(int j = 0; j < child->border.size(); j++){
					T3DMesh<float>::Face fChild = childSub->Faces[child->border.at(j)];

					// compare the faces
					if(MeshConnection::shareEdge(fParent, fChild)){
						parentBorder.push_back(parent->border.at(i)); 
						childBorder.push_back(child->border.at(j)); 
					}

				}//child
			}//parent

			Rval.childBorder = childBorder;
			Rval.parentBorder = parentBorder;
			return Rval;

		}//detectCommonBorder

		double colorDifferenceChildParent(T3DMesh<float> *pMesh, PNGImport *textur, BorderNeighborhood *parent, BorderNeighborhood *child, ParentChildBorder *borderRegions){
			double Rval; 
			// first get the mean color of the regions
			Eigen::Vector4d parentCol = textur->getMeanColor(pMesh, parent->id, &borderRegions->parentBorder, &m_SMPLXMesh, &m_correspondenciesNotInverted);
			Eigen::Vector4d childCol = textur->getMeanColor(pMesh, child->id, &borderRegions->childBorder, &m_SMPLXMesh, &m_correspondenciesNotInverted);
		
			// convert it to lab space and then compare the two colors
			Eigen::Vector3f parentCol3 = Eigen::Vector3f(parentCol.x(), parentCol.y(), parentCol.z());
			Eigen::Vector3f childCol3 = Eigen::Vector3f(childCol.x(), childCol.y(), childCol.z());
			Rval = ColorSpace::differenceRGBValues(parentCol3, childCol3); 
			return Rval; 
		}

		T3DMesh<float> getRegionalModel(const std::string fileName, CForge::T3DMesh<float> *mesh){
            std::vector<int32_t> correspondingGroup = maxSkinningInfluence(fileName); 
			std::vector<BoneGroups> boneRegion = getRegionOfGroup(correspondingGroup);
			return constructRegionalModel(mesh, boneRegion);
		}

		std::vector<std::vector<int>> countDoubleVerticesKDTree(T3DMesh<float>* pProxy, T3DMesh<float>* pOrigin, std::vector<int> *notInverted = nullptr){
			// this is the same as countDoubleVertices but it is much faster due to the use of a kd-tree
			// it achieves the same result as the other function

			std::vector<Eigen::Vector3f> A, B;  
			std::vector<PointLenght> closestPoints;
			std::vector<std::vector<int>> Rval; 

			for(int i = 0; i < pOrigin->vertexCount(); i++){
				A.push_back(pOrigin->vertex(i)); 
			}
			for(int i = 0; i < pProxy->vertexCount(); i++){
				B.push_back(pProxy->vertex(i)); 
			}
			
			ICP::findClosestPointsKDTree(&A, &B, closestPoints);

			for(int i = 0; i < pProxy->vertexCount(); i++){
				std::vector<int> p = {-1}; 
				Rval.push_back(p); 
			}

			// now invert closestPoints
			for(int i = 0; i < closestPoints.size(); i++){
				int t = closestPoints[i].target; 
				int s = closestPoints[i].source; 
				Rval.at(t).push_back(s); 

				// for not inverted
				notInverted->push_back(closestPoints[i].target);
			}

			for(int i = 0; i < Rval.size(); i++){
				Rval.at(i).erase(Rval.at(i).begin()); // get rid of -1
			}

			// for(int i = 0; i < Rval.size(); i++){
			// 	printf("%d: ", i);
			// 	for(int j = 0; j < Rval.at(i).size(); j++){
			// 		printf("%d ", Rval.at(i).at(j)); 
			// 	}
			// 	printf("\n"); 
			// }

			return Rval; 
		}

        std::vector<int32_t> maxSkinningInfluence(const std::string &fileName){
            std::vector<std::vector<int32_t>> Influences; 
            std::vector<std::vector<float>> Weights;
            BuildBones::readJsonFileExtractSkinningWeights(fileName, Influences, Weights); 

            // get the "bone" / influence a vertex belongs the most to
            // i.e. somewhere most weight -> find it 
            // problem: here do not have all, can not just itterate over all

            std::vector<int32_t> Result; 
            
            for(int i = 0; i < Influences[0].size(); i++){
                float maxVal = -1; 
                int32_t boneVal = -1; 
                for(int bone = 0; bone < Influences.size(); bone++){
                    if(Weights[bone][i] > maxVal){
                        boneVal = bone; 
                        maxVal = Weights[bone][i]; 

						// if weight > 0.5 then there can't be any larger one (weight in [0,1])
						if (maxVal > 0.5f) break; 
                    }
                }
				// printf("%d\n", boneVal); 
                Result.push_back(boneVal); 
            }

            return Result; 
        }

		std::vector<BoneGroups> getRegionOfGroup(std::vector<int32_t> correspondingGroup){
			std::vector<BoneGroups> result; 
			for(int i = 0; i < correspondingGroup.size(); i++){
				int32_t group = correspondingGroup[i]; 
				BoneGroups region = m_SMPLXGroup[group];
				result.push_back(region); 
			}
			return result; 
		}

		T3DMesh<float> constructRegionalModel(CForge::T3DMesh<float> *mesh, std::vector<BoneGroups> vertexRegions){
			// the mesh we receive is a proxy and we want to construct a totally new one
			// corresponding Group is the vertex belonging, from this we can construct a face belonging
			
			// 1. construct model - later this will be returned and initialized
			// in this model we just want to display the colors of the different regions
			T3DMesh<float> regionalModel;// = new T3DMesh<float>(); 
			regionalModel.init(); 

			// 1.1. get all the vertices for the new mesh
			std::vector<Eigen::Vector3f> vertices; 
			for(int i = 0; i < mesh->vertexCount(); i++){
				Eigen::Vector3f v = mesh->vertex(i); 
				vertices.push_back(v); 
			}
			regionalModel.vertices(&vertices); 

			// 2. we need to find which face belongs to which group (we only know it for the vertices)
			// so we get the submesh; then we get the faces and put them into on of the groups
			std::vector<std::vector<T3DMesh<float>::Face>> allRegions; 
			for(int i = 0; i < m_allRegionTypes.size(); i++){
				std::vector<T3DMesh<float>::Face> region; 
				allRegions.push_back(region);
			}

			// 2.1 split the faces off in different groups according to their kind
			std::vector<BoneGroups> faceGroupBelonging;
			for(int i = 0; i < mesh->submeshCount(); i++){
				T3DMesh<float>::Submesh *sub = mesh->getSubmesh(i);

				for(int fCount = 0; fCount < sub->Faces.size(); fCount++){
					// get the type/region of the face
					BoneGroups face = faceAccordingGroup(sub, vertexRegions, fCount);
					faceGroupBelonging.push_back(face); 

					// push it to the right region
					auto it = find(m_allRegionTypes.begin(), m_allRegionTypes.end(), face);
					int regionIndex; 
					if(it != m_allRegionTypes.end()) regionIndex = it - m_allRegionTypes.begin(); 
					else throw CForgeExcept("Material: is not inside the vector!"); 

					T3DMesh<float>::Face resultingFace =  sub->Faces[fCount];
					allRegions.at(regionIndex).push_back(resultingFace); 
				}
			}


			// 3. we create a material for the group
			// the material must have the same number as the submesh
			for(int i = 0; i < m_allRegionTypes.size(); i++){
				T3DMesh<float>::Material mat; 
				mat.init(); 
				mat.ID = i;
				mat.Color = m_ColorGroups[i]; 

				regionalModel.addMaterial(&mat, true); 

				// 4. we create a submesh
				T3DMesh<float>::Submesh sub; 
				sub.Material = mat.ID; 

				// 4.1 add the according faces to the submesh
				sub.Faces = allRegions.at(i); 

				// 5. add the submeshes to the T3DMesh
				regionalModel.addSubmesh(&sub, true); 
			}   

			return regionalModel; 
		}

		BoneGroups faceAccordingGroup(T3DMesh<float>::Submesh *sub, std::vector<BoneGroups> correspondingGroup, int count){
			T3DMesh<float>::Face f = sub->Faces[count];
			int v1 = f.Vertices[0], v2 = f.Vertices[1], v3 = f.Vertices[2]; 
			
			BoneGroups rval; 
			if(v1 == v2 && v2 == v3){
				// all are the same, trivial
				rval = correspondingGroup[v1]; 
			}
			else if(v1 == v2 || v1 == v3 || v2 == v3){
				// two are the same return the highest one
				if(v1 == v2) rval = correspondingGroup[v1]; 
				else if (v1 == v3) rval = correspondingGroup[v1]; 
				else rval = correspondingGroup[v2]; 				
			}
			else{
				// maybe something better? -> examin neighborhood
				// which vertex has the most neighbors with the same class?
				rval=  correspondingGroup[v2];  
			}
			return rval; 
		}

	protected:
		// Scene Graph
		SGNTransformation m_RootSGN;

		StaticActor m_Reconstruction; 
        SGNGeometry m_ReconstructionSGN;
        SGNTransformation m_ReconstructionTransformSGN;

		StaticActor m_RegionActor; 
        SGNGeometry m_RegionActorSGN;
        SGNTransformation m_RegionActorTransformSGN;

		SkyboxActor m_Skybox;
		vector<string> m_ClearSky;

		SceneGraph m_SkyboxSG;
		SGNTransformation m_SkyboxTransSGN;
		SGNGeometry m_SkyboxGeomSGN;
		
        T3DMesh<float> m_SMPLXMesh;
		T3DMesh<float> m_ProxyMesh; 
		T3DMesh<float> m_RegionMesh;
		T3DMesh<float>* m_RegionMeshPtr;

		std::vector<std::vector<int32_t>> m_correspondencies;
		std::vector<int32_t> m_correspondenciesNotInverted;

        Eigen::Vector3f m_translationReconstruction = Eigen::Vector3f(0.0f, 0.0f, 0.0f); 

		std::array<BoneGroups, 6> m_allRegionTypes = {HEAD, TORSO, FOOT, LEG, ARM, HAND};
		std::array<Eigen::Vector4f, 6> m_ColorGroups = {Vector4f(1.0f, 0.0f, 0.0f, 1.0f), 
														Vector4f(0.0f, 1.0f, 0.0f, 1.0f),
														Vector4f(0.0f, 0.0f, 1.0f, 1.0f),
														Vector4f(1.0f, 1.0f, 0.0f, 1.0f),
														Vector4f(1.0f, 0.0f, 1.0f, 1.0f),
														Vector4f(0.0f, 1.0f, 1.0f, 1.0f)};
		
		std::array<BoneGroups, 55> m_SMPLXGroup =  
									{TORSO, LEG, LEG, TORSO, LEG, 
									LEG, TORSO, FOOT, FOOT, TORSO, 
									FOOT, FOOT, HEAD, TORSO, TORSO, 
									HEAD, ARM, ARM, ARM, ARM, 
									HAND, HAND, HEAD, HEAD, HEAD, 
									HAND, HAND, HAND, HAND, HAND, 
									HAND, HAND, HAND, HAND, HAND, 
									HAND, HAND, HAND, HAND, HAND, 
									HAND, HAND, HAND, HAND, HAND,
									HAND, HAND, HAND, HAND, HAND, 
									HAND, HAND, HAND, HAND, HAND}; 
		
		// hard coded neighborship 
		std::vector<std::vector<BoneGroups>> m_Neighborship = {
			{TORSO},
			{HEAD, ARM, LEG},
			{LEG},
			{FOOT},
			{HAND},
			{ARM}
		}; 

		// during floatfill we do not want to do things two times (i.e. HEAD -> TORSO and TORSO -> HEAD)
		// so we have a one way neighborship
		std::vector<std::vector<BoneGroups>> m_NeighborshipOneWay = {
			{}, // HEAD
			{HEAD, ARM, LEG}, // TORSO
			{LEG}, // FOOT
			{}, // LEG
			{}, // ARM
			{ARM}, // HAND
		};

		// max color difference in lab space
		double m_thresholdDeltaColor = 3;
		int m_maxFloatFillItteration = 10;
		PNGImport m_textur; 

	}; //ExampleBAMaterials

}//name space
