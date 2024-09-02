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
#include <Prototypes/SMPLTexturing/SubmeshFabricHelper.h>

#include "python3.10/Python.h"

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
			
            m_ReconstructionTransformSGN.translation(Vector3f(-0.7f, 0.55f, 0));
			m_ReconstructionTransformSGN.rotation(Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), Vector3f::UnitY())));
            m_ReconstructionSGN.init(&m_ReconstructionTransformSGN, &m_Reconstruction);

			const std::string fileName = "MyAssets/result.json"; 
			T3DMesh<float> regionMesh = getRegionalModel(fileName, &m_ProxyMesh);
			
			// copying the region mesh to m_RegionMesh
			std::vector<Vector3f> vertices;
			std::vector<T3DMesh<float>::Face> faces;
			for(int i = 0; i < regionMesh.vertexCount(); i++){
				vertices.push_back(regionMesh.vertex(i));
			}
			m_RegionMesh.vertices(&vertices);
			for(int i = 0; i < regionMesh.submeshCount(); i++){
				T3DMesh<float>::Submesh *sub = regionMesh.getSubmesh(i); 
				m_RegionMesh.addSubmesh(sub, true);	

				std::vector<T3DMesh<float>::Face> faces;
				faces = sub->Faces;
				m_allFaces.push_back(faces);
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
			m_RegionActorTransformSGN.translation(Vector3f(0.3f, 0.55f, 0.0f));

			for(int i = 0; i < m_RegionMesh.submeshCount(); i++){
				T3DMesh<float>::Submesh *sub = m_RegionMesh.getSubmesh(i);
				m_areaSubmeshes[i] = MeshConnection::getAreaSubmesh(&m_RegionMesh, i);
			}

			// create help text
			LineOfText* pKeybindings = new LineOfText();
			pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Movement: (Shift) + W,A,S,D  | Rotation: LMB/RMB + Mouse | F1: Toggle help text | 1: GrowRegion | 2: FloatFill | 3: Reset | 5: Split&Export");
			m_HelpTexts.push_back(pKeybindings);
			m_DrawHelpTexts = true;
			m_ScreenshotExtension = "jpg";

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

			// needed later for floatfill
			m_textur.readPNGFile("MyAssets/Reko_Timo/Timon_A_20k.png"); 
			m_correspondencies = countDoubleVerticesKDTree(&m_ProxyMesh, &m_SMPLXMesh, &m_correspondenciesNotInverted);

			// removeIsolatedVertices(&m_RegionMesh, BoneGroups::TORSO);

			// std::string ErrorMsg;
			// if (0 != CForgeUtility::checkGLError(&ErrorMsg)) {
			// 	SLogger::log("OpenGL Error" + ErrorMsg, "PrimitiveFactoryTestScene", SLogger::LOGTYPE_ERROR);
			// } 

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
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_3)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_3, Keyboard::KEY_RELEASED); 
				for(int i = 0; i < m_RegionMesh.submeshCount(); i++){
					T3DMesh<float>::Submesh *sub = m_RegionMesh.getSubmesh(i);
					sub->Faces = m_allFaces.at(i);
				}
				m_RegionActor.init(&m_RegionMesh);	
			}
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_4)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_4, Keyboard::KEY_RELEASED); 
				// take screenshot
				m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
				std::string ScreenshotURI = "Screenshots/ScreenshotforAI_" + std::to_string(m_ScreenshotCount++) + "." + m_ScreenshotExtension;
				takeScreenshot(ScreenshotURI);
				runPythonScript("/home/niclas/dev/BA/TextileNet/src/get_material_of_screenshot_cpp.py"); 
				
			}
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_5)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_5, Keyboard::KEY_RELEASED); 
				
				// like this I can display it - problem: need to do a deep copy for SMPLXMesh to display it (faces are not copied)
				// you can not delete submeshes, so "updating it" is not possible - or you need to adjust the T3DMesh class

				// TODO: set a check if no screenshots are taken yet, then do it now and split the mesh then

				if(!m_meshIsSplit){
					m_takePictures = true;
					m_splitMesh = true; 
				}
				else{
					m_meshIsSplit = true; 
				}

			}
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_6)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_6, Keyboard::KEY_RELEASED); 
				if(!m_meshIsSplit) m_takePictures = true;
			}
			if(m_takePictures){
				// 1 move camera, 2 wait, 3 take picture, set to 1 unitl all pictures are taken
				if(m_indexPicture == m_trianglesToPictures.size()){
					m_takePictures = false; 
					m_indexPicture = 0; 

					// all pictures are taken - run the python script
					runPythonScript("/home/niclas/dev/BA/TextileNet/src/analyse_images_in_folder.py"); 
					m_FabricHelpers = SubmeshFabricHelper::readFabricHelper("results_folder.json"); 
					updateFabricHelper(); 
				}

				switch (m_indexPictureLastFrame)
				{
				case 0: {
					/* set camera*/
					m_indexPictureLastFrame++;
					int index = m_trianglesToPictures.at(m_indexPicture); 
					takePictuesOfTrianglregion(&m_ReconstructionTransformSGN, &m_SMPLXMesh, 0, index, false);
					m_picturesAreTaken = true; 
					break;
				}
				case 1: {
					/* wait one frame */
					m_indexPictureLastFrame++;
					break;
				}
				case 2: {
					m_indexPicture++; 
					m_indexPictureLastFrame = 0;
					m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
					std::string ScreenshotURI = "Screenshots/ScreenshotforAI_" + std::to_string(m_ScreenshotCount++) + "." + m_ScreenshotExtension;
					takeScreenshot(ScreenshotURI);
					
					break;
				}
				default:
					break;
				}
			}
			if(m_splitMesh && !m_takePictures){
				m_splitMesh = false;

				T3DMesh<float> mesh = MeshConnection::splitMeshFromProxy(&m_SMPLXMesh, &m_RegionMesh, m_correspondencies);
				MeshConnection::copyMesh(&mesh, &m_SMPLXMesh);
				m_SMPLXMesh.computePerFaceNormals();
				m_SMPLXMesh.computePerVertexNormals();
				m_Reconstruction.init(&m_SMPLXMesh); 

				objImportExport::exportSubmeshesAsObjFiles(std::vector<std::string> {}, &m_SMPLXMesh); 
				std::cout<<"Exported SMPLX Submeshes" << std::endl; 
			}

		}//mainLoop{}


		void runPythonScript(const char* filename){
			// https://stackoverflow.com/questions/16962430/calling-python-script-from-c-and-using-its-output
			// https://github.com/hackable-devices/polluxnzcity/blob/master/PolluxGateway/src/pollux/pollux_extension.C
			// remove from the filename the last part
			std::string filePath(filename);
			std::string directoryPath = filePath.substr(0, filePath.find_last_of('/'));
			directoryPath = directoryPath.substr(0, directoryPath.find_last_of('/'));
			setenv("PYTHONPATH", directoryPath.c_str(), 1);
			
			Py_Initialize(); 
			FILE* file = fopen(filename, "r");
			 if (file) {
        		// Run the Python script
        		// PyRun_SimpleFile(file, filename);
				PyRun_AnyFile(file, filename); 
        		// Close the file
        		fclose(file);
				std::cout << "" << std::endl; 
    		} else {
        		std::cerr << "Could not open file: " << filename << std::endl;
    		}
			Py_Finalize();
		}

		void floatfill(T3DMesh<float> *pMesh, PNGImport *textur){
			if(pMesh->submeshCount() != m_allRegionTypes.size()) throw CForgeExcept("Region Mesh has not the same number of regions as the hard coded ones");
			if(pMesh == nullptr) throw CForgeExcept("Mesh is nullptr");
			if(textur == nullptr || !textur->isPictureLoaded()) throw CForgeExcept("Texture is nullptr or not existing");
			
			std::vector<T3DMesh<float>::Face> isolatedFaces = MeshConnection::getIsolatedFacesWholeMesh(pMesh);

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

					// get the threshold for the individual regions - if not set, then use the global one
					// this is to make the regions more individual/grow better
					double thresholdIndividualDelta = m_thresholdIndividual.at(parentGroup).at(childID);
					if(thresholdIndividualDelta <= 0.0){
						thresholdIndividualDelta = m_thresholdDeltaColor; 
					}

					do {
						// 2.1 get the border of the parent and the child
						parentBorder = MeshConnection::getEdgesSubmesh(pMesh, parentGroup, &neighboringFacesParent);
						childBorder = MeshConnection::getEdgesSubmesh(pMesh, childGroup, &neighboringFacesChild);
						parent = BorderNeighborhood{parentGroup, parentGroup, neighboringFacesParent, parentBorder};
						child = BorderNeighborhood{childGroup, childGroup, neighboringFacesChild, childBorder};
						borderRegions = detectCommonBorder(pMesh, &parent, &child);

						// 2.2 what to do when border is empty? -> just skip the region
						if(borderRegions.childBorder.empty() || borderRegions.parentBorder.empty()){
							std::cout << "Border is empty" << std::endl;
							break;
						}

						// 3. get the color of the borders
						oldColorDifference = colorDifference;
						colorDifference = colorDifferenceChildParent(pMesh, textur, &parent, &child, &borderRegions);

						// 4. if color under threshold -> add to parent and remove from child
						if(colorDifference < thresholdIndividualDelta){ // && counter < m_maxFloatFillItteration){
							addAndRemoveFacesMesh(pMesh, &parent, &child, &borderRegions);
							std::cout << "Parent: " << parentID << "| Child: " << childID << " | Color Difference: " << colorDifference << std::endl;
							counter++;

							// std::vector<int32_t> isolatedFaces = MeshConnection::getIsolatedFacesSubmesh(pMesh, childGroup);
							// std::cout << "Isolated Faces: " << isolatedFaces.size() << std::endl;

							// if(counter == m_maxFloatFillItteration) removeIsolatedFacesNeighbor(pMesh, childGroup, parentGroup);
						}
						else{
							std::cout << "Color Difference too high: " << colorDifference << std::endl;
							counter = 0; 

							// serach for isolated faces and add them to the according region
							// removeIsolatedFacesNeighbor(pMesh, childGroup, parentGroup);
							mergeIsolatedFacesNeighbor(pMesh, isolatedFaces, childGroup, parentGroup); 

							// there can still be isolated Faces, which came through the creation of the 
							// regionmesh -> here we want to remove them
							// removeIsolatedFacesNeighbor(pMesh, childGroup, parentGroup);
							// the problem is, that there are no isolated faces, it is easier to just merge them if the area is very small
						
							mergeIfSubmeshSmall(pMesh, childGroup, parentGroup);
						}

					// 5. rinse and repeat // && counter < m_maxFloatFillItteration);
					} while (colorDifference < thresholdIndividualDelta); 
					m_NeighborshipOneWay = m_NeighborshipOneWayNew;
					
				}// childID
			}// parentID

			// add submesh to FabricHelper
			if(m_FabricHelpers.size() == m_RegionMesh.submeshCount()){
				for(int i = 0; i < m_FabricHelpers.size(); i++){
					m_FabricHelpers[i].submesh = m_RegionMesh.getSubmesh(i);

					BoneGroups current = m_allRegionTypes.at(i);
					if(current == BoneGroups::HEAD || current == BoneGroups::HAND){
						m_FabricHelpers[i].isSkin = true; 
					}
				}
			}
			

			// update the fabrichelper
			// if the submesh is empty, then remove it
			// for (int i = m_FabricHelpers.size() - 1; i >= 0; --i) {
    		// 	if (m_FabricHelpers[i].submesh->Faces.empty()) {
        	// 		m_FabricHelpers.erase(m_FabricHelpers.begin() + i);
    		// 	}
			// }
			m_FabricHelpers.erase(
    			std::remove_if(m_FabricHelpers.begin(), m_FabricHelpers.end(),
                   [](const auto& helper) { return helper.submesh->Faces.empty(); }),
    		m_FabricHelpers.end());
		}

		void mergeIfSubmeshSmall(T3DMesh<float> *pMesh, int subIndexChild, int subIndexParent){
			float areaChildNew = MeshConnection::getAreaSubmesh(pMesh, subIndexChild);
			float areaParentNew = MeshConnection::getAreaSubmesh(pMesh, subIndexParent);

			float areaChildOld = m_areaSubmeshes[subIndexChild]; 
			float areaParentOld = m_areaSubmeshes[subIndexParent];

			// if the area ratio before and after is smaller then a preset ratio, then we merge the regions
			// the ratio of the child should get smaller and the ratio of the parent should get bigger
			float ratioChild = areaChildNew / areaChildOld;
			float ratioParent = areaParentNew / areaParentOld; 

			if(ratioChild < m_areaDifferenceRatio){
				// merge the whole submesh 
				T3DMesh<float>::Submesh *subChild = pMesh->getSubmesh(subIndexChild);
				T3DMesh<float>::Submesh *subParent = pMesh->getSubmesh(subIndexParent);

				// add the faces of the child to the parent
				for(int i = 0; i < subChild->Faces.size(); i++){
					subParent->Faces.push_back(subChild->Faces[i]);
				}
				// remove the faces of the child
				subChild->Faces.clear();

				// and set the neighborhood differently
				// the children of the child should be the children of the parent
				m_NeighborshipOneWayNew[subIndexParent].insert(m_NeighborshipOneWay[subIndexParent].end(), m_NeighborshipOneWay[subIndexChild].begin(), m_NeighborshipOneWay[subIndexChild].end());
				m_NeighborshipOneWayNew[subIndexParent].erase(std::remove(m_NeighborshipOneWayNew[subIndexParent].begin(), m_NeighborshipOneWayNew[subIndexParent].end(), subIndexChild), m_NeighborshipOneWayNew[subIndexParent].end());
				m_NeighborshipOneWayNew[subIndexChild].clear(); 
			}
		}

		void mergeIsolatedFacesNeighbor(T3DMesh<float> *pMesh, std::vector<T3DMesh<float>::Face> isolatedFacesOrg, int subIndexChild, int subIndexParent){
			// goal: if the isolated faces are new, we want to merge them (because they came through the process of floatfill)
			// for the others we need to decide

			// isolatedFacesOrg -> original isolated faces, we can compare the isolated faces now and beforehand to mere them
			if(pMesh == nullptr) throw CForgeExcept("Mesh is nullptr");
			if(subIndexChild < 0 || subIndexChild >= pMesh->submeshCount()) throw CForgeExcept("Submesh index Child is out of bounds");
			if(subIndexParent < 0 || subIndexParent >= pMesh->submeshCount()) throw CForgeExcept("Submesh index Parent is out of bounds");

			T3DMesh<float>::Submesh *subChild = pMesh->getSubmesh(subIndexChild);
			T3DMesh<float>::Submesh *subParent = pMesh->getSubmesh(subIndexParent); 

			// get the isolated faces of the child
			std::vector<int32_t> isolatedFacesIndexNow = MeshConnection::getIsolatedFacesSubmesh(pMesh, subIndexChild);
			std::vector<T3DMesh<float>::Face> isolatedFacesNow;
			for(int i = 0; i < isolatedFacesIndexNow.size(); i++){
				isolatedFacesNow.push_back(subChild->Faces[isolatedFacesIndexNow.at(i)]);
			}

			if(isolatedFacesNow.empty()) return; // no isolated faces -> nothing to do

			// wenn ich es nicht finde, hier ist gerade, dass ich es finde -> einfach mit find (wenn das geht...)
			// wenn nicht, find, dann einfach hinzufügen
			for(int i = 0; i < isolatedFacesNow.size(); i++){
				T3DMesh<float>::Face fNow = isolatedFacesNow.at(i);

				// find the face in the original isolated faces
				int pos = -1;
				for(int j = 0; j < isolatedFacesOrg.size(); j++){
					if(MeshConnection::areSameFace(fNow, isolatedFacesOrg.at(j))){
						pos = j;
						break;
					}
				}
				
				// did not find it -> add it to the parent
				if(pos == -1){
					// add the face to the parent
					subParent->Faces.push_back(fNow);
					// remove the face from the child
					T3DMesh<float>::Face f; f.Vertices[0] = -1; f.Vertices[1] = -1; f.Vertices[2] = -1;
					int indexFace = MeshConnection::getFaceIndex(pMesh, subIndexChild, fNow);

					if(indexFace != -1) subChild->Faces.at(indexFace) = f;
					else throw CForgeExcept("Face not found in the submesh");
				}
			}
			
			// "remove" the old faces - i.e. set new faces
			std::vector<T3DMesh<float>::Face> newFacesChild;
			for(int i = 0; i < subChild->Faces.size(); i++){
				T3DMesh<float>::Face f = subChild->Faces[i];
				if(f.Vertices[0] != -1 && f.Vertices[1] != -1 && f.Vertices[2] != -1){
					newFacesChild.push_back(f);
				}
			}
			subChild->Faces = newFacesChild;
		}

		void removeIsolatedFacesNeighbor(T3DMesh<float> *pMesh, int subIndexChild, int subIndexParent){
			if(pMesh == nullptr) throw CForgeExcept("Mesh is nullptr");
			if(subIndexChild < 0 || subIndexChild >= pMesh->submeshCount()) throw CForgeExcept("Submesh index Child is out of bounds");
			if(subIndexParent < 0 || subIndexParent >= pMesh->submeshCount()) throw CForgeExcept("Submesh index Parent is out of bounds");

			T3DMesh<float>::Submesh *subChild = pMesh->getSubmesh(subIndexChild);
			T3DMesh<float>::Submesh *subParent = pMesh->getSubmesh(subIndexParent);

			// get the isolated faces of the childmesh
			std::vector<int32_t> isolatedFacesIndex = MeshConnection::getIsolatedFacesSubmesh(pMesh, subIndexChild);
			

			if(isolatedFacesIndex.empty()) return; // no isolated faces -> nothing to do

			// check if they are nearer to the parent, if so, add them to the parent
			// how do we check if they are closer to the parent? 
			// create a kd-tree of the vertices of the parent

			// get the vertices of the parent - later construct the vector<Eigen::Vector3f> from it
			// then the kd-tree
			std::unordered_set<int32_t> verticesParent;
			for(int i = 0; i < subParent->Faces.size(); i++){
				T3DMesh<float>::Face f = subParent->Faces[i];
				verticesParent.insert(f.Vertices[0]);
				verticesParent.insert(f.Vertices[1]);
				verticesParent.insert(f.Vertices[2]);
			}
			// construct the vector<Eigen::Vector3f> from the vertices	
			std::vector<Eigen::Vector3f> verticesParentVec;
			for(auto it = verticesParent.begin(); it != verticesParent.end(); it++){
				verticesParentVec.push_back(pMesh->vertex(*it));
			}

			// get the verticesChildVec - these are the Eigen::Vector3f of the isolated faces
			std::unordered_set<int32_t> verticesChild;
			for(int i = 0; i < isolatedFacesIndex.size(); i++){
				T3DMesh<float>::Face f = subChild->Faces[isolatedFacesIndex.at(i)];
				verticesChild.insert(f.Vertices[0]);
				verticesChild.insert(f.Vertices[1]);
				verticesChild.insert(f.Vertices[2]);
			}
			std::vector<Eigen::Vector3f> verticesChildVec;
			for(auto it = verticesChild.begin(); it != verticesChild.end(); it++){
				verticesChildVec.push_back(pMesh->vertex(*it));
			}

			// create the kd-tree
			std::vector<PointLenght> closestsPoints;
			ICP::findClosestPointsKDTree(&verticesChildVec, &verticesParentVec, closestsPoints);

			// now we have the closest points - we can check if the isolated faces are closer to the parent

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

			// later we need to assign fabrics - populate the fabric helpers
			m_FabricHelpers.clear(); 
			for(int i = 0; i < m_allRegionTypes.size(); i++){
				FabricHelper helper; 
				helper.id = i; 
				helper.submesh = regionalModel.getSubmesh(i);
				m_FabricHelpers.push_back(helper);
			}
			
			return regionalModel; 
		}

		// set to skin if the region was marked beforehand as skin
		void updateFabricHelper(){
			for(int i = 0; i < m_FabricHelpers.size(); i++){
				BoneGroups current = m_allRegionTypes.at(i);
				if(current == BoneGroups::HEAD || current == BoneGroups::HAND){
					m_FabricHelpers[i].isSkin = true; 
				}
				// get the submesh of the fabric helper
				m_FabricHelpers[i].submesh = m_RegionMesh.getSubmesh(i);
			}
			
			return; 
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

		void takePictuesOfTrianglregion(SGNTransformation *transform, T3DMesh<float> *pMesh, int submeshIndex, int faceIndex, bool takePictures = true){
			if(pMesh == nullptr || submeshIndex < 0 || submeshIndex > pMesh->submeshCount() ) throw CForgeExcept("Mesh is nullptr");
			T3DMesh<float>::Submesh *sub = pMesh->getSubmesh(submeshIndex);
			T3DMesh<float>::Face f = sub->Faces[faceIndex];
			Eigen::Vector3f v1 = pMesh->vertex(f.Vertices[0]);
			Eigen::Vector3f v2 = pMesh->vertex(f.Vertices[1]);
			Eigen::Vector3f v3 = pMesh->vertex(f.Vertices[2]);
			Eigen::Vector3f normal = ((v2 - v1).cross(v3 - v1)).normalized();

			// do it in worldspace not in local space
			Eigen::Vector3f position;
			Eigen::Quaternionf rotation;
			Eigen::Vector3f scale;

			// m_ReconstructionTransformSGN
			transform->buildTansformation(&position, &rotation, &scale);

			Eigen::Matrix4f worldScale = CForgeMath::scaleMatrix(scale); 
			Eigen::Matrix4f worldPos = CForgeMath::translationMatrix(position); 
			Eigen::Matrix4f worldRot = CForgeMath::rotationMatrix(rotation); 
			Eigen::Matrix4f worldTransform = worldPos * worldRot * worldScale;

			Eigen::Vector4f v1_homogeneous(v1(0), v1(1), v1(2), 1.0f);
			Eigen::Vector4f worldV1_homogeneous = worldTransform * v1_homogeneous;
			Eigen::Vector3f worldV1 = worldV1_homogeneous.head<3>();

			Eigen::Vector3f camPos = worldV1 + (-1 * normal) * 0.5f;
			m_Cam.lookAt(camPos, worldV1);

			// take the picture
			if(takePictures){
				m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
				std::string ScreenshotURI = "Screenshots/ScreenshotforAI_" + std::to_string(m_ScreenshotCount++) + "." + m_ScreenshotExtension;
				takeScreenshot(ScreenshotURI);
			}
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

		std::vector<std::vector<int32_t>> m_correspondencies;	
		std::vector<int32_t> m_correspondenciesNotInverted;

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
		
		std::array<BoneGroups, 6> m_allRegionTypes = {HEAD, TORSO, FOOT, LEG, ARM, HAND};
		// later for taking the pictures we need to know where to take the pictues
		// face: 471, foot: 18744
		std::vector<int32_t> m_trianglesToPictures = {3057, 5835, 14994, 8376, 3772, 17624};  
		std::array<float, 6> m_areaSubmeshes; 
		float m_areaDifferenceRatio = 0.1f; 

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
			{HEAD, ARM}, // TORSO
			{LEG}, // FOOT
			{TORSO}, // LEG
			{}, // ARM
			{ARM}, // HAND
		};

		std::vector<std::vector<BoneGroups>> m_NeighborshipOneWayNew = m_NeighborshipOneWay; 

		std::vector<std::vector<double>> m_thresholdIndividual = {
			{-1}, // HEAD
			{3.5, 8}, // TORSO
			{3}, // FOOT
			{7}, // LEG
			{-1}, // ARM
			{3.5}, // HAND
		}; 

		// for resetting the mesh
		std::vector<std::vector<T3DMesh<float>::Face>> m_allFaces; 

		// max color difference in lab space
		double m_thresholdDeltaColor = 5;
		int m_maxFloatFillItteration = 50;
		PNGImport m_textur; 

		std::vector<FabricHelper> m_FabricHelpers;
		int triangleCounter = 0; 

		bool m_takePictures = false, m_picturesAreTaken = false, m_meshIsSplit = false, m_splitMesh = false; 
		int m_indexPicture = 0; 
		int m_indexPictureLastFrame = 0; 
	}; //ExampleBAMaterials

}//name space
