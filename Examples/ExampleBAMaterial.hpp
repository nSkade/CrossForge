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

#include <iostream>
#include <fstream>

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
            m_Reconstruction.init(&m_ProxyMesh);
			m_ReconstructionTransformSGN.init(&m_RootSGN); // Vector3f(0.0f, 0.0f, 0.0f)
			
            m_ReconstructionTransformSGN.translation(Vector3f(0.0f, 1.25f, 0.0f));
			m_ReconstructionTransformSGN.rotation(Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), Vector3f::UnitY())));
            m_ReconstructionSGN.init(&m_ReconstructionTransformSGN, &m_Reconstruction);

			const std::string fileName = "MyAssets/result.json"; 
			T3DMesh<float> regionMesh = getRegionalModel(fileName, &m_ProxyMesh);
			
			regionMesh.computePerVertexNormals(); 
			regionMesh.computePerFaceNormals(); 

			m_RegionActor.init(&regionMesh); 
			m_RegionActorTransformSGN.init(&m_RootSGN); 
			m_RegionActorSGN.init(&m_RegionActorTransformSGN, &m_RegionActor); 

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

		}//initialize

		T3DMesh<float> getRegionalModel(const std::string fileName, CForge::T3DMesh<float> *mesh){
            std::vector<int32_t> correspondingGroup = maxSkinningInfluence(fileName); 
			std::vector<BoneGroups> boneRegion = getRegionOfGroup(correspondingGroup);
			return constructRegionalModel(mesh, boneRegion);
		}

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

        std::vector<std::vector<int>> countDoubleVertices(T3DMesh<float>* pOrigin, T3DMesh<float>* pProxy, float eps = 0.0001f){
			std::vector<std::vector<int>> correspondingIndex;
			for(int i = 0; i < pOrigin->vertexCount(); i++){
				Vector3f originVertex = pOrigin->vertex(i);
				std::vector<int> vertexList; 
				
				for(int j = 0; j < pProxy->vertexCount(); j++){
					Vector3f proxyVertex = pProxy->vertex(j);

					float dist = (proxyVertex - originVertex).norm();
					if(dist < eps){
						vertexList.push_back(j); 
					}
				}
				if(vertexList.empty()){
					std::cout << "At place " << i << " is empty" << std::endl; 
					vertexList.push_back(-1); 
				}
				correspondingIndex.push_back(vertexList); 
			}
			
			// check whether the corresponding list has as many entrys as the proxy geometry
			// int counter = 0; 
			// for (size_t i = 0; i < correspondingIndex.size(); i++){
			// 	counter += correspondingIndex[i].size();
			// } // -> should be the same 
			
			return correspondingIndex; 
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

					// TODO: HIER IRGENDWO LIEGT DER FEHLER
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
				rval=  correspondingGroup[v1];  
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
									FOOT, FOOT, FOOT, TORSO, TORSO, 
									FOOT, ARM, ARM, ARM, ARM, 
									HAND, HAND, FOOT, FOOT, FOOT, 
									HAND, HAND, HAND, HAND, HAND, 
									HAND, HAND, HAND, HAND, HAND, 
									HAND, HAND, HAND, HAND, HAND, 
									HAND, HAND, HAND, HAND, HAND,
									HAND, HAND, HAND, HAND, HAND, 
									HAND, HAND, HAND, HAND, HAND}; 
	
    };//ExampleScanScreenshot

}//name space
