/*****************************************************************************\
*                                                                           *
* File(s): ExampleICP.hpp                                        			*
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
#ifndef __CFORGE_EXAMPLEICP_HPP__
#define __CFORGE_EXAMPLECP_HPP__


#include "ExampleSceneBase.hpp"
#include <Prototypes/SMPLTexturing/ICP.h> 
#include <Prototypes/asicp/asicp.hxx>

#include "json/json.h"
#include <Prototypes/JsonBone/JsonBoneRead.h>
#include <Prototypes/objImport/objImport.h>
#include <Prototypes/HalfEdge/HalfEdge.h>

#include <iostream>
#include <fstream>
#include <Prototypes/Assets/GLTFIO/GLTFIO.hpp>

using namespace Eigen;
using namespace std;

namespace CForge {

	class ExampleICP : public ExampleSceneBase {
	public:
		ExampleICP(void) {
			m_WindowTitle = "CrossForge Example - ICP";
			m_WinWidth = 1280;
			m_WinHeight = 720;
		}//Constructor

		~ExampleICP(void) {
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

			// load skydome and a textured cube
			// initGroundPlane(&m_RootSGN, 100.0f, 20.0f);
			
			// load the Scan (Timon / Staatstest)
            // SAssetIO::load("MyAssets/Reko/2017_10_11_Staatstest_6tex.obj", &M);
			SAssetIO::load("MyAssets/Reko_Timo/Timon_A_20k.obj", &m_TimonMesh);
            m_TimonMesh.computePerVertexNormals();
			m_TimonMesh.computeAxisAlignedBoundingBox();
            
            for (size_t i = 0; i < m_TimonMesh.vertexCount(); i++)
            {
                m_ScanVertices.push_back(m_TimonMesh.vertex(i)); 
            }
			// mergeRedundantVertices(&m_TimonMesh);
			m_Scan.init(&m_TimonMesh); 			
            m_ScanTransformSGN.init(&m_RootSGN); // Vector3f(0.0f, 0.0f, 0.0f)
			m_ScanTransformSGN.translation(Vector3f(1.f, 1.25f, 0.0f));
			m_ScanTransformSGN.rotation(Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), Vector3f::UnitY())));
            m_ScanSGN.init(&m_ScanTransformSGN, &m_Scan);

			// load SMPLX model
			// SAssetIO::load("MyAssets/010.obj", &m_SMPLXMesh);
			objImportExport::storeInMesh("MyAssets/010.obj", &m_SMPLXMesh);
			// SAssetIO::load("MyAssets/Reko_Timo/test_own_export_from_python.obj", &m_SMPLXMesh);
			SAssetIO::load("MyAssets/Reko_Timo/test_own_export_from_python.obj", &m_ProxyMesh); 

            m_SMPLXMesh.computePerVertexNormals();
			m_SMPLXMesh.computeAxisAlignedBoundingBox();

            for (size_t i = 0; i < m_SMPLXMesh.vertexCount(); i++)
            {
                m_SMPLXVertices.push_back(m_SMPLXMesh.vertex(i)); 
			}		
			m_Reconstruction.init(&m_SMPLXMesh);
			m_ReconstructionTransformSGN.init(&m_RootSGN); // Vector3f(0.0f, 0.0f, 0.0f)
			m_ReconstructionTransformSGN.translation(Vector3f(-1.0f, 1.25f, 0.0f));
			m_ReconstructionTransformSGN.rotation(Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), Vector3f::UnitY())));
			m_ReconstructionSGN.init(&m_ReconstructionTransformSGN, &m_Reconstruction);
			
			// //  Rotating every point -> also need to do it for the whole model!
			// Quaternionf QM = Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitX()));
			// Matrix4f Mat = CForgeMath::rotationMatrix(QM);
			// for (size_t i = 0; i < m_ScanVertices.size(); i++)
			// {
			// 	Vector4f M_v = Mat * Vector4f(m_ScanVertices[i].x(), m_ScanVertices[i].y(), m_ScanVertices[i].z(), 1.0f); 
			// 	m_ScanVertices[i] = Vector3f(M_v.x(), M_v.y(), M_v.z());
			// }
				
			// Rotating every point -> also need to do it for the whole model!
			// Quaternionf QM = Quaternionf(AngleAxis(CForgeMath::degToRad(90.0f), Vector3f::UnitZ()));
			// Matrix4f Mat = CForgeMath::rotationMatrix(QM);
			// for (size_t i = 0; i < m_ScanVertices.size(); i++)
			// {
			// 	Vector4f M_v = Mat * Vector4f(m_ScanVertices[i].x(), m_ScanVertices[i].y(), m_ScanVertices[i].z(), 1.0f); 
			// 	m_ScanVertices[i] = Vector3f(M_v.x(), M_v.y(), M_v.z());
			// }
				
			// for (size_t i = 0; i < m_TimonMesh.vertexCount(); i++)
			// {
			// 	m_TimonMesh.vertex(i) = m_ScanVertices[i]; 
			// }
			// m_Scan.init(&m_TimonMesh);		

			// create help text
			LineOfText* pKeybindings = new LineOfText();
			pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "F1: Toggle help text | 1: ICP | Shift+1: scaling ICP | 2: Hausdorff Distance | 3: Rotate Mismatch | 4: Texture Transfer Naive | 5: Texture Transfer Normals | 6: Save Model | 7: Save Model with Skeleton"); 
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

			//pHE_SMPLX->initHEDataStructure(&m_SMPLXMesh);

			// std::string ErrorMsg;
			// if (0 != CForgeUtility::checkGLError(&ErrorMsg)) {
			// 	SLogger::log("OpenGL Error" + ErrorMsg, "PrimitiveFactoryTestScene", SLogger::LOGTYPE_ERROR);
			// }
			
		}//initialize

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
			// }
			
			return correspondingIndex; 
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

			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_2, true)){
				std::cout<<"Hausdorff Distance: "<< hausdorffDistance(m_ScanVertices, m_SMPLXVertices)<<std::endl;
			}

			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_LEFT_SHIFT) && m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_1)) {
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_1, Keyboard::KEY_RELEASED);

				int maxItterations = 100; 
				
				Eigen::MatrixXd X(3, m_ScanVertices.size());
				for (uint32_t i = 0; i < m_ScanVertices.size(); i++) {
					X.col(i) = m_ScanVertices[i].cast<double>();
				}
				Eigen::MatrixXd Y(3, m_SMPLXVertices.size());
				for (uint32_t i = 0; i < m_SMPLXVertices.size(); i++) {
					Y.col(i) = m_SMPLXVertices[i].cast<double>();
				}

				Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
				Eigen::Matrix3d A = Eigen::Matrix3d::Identity(); 
				Eigen::Vector3d t = Eigen::Vector3d::Zero();
				std::vector<size_t> indices;
				double RMSE = 0.0f;

				asicp(X, Y, 1e-9, maxItterations, 1e-14, true, 8, Q, A, t, indices, RMSE); 

				for (size_t i = 0; i < m_ScanVertices.size(); i++)
				{
					m_ScanVertices[i] = (A * Q * m_ScanVertices[i].cast<double>() + t).cast<float>();
				}

				for (size_t i = 0; i < m_TimonMesh.vertexCount(); i++)
				{
					m_TimonMesh.vertex(i) = m_ScanVertices[i]; 
				}
				m_Scan.init(&m_TimonMesh);		
			}
			else if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_1)) {
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_1, Keyboard::KEY_RELEASED);
				
				int maxItterations = 100; 
				ICP::icp(m_ScanVertices, m_SMPLXVertices, maxItterations);
				
				// write the result back 
				for (size_t i = 0; i < m_TimonMesh.vertexCount(); i++)
				{
					m_TimonMesh.vertex(i) = m_ScanVertices[i]; 
				}
				m_Scan.init(&m_TimonMesh);		
			}
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_3)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_3, Keyboard::KEY_RELEASED);
				rotateMismatch();
			}
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_4)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_4, Keyboard::KEY_RELEASED);
				auto start = CForgeUtility::timestamp();
				textureTransferNaive();
				auto end = CForgeUtility::timestamp();
				std::cout<<"Time for naive transfer: "<<end - start << "ms" <<std::endl;
			}
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_5)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_5, Keyboard::KEY_RELEASED);
				auto start = CForgeUtility::timestamp();
				textureTransferNormals();
				auto end = CForgeUtility::timestamp();
				std::cout<<"Time for normal transfer: "<<end - start<< "ms" <<std::endl;
			}
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_6)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_6, Keyboard::KEY_RELEASED);
		
				m_SMPLXMesh.computePerFaceNormals();
				m_SMPLXMesh.computePerVertexNormals(); 
				AssetIO::store("MyAssets/Reko_Timo/test.obj", &m_SMPLXMesh);
				objImportExport::exportAsObjFile("test_own_export.obj", &m_SMPLXMesh);

				std::cout<<"Saved the model"<<std::endl;
			}
			if(m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_7)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_7, Keyboard::KEY_RELEASED);
				std:cout << "Save Model with Skeleton as fbx" << std::endl;
				try
				{
					std::vector<std::vector<int>> correspondingIndex; 
					correspondingIndex = countDoubleVertices(&m_SMPLXMesh, &m_ProxyMesh); 
					insertBones(m_ProxyMesh, correspondingIndex);
					AssetIO::store("MyAssets/Reko_Timo/proxy.fbx", &m_ProxyMesh); 

					// m_SMPLXMesh.computePerFaceNormals();
					// m_SMPLXMesh.computePerVertexNormals(); 
					// AssetIO::store("MyAssets/Reko_Timo/test_with_skeleton.obj", &m_SMPLXMesh);
					// AssetIO::store("MyAssets/Reko_Timo/test_with_skeleton.fbx", &m_SMPLXMesh);

				}
				catch(const std::exception& e)
				{
					std::cerr << e.what() << '\n';
				}
				std::cout << "Successfully saved the model with skeleton" << std::endl;

			}
		}//mainLoop

		void textureTransferNormals(){
			rotateMismatch();
			std::vector<Eigen::Vector3f> uv(m_SMPLXMesh.vertexCount(), Eigen::Vector3f::Zero());
			std::vector<uint32_t> indexTriangle(m_SMPLXVertices.size(), 0); 
			std::vector<std::vector<int32_t>> neighboringTriangles; 

			// only for adjecent triangles do sdf -> if no hit, can do it for all triangles
			std::vector<PointLenght> pt; 
			ICP::findClosestPointsKDTree(&m_SMPLXVertices, &m_ScanVertices, pt);

			for(uint32_t i = 0; i < pt.size(); i++){
				size_t search = pt[i].target;

				std::vector<int32_t> adjTriangles; 
				// get the adjecent triangles of the target point
				for(uint32_t j = 0; j < m_TimonMesh.getSubmesh(0)->Faces.size(); j++){
					T3DMesh<float>::Face v =  m_TimonMesh.getSubmesh(0)->Faces[j]; 
					if(v.Vertices[0] == search || v.Vertices[1] == search || v.Vertices[2] == search){
						adjTriangles.push_back(j);
					}
				}

				// for every adjecent triangle do sdf and get the nearest triangle
				float minDist = std::numeric_limits<float>::max();
				for(uint32_t j = 0; j < adjTriangles.size(); j++){
					T3DMesh<float>::Face face = m_TimonMesh.getSubmesh(0)->Faces[adjTriangles[j]];
					
					Vector3f a = m_TimonMesh.vertex(face.Vertices[0]); 
					Vector3f b = m_TimonMesh.vertex(face.Vertices[1]);
					Vector3f c = m_TimonMesh.vertex(face.Vertices[2]);
					float dist = SDFTriangle(m_SMPLXVertices[i], a, b, c);					

					if(dist < minDist){
						minDist = dist;
						indexTriangle.at(i) = adjTriangles[j];
					}
				}
				neighboringTriangles.push_back(adjTriangles);
				adjTriangles.clear();
				
				// // naive implementation
				// float minDist = std::numeric_limits<float>::max();
				// for(uint32_t j = 0; j < m_TimonMesh.getSubmesh(0)->Faces.size(); j++){
				// 	T3DMesh<float>::Face face = m_TimonMesh.getSubmesh(0)->Faces[j];
				// 	Vector3f a = m_TimonMesh.vertex(face.Vertices[0]); 
				// 	Vector3f b = m_TimonMesh.vertex(face.Vertices[1]);
				// 	Vector3f c = m_TimonMesh.vertex(face.Vertices[2]);
				// 	float dist = SDFTriangle(m_SMPLXVertices[i], a, b, c);

				// calculate normal
				T3DMesh<float>::Face face = m_TimonMesh.getSubmesh(0)->Faces[indexTriangle.at(i)];
				Vector3f a = m_TimonMesh.vertex(face.Vertices[0]); 
				Vector3f b = m_TimonMesh.vertex(face.Vertices[1]);
				Vector3f c = m_TimonMesh.vertex(face.Vertices[2]);

				float eps = 0.002f; 
				auto map = [&](Vector3f v) -> float {return SDFTriangle(v, a, b, c);};
				Vector3f normal = {
					map(m_SMPLXVertices[i] + Vector3f(eps, 0.0f, 0.0f)) - map(m_SMPLXVertices[i] - Vector3f(eps, 0.0f, 0.0f)), 
					map(m_SMPLXVertices[i] + Vector3f(0.0f, eps, 0.0f)) - map(m_SMPLXVertices[i] - Vector3f(0.0f, eps, 0.0f)), 
					map(m_SMPLXVertices[i] + Vector3f(0.0f, 0.0f, eps)) - map(m_SMPLXVertices[i] - Vector3f(0.0f, 0.0f, eps))
					};
				normal.normalize();

				Vector3f projectedPoint = m_SMPLXVertices[i] - normal * minDist;  
				Vector3f uvw = Baryzentric(projectedPoint, a, b, c);

				// nehme an, dass dieser Fall wahr ist
				// uvw.x() = std::clamp(uvw.x(), 0.0f, 1.0f); 
				// uvw.y() = std::clamp(uvw.y(), 0.0f, 1.0f);
				// uvw.z() = std::clamp(uvw.z(), 0.0f, 1.0f);
				//assert(uvw.x() >= 0.0f && uvw.y() >= 0.0f && uvw.z() >= 0.0f);
				//assert(uvw.x() <= 1.0f && uvw.y() <= 1.0f && uvw.z() <= 1.0f);
				
				// transfer the texture coordinates
				// TODO: sind alle werte zwischen 0 und 1? 
				uv[i] = m_TimonMesh.textureCoordinate(face.Vertices[0]) * uvw.x() + m_TimonMesh.textureCoordinate(face.Vertices[1]) * uvw.y() + m_TimonMesh.textureCoordinate(face.Vertices[2]) * uvw.z(); 
			}

			// get the points wich are too far apart in the uv space
			// if they are too far apart we need to clamp them
			// float eps = 0.5; 
			// float tmp = 0.0f; 
			// float tmpMax = 0.0f; 
			// std::vector<HalfEdgeSetup::Vertex*> neighbors; 

			// // was ist der größte Abstand?
			// for(int i = 0; i < m_SMPLXVertices.size(); i++){
			// 	neighbors =  pHE_SMPLX->getAjacentVertices(i);
			// 	for(int vertexIndex = 0; vertexIndex < neighbors.size(); vertexIndex++){
			// 		Vector3f neighborPos = m_SMPLXVertices[neighbors[vertexIndex]->ID];
			// 		tmp += (m_SMPLXVertices[i] - neighborPos).norm() / neighbors.size(); 

			// 		if(tmp > tmpMax){
			// 			tmpMax = tmp; 
			// 		}
			// 		tmp = 0.0f; 
			// 	}
			// }

			m_SMPLXMesh.textureCoordinates(&uv); 
			m_SMPLXMesh.addMaterial(m_TimonMesh.getMaterial(2), true);
			// write uv back to the model 
			for(uint32_t i = 0; i < m_SMPLXMesh.submeshCount(); i++){
				m_SMPLXMesh.getSubmesh(i)->Material = m_SMPLXMesh.materialCount() - 1;
			}
			m_TimonMesh.computePerVertexNormals();
			m_Scan.init(&m_TimonMesh);
			m_Reconstruction.init(&m_SMPLXMesh);
		}

		Vector3f Baryzentric(Vector3f p, Vector3f a, Vector3f b, Vector3f c){
			// https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
			// lamda function for the dot function 
			auto Dot = [](Vector3f v0, Vector3f v1) -> float { return v0.dot(v1); };

			Vector3f v0 = b - a, v1 = c - a, v2 = p - a;
		    float d00 = Dot(v0, v0);
		    float d01 = Dot(v0, v1);
		    float d11 = Dot(v1, v1);
		    float d20 = Dot(v2, v0);
		    float d21 = Dot(v2, v1);
		    float denom = d00 * d11 - d01 * d01;
		    float v = (d11 * d20 - d01 * d21) / denom;
		    float w = (d00 * d21 - d01 * d20) / denom;
		    float u = 1.0f - v - w;

			return Vector3f(u, v, w); 
		}		

		float SDFTriangle(Vector3f p, Vector3f a, Vector3f b, Vector3f c){
            // https://iquilezles.org/articles/distfunctions/ 
            // create the sdf of a Triangle in 3D
            Vector3f ba = b - a, pa = p - a;
            Vector3f cb = c - b, pb = p - b;
            Vector3f ac = a - c, pc = p - c;
            Vector3f nor = ba.cross(ac);

            // create a lamda sign function for a float
			/**
			 * @return 1 if x > 0, -1 if x < 0, 0 if x == 0
			 */
            auto sign = [](float x) -> float { return (0.0f < x) - (x < 0.0f); };
            auto dot2 = [](Vector3f v) -> float { return v.dot(v); };
            auto clamp = [](float x) -> float{ return std::clamp(x, 0.0f, 1.0f); };
            auto minVec = [](Vector3f a, Vector3f b) -> Vector3f { return Vector3f(std::fmin(a.x(), b.x()), std::min(a.y(), b.y()), std::min(a.z(), b.z())); };
			
			/**
			 * @brief check if the point is on the edge of the triangle
			 */
            bool edge = sign(ba.cross(nor).dot(pa)) + sign(cb.cross(nor).dot(pb)) + sign(ac.cross(nor).dot(pc)) < 2.0f;
            
            float vpa = dot2(ba * clamp(ba.dot(pa) / dot2(ba)) - pa); 
            float vpb = dot2(cb * clamp(cb.dot(pb) / dot2(cb)) - pb);
            float vpc = dot2(ac * clamp(ac.dot(pc) / dot2(ac)) - pc);
            float edgeDist = std::min(std::min(vpa, vpb), vpc); 

            float inner = nor.dot(pa) * nor.dot(pa) / dot2(nor);

            return (edge ? sqrt(edgeDist) : sqrt(inner));
        }

		void textureTransferNaive(){
			// align meshes
			rotateMismatch();
			std::vector<PointLenght> pt; 
			ICP::findClosestPointsKDTree(&m_SMPLXVertices, &m_ScanVertices, pt);

			if(pt.size() != m_SMPLXMesh.vertexCount()) throw CForgeExcept("Vertex count of SMPLXMesh and pt are not equal!");			
			
			// for every vertex in smplx mesh
			// apply uv coordinates of timon mesh
			std::vector<Eigen::Vector3f> uv(m_SMPLXMesh.vertexCount(), Eigen::Vector3f::Zero());

			for(uint32_t i = 0; i < uv.size(); i++){
				uv[i] = m_TimonMesh.textureCoordinate(pt[i].target); 
			}
			m_SMPLXMesh.textureCoordinates(&uv); 
			m_SMPLXMesh.addMaterial(m_TimonMesh.getMaterial(2), true);

			// for every submesh in smplx mesh use new material
			for(uint32_t i = 0; i < m_SMPLXMesh.submeshCount(); i++){
				m_SMPLXMesh.getSubmesh(i)->Material = m_SMPLXMesh.materialCount() - 1;
			}
			m_TimonMesh.computePerVertexNormals();
			m_Scan.init(&m_TimonMesh);
			m_Reconstruction.init(&m_SMPLXMesh);
		}

		void rotateMismatch(){
			// caculate the length of all the axis of the two point clouds
			m_TimonMesh.computeAxisAlignedBoundingBox();
			m_SMPLXMesh.computeAxisAlignedBoundingBox();
			CForge::T3DMesh<float>::AABB timonAABB = m_TimonMesh.aabb();
			CForge::T3DMesh<float>::AABB smplxAABB = m_SMPLXMesh.aabb();
			Vector3f lenghtTimon = lenghtAxis(timonAABB.Min, timonAABB.Max);
			Vector3f lenghtSMPLX = lenghtAxis(smplxAABB.Min, smplxAABB.Max);
			Vector3i misMatched = getMismatch(lenghtSMPLX, lenghtTimon);

			if(misMatched[0] == 1) rotateModell(Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitX())));
			if(misMatched[1] == 1) rotateModell(Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitY())));
			if(misMatched[2] == 1) rotateModell(Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitZ())));

			m_TimonMesh.computeAxisAlignedBoundingBox(); timonAABB = m_TimonMesh.aabb();
			m_SMPLXMesh.computeAxisAlignedBoundingBox(); smplxAABB = m_SMPLXMesh.aabb();
			float toScale = smplxAABB.diagonal().norm() / timonAABB.diagonal().norm();
			
			if (m_TimonMesh.vertexCount() != m_ScanVertices.size()) throw CForgeExcept("Vertex count of TimonMesh and ScanVertices are not equal!");			
			for (size_t i = 0; i < m_ScanVertices.size(); i++)
			{
				m_ScanVertices[i] *= toScale;
				m_TimonMesh.vertex(i) = m_ScanVertices[i]; 
			}
			m_Scan.init(&m_TimonMesh);


			auto align = [&]() -> void {
			// align along the axis
			m_TimonMesh.computeAxisAlignedBoundingBox(); timonAABB = m_TimonMesh.aabb();
			m_SMPLXMesh.computeAxisAlignedBoundingBox(); smplxAABB = m_SMPLXMesh.aabb();
			Eigen::Vector3f move = timonAABB.Min - smplxAABB.Min; 

			for (size_t i = 0; i < m_ScanVertices.size(); i++)
			{
				m_ScanVertices[i] = m_ScanVertices[i] - move; 
				m_TimonMesh.vertex(i) = m_ScanVertices[i]; 
			}
			m_Scan.init(&m_TimonMesh);};

			align();
			// need to beat this dist 
			float dist = hausdorffDistance(m_ScanVertices, m_SMPLXVertices);
			if(dist < 0.1f) return;	

			auto rotateAlign = [&](Vector3f unit) -> void{
				rotateModell(Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), unit)));
				align(); 
				float newDist = hausdorffDistance(m_ScanVertices, m_SMPLXVertices);
				if(newDist < 0.1f) return; 
				if(newDist < dist){ dist = newDist;}
				else{
					rotateModell(Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), unit)));
					align(); 
				}
			};

			rotateAlign(Vector3f::UnitX());
			rotateAlign(Vector3f::UnitY());
			rotateAlign(Vector3f::UnitZ());

			if(dist > 0.1f){
				rotateAlign(Vector3f::UnitX());
				rotateAlign(Vector3f::UnitY());
				rotateAlign(Vector3f::UnitZ());
			} 

		}

		// get Vector of lenght of x,y and z-axis, Min: Min of the AABB, Max: Max of the AABB
		Vector3f lenghtAxis(Vector3f Min, Vector3f Max){
			Vector3f Rval;
			Rval.x() = std::abs(Max.x() - Min.x());
			Rval.y() = std::abs(Max.y() - Min.y());
			Rval.z() = std::abs(Max.z() - Min.z());
			return Rval;
		}

		// if the two meshes are misaligned, get the axis of misalignment
		// smplx & timon: lenght of the axis of the aabb
		Vector3i getMismatch(Vector3f smplx, Vector3f timon){
			smplx = smplx.normalized(); timon = timon.normalized(); 
			
			// get the index of the smallest difference between the two axis
			// if the axis are the same, the difference is near 0 -> just search for min
			std::vector<float> diff;
			Vector3i Rval = Vector3i::Zero();
			for(int i = 0; i < smplx.size(); i++){
				for(int j = 0; j < timon.size(); j++){
					diff.push_back(std::abs(smplx[i] - timon[j]));
				}
				// find the index of min value in diff
				auto min = std::min_element(diff.begin(), diff.end());
				int index = std::distance(diff.begin(), min);
				diff.clear();
				Rval[i] = index;
			}
			
			// get the axis to rotate - if (1,2,3) nothing to rotate
			int mistakes = 0; 
			for(int i = 0; i < Rval.size(); i++){
				if(Rval[i] != i) mistakes++;
			}
			// return: 
			// 012 - ok
			// 102 - x | 021 - y | 210 - z
			// 120 - yz | 201 - xz   
			if(mistakes == 0) return Vector3i::Zero();
			else if(mistakes == 2){
				if(Rval[0] == 0) return Vector3i(1, 0, 0);
				else if(Rval[0] == 2) return Vector3i(0, 1, 0);
				else if(Rval[0] == 1) return Vector3i(0, 0, 1); 
			}
			else if(mistakes == 3){
				if(Rval[0] == 1) return Vector3i(0, 1, 1);
				else if(Rval[0] == 2) return Vector3i(1, 0, 1);
			}

			return Vector3i::Zero();
		}

		void rotateModell(Quaternionf QM){
			Matrix4f Mat = CForgeMath::rotationMatrix(QM);
			for (size_t i = 0; i < m_ScanVertices.size(); i++)
			{
				Vector4f M_v = Mat * Vector4f(m_ScanVertices[i].x(), m_ScanVertices[i].y(), m_ScanVertices[i].z(), 1.0f); 
				m_ScanVertices[i] = Vector3f(M_v.x(), M_v.y(), M_v.z());
			}
	
			for (size_t i = 0; i < m_TimonMesh.vertexCount(); i++)
			{
				m_TimonMesh.vertex(i) = m_ScanVertices[i]; 
			}
			m_Scan.init(&m_TimonMesh);
		}

		float hausdorffDistance(const std::vector<Vector3f>& A, const std::vector<Vector3f>& B, bool displayTime = false) {
			// a kd-tree will be created and the closest point will be searched
			float maxDist = 0.0f;

			std::vector<PointLenght> pt; 
			auto start = CForgeUtility::timestamp();
			ICP::findClosestPointsKDTree(&A, &B, pt);
			auto end = CForgeUtility::timestamp();
			if(displayTime) std::cout<<"Time for closest point search: "<<(end - start)<<"ms"<<std::endl;

			int minIndex = 0; 
			// find the max distance in pt
			for (size_t i = 0; i < pt.size(); i++)
			{
				if (pt[i].lenght > maxDist)
				{
					maxDist = pt[i].lenght; 
					minIndex = i;
				}
			}

			return maxDist;
		}//hausdorffDistance

		void insertBones(T3DMesh<float> &mesh){
			const std::string fileName = "MyAssets/result.json";
			std::vector<T3DMesh<float>::Bone*> bones = BuildBones::getBones(fileName);
			mesh.bones(&bones);
		}

		void insertBones(T3DMesh<float> &mesh, std::vector<std::vector<int>> &correspondences){
			const std::string fileName = "MyAssets/result.json";
			std::vector<T3DMesh<float>::Bone*> bones = BuildBones::getBones(fileName, correspondences);
			mesh.bones(&bones);
		}

	protected:

		// Scene Graph
		SGNTransformation m_RootSGN;

        StaticActor m_Scan; 
        SGNGeometry m_ScanSGN;
        SGNTransformation m_ScanTransformSGN;

		StaticActor m_Reconstruction; 
        SGNGeometry m_ReconstructionSGN;
        SGNTransformation m_ReconstructionTransformSGN;

		SkyboxActor m_Skybox;
		vector<string> m_ClearSky;

		SceneGraph m_SkyboxSG;
		SGNTransformation m_SkyboxTransSGN;
		SGNGeometry m_SkyboxGeomSGN;

		CForge::T3DMesh<float>::AABB m_AABBScan;
		CForge::T3DMesh<float>::AABB m_AABBSMPLX; 
		Vector3f m_ScanDiag;
		Vector3f m_SMPLXDiag;

        std::vector<Vector3f> m_ScanVertices;
        std::vector<Vector3f> m_SMPLXVertices;
		Eigen::MatrixXd m_SourceVertices, m_TargetVertices;  

		T3DMesh<float> m_TimonMesh;
		T3DMesh<float> m_SMPLXMesh;

		StaticActor m_Cube;
		SGNTransformation m_CubeTransformSGN;
		SGNGeometry m_CubeSGN;
		T3DMesh<float> m_CubeMesh;

		T3DMesh<float> m_ProxyMesh; 

		HalfEdgeSetup *pHE_SMPLX = new HalfEdgeSetup();
	};//ExampleScanScreenshot

}//name space

#endif