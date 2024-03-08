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
#include <iostream>
#include <Prototypes/SMPLTexturing/ICP.h> 
#include <Prototypes/asicp/asicp.hxx>

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

			m_Scan.init(&m_TimonMesh); 			
            m_ScanTransformSGN.init(&m_RootSGN); // Vector3f(0.0f, 0.0f, 0.0f)
			m_ScanTransformSGN.translation(Vector3f(1.5f, 0.0f, 0.0f));
            m_ScanSGN.init(&m_ScanTransformSGN, &m_Scan);

			// load SMPLX model
			SAssetIO::load("MyAssets/Reko_Timo/010.obj", &m_SMPLXMesh);
            m_SMPLXMesh.computePerVertexNormals();
			m_SMPLXMesh.computeAxisAlignedBoundingBox();

            for (size_t i = 0; i < m_SMPLXMesh.vertexCount(); i++)
            {
                m_SMPLXVertices.push_back(m_SMPLXMesh.vertex(i)); 
			}		
			m_Reconstruction.init(&m_SMPLXMesh);
			m_ReconstructionTransformSGN.init(&m_RootSGN); // Vector3f(0.0f, 0.0f, 0.0f)
			m_ReconstructionSGN.init(&m_ReconstructionTransformSGN, &m_Reconstruction);
			
			// //  Rotating every point -> also need to do it for the whole model!
			// Quaternionf QM = Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitX()));
			// Matrix4f Mat = CForgeMath::rotationMatrix(QM);
			// for (size_t i = 0; i < m_ScanVertices.size(); i++)
			// {
			// 	// here you can also implement a scaling, if ICP is not capable of 
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

			
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_C, true)) {
				Vector3f camPos = 4 * circleCameraStep(m_maxCount, m_StepCount) + Vector3f(0.0f, 1.5f, 0.0f);
				m_Cam.lookAt(camPos,  Vector3f(0.0f, 1.5f, 0.0f));
				m_StepCount = (m_StepCount + 1) % m_maxCount; 
				
			}
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

			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_3)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_3, Keyboard::KEY_RELEASED);
				haudsdorffTransformation();
			}

			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_4)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_4, Keyboard::KEY_RELEASED);
				textureTransferNaive();
			}
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_5)){
				m_RenderWin.keyboard()->keyState(Keyboard::KEY_5, Keyboard::KEY_RELEASED);
				textureTransferNormals();
			}
		}//mainLoop

        // get position of the camera 
        Vector3f circleCameraStep(int camTotal, int camIdx){
			double angle = (2.0 * EIGEN_PI * (double)camIdx) / (double)camTotal;
            return Vector3f(sin(angle), 0.0f, cos(angle));
        }//circleStep

		void textureTransferNormals(){
			haudsdorffTransformation();
			std::vector<Eigen::Vector3f> uv(m_SMPLXMesh.vertexCount(), Eigen::Vector3f::Zero());

			vector<uint32_t> indexTriangle(m_SMPLXVertices.size(), 0); 
			// for every point in the smplx mesh to timon 

			// only for adjecent triangles do sdf -> if no hit, can do it for all triangles
			std::vector<PointLenght> pt; 
			ICP::findClosestPointsKDTree(&m_SMPLXVertices, &m_ScanVertices, pt);
			auto start = CForgeUtility::timestamp();

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
				// if empty, just use nearest point
				if (adjTriangles.size() == 0){
					adjTriangles.push_back(pt.at(i).target);
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
				adjTriangles.clear();

				// calculate normal
				T3DMesh<float>::Face face = m_TimonMesh.getSubmesh(0)->Faces[indexTriangle.at(i)];
				Vector3f a = m_TimonMesh.vertex(face.Vertices[0]); 
				Vector3f b = m_TimonMesh.vertex(face.Vertices[1]);
				Vector3f c = m_TimonMesh.vertex(face.Vertices[2]);

				float eps = 0.002f; 
				auto map = [&](Vector3f v) -> float { SDFTriangle(v, a, b, c);};
				Vector3f normal = {
					map(m_SMPLXVertices[i] + Vector3f(eps, 0.0f, 0.0f)) - map(m_SMPLXVertices[i] - Vector3f(eps, 0.0f, 0.0f)), 
					map(m_SMPLXVertices[i] + Vector3f(0.0f, eps, 0.0f)) - map(m_SMPLXVertices[i] - Vector3f(0.0f, eps, 0.0f)), 
					map(m_SMPLXVertices[i] + Vector3f(0.0f, 0.0f, eps)) - map(m_SMPLXVertices[i] - Vector3f(0.0f, 0.0f, eps))
					};
				normal.normalize();

				Vector3f projectedPoint = m_SMPLXVertices[i] + normal * minDist;  
				Vector3f uvw = Baryzentric(projectedPoint, a, b, c);
				
				// transfer the texture coordinates
				// TODO: sind alle werte zwischen 0 und 1? 
				uv[i] = m_TimonMesh.textureCoordinate(face.Vertices[0]) * uvw.x() + m_TimonMesh.textureCoordinate(face.Vertices[1]) * uvw.y() + m_TimonMesh.textureCoordinate(face.Vertices[2]) * uvw.z(); 
			}
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
			Vector3f v0 = b - a, v1 = c - a, v2 = p - a;
			float d00 = v0.dot(v0), d01 = v0.dot(v1), d11 = v1.dot(v1);
			float d20 = v2.dot(v0), d21 = v2.dot(v1), denom = d00 * d11 - d01 * d01; 
			Vector3f Rval; 
			Rval.y() = (d11 * d20 - d01 * d21) / denom;
			Rval.z() = (d00 * d21 - d01 * d20) / denom;
			Rval.x() = 1.0f - Rval.x() - Rval.y();
			return Rval;
		}		

		float SDFTriangle(Vector3f p, Vector3f a, Vector3f b, Vector3f c){
			// https://iquilezles.org/articles/distfunctions/ 
			// create the sdf of a Triangle in 3D
			Vector3f ba = b - a, pa = p - a;
			Vector3f cb = c - b, pb = p - b;
			Vector3f ac = a - c, pc = p - c;
			Vector3f nor = ba.cross(ac);

			// create a lamda sign function for a float
			auto sign = [](float x) -> float { return (0.0f < x) - (x < 0.0f); };
			auto dot2 = [](Vector3f v) -> float { return v.dot(v); };
			auto clamp = [](float x) -> float{ return std::clamp(x, 0.0f, 1.0f); };
			auto minVec = [](Vector3f a, Vector3f b) -> Vector3f { return Vector3f(std::fmin(a.x(), b.x()), std::min(a.y(), b.y()), std::min(a.z(), b.z())); };

			bool edge = sign(ba.cross(nor).dot(pa)) + sign(cb.cross(nor).dot(pb)) + sign(ac.cross(nor).dot(pc)) < 2.0f;
			
			Vector3f vpa = Vector3f(dot2(ba * clamp(ba.dot(pa) / dot2(ba))), 0.0f, 1.0f) - pa; 
			Vector3f vpb = Vector3f(dot2(cb * clamp(cb.dot(pb) / dot2(cb))), 0.0f, 1.0f) - pb;
			Vector3f vpc = Vector3f(dot2(ac * clamp(ac.dot(pc) / dot2(ac))), 0.0f, 1.0f) - pc;
			Vector3f edgeVec = minVec(minVec(vpa, vpb), vpc); 

			float inner = nor.dot(pa) * nor.dot(pa) / dot2(nor);

			return (edge ? edgeVec.norm() : sqrt(inner));
			
		}

		void textureTransferNaive(){
			// align meshes
			haudsdorffTransformation();
			std::vector<PointLenght> pt; 
			ICP::findClosestPointsKDTree(&m_SMPLXVertices, &m_ScanVertices, pt);

			if(pt.size() != m_SMPLXMesh.vertexCount()) throw CForgeExcept("Vertex count of SMPLXMesh and pt are not equal!");
			
			// uint32_t s = m_SMPLXMesh.textureCoordinatesCount();
			
			// get texture of timon
			
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

		/* The idea is that the two point clouds are aligned at least along on axis
		no we can compute the rotation along one axis of the two point clouds
		the last step is to compute whether the meshes stand on their feet or head
		this can be done with hausdorff (if the distance is minimal then they are aligned) */
		void haudsdorffTransformation(){
			m_TimonMesh.computeAxisAlignedBoundingBox();
			m_SMPLXMesh.computeAxisAlignedBoundingBox();
			CForge::T3DMesh<float>::AABB timonAABB = m_TimonMesh.aabb();
			CForge::T3DMesh<float>::AABB smplxAABB = m_SMPLXMesh.aabb();
			
			// find the max value for the max index
			Eigen::Matrix3f tmp = timonAABB.Max * smplxAABB.Max.transpose(); 
			Eigen::Index maxRow, maxCol;
			tmp.maxCoeff(&maxRow, &maxCol);

			// the two point clouds are already aligned
			if(maxRow == maxCol){
				std::cout<<"The two point clouds are already aligned"<<std::endl;
				return;
			}
			
			// the axis to rotate to is the sum of the maxRow and maxCol 
			// 1: z; 2: y; 3: x
			int8_t axis = maxRow + maxCol; 
			Quaternionf QM;

			switch (axis)
			{
			case 1:
				QM = Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitZ()));
				rotateModell(QM);
				break;
			case 2: 
				QM = Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitY()));
				rotateModell(QM);	
				break;
			default:
				QM = Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitX()));
				rotateModell(QM);
				break;
			} 

			// align the two point clouds and meshes along their height 
			// before that we need to scale the two point clouds to the same size
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

			// align along the y-axis
			m_TimonMesh.computeAxisAlignedBoundingBox(); timonAABB = m_TimonMesh.aabb();
			m_SMPLXMesh.computeAxisAlignedBoundingBox(); smplxAABB = m_SMPLXMesh.aabb();
			Eigen::Vector3f moveY = timonAABB.Min - smplxAABB.Min; 

			for (size_t i = 0; i < m_ScanVertices.size(); i++)
			{
				m_ScanVertices[i] = m_ScanVertices[i] - moveY; 
				m_TimonMesh.vertex(i) = m_ScanVertices[i]; 
			}
			m_Scan.init(&m_TimonMesh);

			// now we need to check if the two meshes are standing on their feet or head 
			float distHausdorff = hausdorffDistance(m_ScanVertices, m_SMPLXVertices);
			std::cout << "Hausdorfdistance:" << distHausdorff << std::endl;
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

		float hausdorffDistance(const std::vector<Vector3f>& A, const std::vector<Vector3f>& B) {
			// a kd-tree will be created and the closest point will be searched
			float maxDist = 0.0f;

			std::vector<PointLenght> pt; 
			auto start = CForgeUtility::timestamp();
			ICP::findClosestPointsKDTree(&A, &B, pt);
			auto end = CForgeUtility::timestamp();
			std::cout<<"Time for closest point search: "<<(end - start)<<"ms"<<std::endl;

			// find the max distance in pt
			for (size_t i = 0; i < pt.size(); i++)
			{
				if (pt[i].lenght > maxDist)
				{
					maxDist = pt[i].lenght; 
				}
			}

			// // naive implementation for testing
			// int32_t targetIndex = 0;
			// for (size_t i = 0; i < A.size(); i++) {
			// 	float minDist = std::numeric_limits<float>::max();
			// 	for (size_t j = 0; j < B.size(); j++) {
			// 		float dist = (A[i] - B[j]).norm();
			// 		if (dist < minDist){
			// 			minDist = dist;
			// 			targetIndex = j;
			// 		} 
			// 	}
			// 	minDist = (A[i] - B[targetIndex]).norm();
			// 	if (minDist > maxDist) maxDist = minDist;
			// }

			return maxDist;
		}//hausdorffDistance

	protected:

		// Scene Graph
		SGNTransformation m_RootSGN;

        StaticActor m_Scan; 
        SGNGeometry m_ScanSGN;
        SGNTransformation m_ScanTransformSGN;

		StaticActor m_Reconstruction; 
        SGNGeometry m_ReconstructionSGN;
        SGNTransformation m_ReconstructionTransformSGN;

		int32_t m_StepCount = 0; 
		int32_t m_maxCount = 8;   
		int32_t m_waitFrames = 5; 
		bool m_orthographicCam = false;
		bool m_takeScreenShot = false; 

		// Skyboxauto i = 
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
	};//ExampleScanScreenshot

}//name space

#endif