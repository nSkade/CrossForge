/*****************************************************************************\
*                                                                           *
* File(s): ExampleScanScreenshot.hpp                                        *
*                                                                           *
* Content: Example scene that shows minimum setup with an OpenGL capable    *
*          window, lighting setup, and a single moving object.              *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
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
			
			// Rotating every point -> also need to do it for the whole model!
			Quaternionf QM = Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitX()));
			Matrix4f Mat = CForgeMath::rotationMatrix(QM);
			for (size_t i = 0; i < m_ScanVertices.size(); i++)
			{
				// here you can also implement a scaling, if ICP is not capable of 
				Vector4f M_v = Mat * Vector4f(m_ScanVertices[i].x(), m_ScanVertices[i].y(), m_ScanVertices[i].z(), 1.0f); 
				m_ScanVertices[i] = Vector3f(M_v.x(), M_v.y(), M_v.z());
			}
				
			for (size_t i = 0; i < m_TimonMesh.vertexCount(); i++)
			{
				m_TimonMesh.vertex(i) = m_ScanVertices[i]; 
			}
			m_Scan.init(&m_TimonMesh);		

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
				TotalRT TotalRT; 
				TotalRT = ICP::icp(m_ScanVertices, m_SMPLXVertices, maxItterations);
				
				// write the result back 
				for (size_t i = 0; i < m_TimonMesh.vertexCount(); i++)
				{
					m_TimonMesh.vertex(i) = m_ScanVertices[i]; 
				}
				m_Scan.init(&m_TimonMesh);		
			}
		}//mainLoop

        // get position of the camera 
        Vector3f circleCameraStep(int camTotal, int camIdx){
			double angle = (2.0 * EIGEN_PI * (double)camIdx) / (double)camTotal;
            return Vector3f(sin(angle), 0.0f, cos(angle));
        }//circleStep

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

		// Skybox
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