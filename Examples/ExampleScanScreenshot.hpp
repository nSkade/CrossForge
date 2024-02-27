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
#ifndef __CFORGE_EXAMPLESCANSCREENSHOT_HPP__
#define __CFORGE_EXAMPLESCANSCREENSHOT_HPP__


#include "ExampleSceneBase.hpp"
#include <iostream>

using namespace Eigen;
using namespace std;

namespace CForge {

	class ExampleScanScreenshot : public ExampleSceneBase {
	public:
		ExampleScanScreenshot(void) {
			m_WindowTitle = "CrossForge Example - Scan Screenshots";
			m_WinWidth = 1600; //1280
			m_WinHeight = 1200; //720
		}//Constructor

		~ExampleScanScreenshot(void) {
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
			T3DMesh<float> M;

			initGroundPlane(&m_RootSGN, 100.0f, 20.0f);

            //SAssetIO::load("MyAssets/Reko/2017_10_11_Staatstest_6tex.obj", &M);
			SAssetIO::load("MyAssets/Reko_Timo/Timon_A_20k.obj", &M);
            M.computePerVertexNormals();
			M.computeAxisAlignedBoundingBox();
			m_Scan.init(&M);
			m_AABBScan = M.aabb();
			m_ScanDiag = m_AABBScan.Min + (m_AABBScan.diagonal() * 0.5f);			
			M.clear();

            m_ScanTransformSGN.init(&m_RootSGN, Vector3f(0.0f, 0.0f, 0.0f));
            m_ScanSGN.init(&m_ScanTransformSGN, &m_Scan);
            m_ScanSGN.scale(Vector3f(1.0f, 1.0f, 1.0f));
        	m_ScanSGN.rotation(-90.0f, Vector3f::UnitX());

			Quaternionf rot = Quaternionf(AngleAxis(CForgeMath::degToRad(-90.0f), Vector3f::UnitX())) * Quaternionf(AngleAxis(CForgeMath::degToRad(180.0f), Vector3f::UnitZ()));
			m_ScanSGN.rotation(rot);

			// SAssetIO::load("MyAssets/Reko_Timo/010.obj", &M);
            // M.computePerVertexNormals();
			// M.computeAxisAlignedBoundingBox();
			// m_Reconstruction.init(&M);
			// m_AABBSMPLX = M.aabb();
			// m_SMPLXDiag = m_AABBSMPLX.Min + (m_AABBSMPLX.diagonal() * 0.5f);
			// M.clear();

			// m_ReconstructionTransformSGN.init(&m_RootSGN, Vector3f(0.0f, 0.0f, 0.0f));
			// m_ReconstructionSGN.init(&m_ReconstructionTransformSGN, &m_Reconstruction);
			// float toScale = m_AABBScan.diagonal().norm() / m_AABBSMPLX.diagonal().norm(); 
			// m_ReconstructionSGN.scale(Vector3f(toScale, toScale, toScale));
			// m_ReconstructionSGN.position(m_ReconstructionSGN.position() + Vector3f(0.0f, 0.7f, 0.12f));


			// create help text
			LineOfText* pKeybindings = new LineOfText();
			pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Movement: (Shift) + W,A,S,D  | Rotation: LMB/RMB + Mouse | F1: Toggle help text | Cirlce Cam: 1 | Screenshot: F12");
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

			
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_1, true)) {
				Vector3f camPos = 3.25f * circleCameraStep(m_maxCount, m_StepCount) + Vector3f(0.0f, 2.5f, 0.0f);
				m_Cam.lookAt(camPos,  Vector3f(0.0f, 1.2f, 0.0f));

				//camPos = 4.0f * circleCameraStep(m_maxCount, m_StepCount) + Vector3f(0.0f, 1.5f, 0.0f);
				//m_Cam.lookAt(camPos,  Vector3f(0.0f, 1.5f, 0.0f));
				m_StepCount = (m_StepCount + 1) % m_maxCount; 
				
			}
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_C, true)) {
				m_orthographicCam = !m_orthographicCam;

				if (!m_orthographicCam) {
					m_Cam.projectionMatrix(m_WinWidth, m_WinHeight, CForgeMath::degToRad(35.0f), 0.1f, 1000.0f);
				}
			}
			if (m_orthographicCam) {
				float scale = m_Cam.position().norm();
				m_Cam.orthographicProjection(-1.0f * m_WinWidth / m_WinHeight * scale, 1.0f * m_WinWidth / m_WinHeight * scale, -1.0f * scale, 1.0f * scale, -1000.0f, 1000.0f);
			}
			if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_F12, true)) {
				// when multible screenshots need to be taken, it needs to be in different frames
				// we just do it x-times in the mainLoop
				m_takeScreenShot = !m_takeScreenShot; 
			}

			if(m_takeScreenShot){
				// take multible screenshots
				takeScreenshotsOfPerson();
			}

		}//mainLoop

        // get position of the camera 
        Vector3f circleCameraStep(int camTotal, int camIdx){
			double angle = (2.0 * EIGEN_PI * (double)camIdx) / (double)camTotal;
            return Vector3f(sin(angle), 0.0f, cos(angle));
        }//circleStep

		void takeScreenshotsOfPerson(){
			if (m_waitFrames > 3) {
				m_waitFrames--;
				return;
			}
			if(m_StepCount == m_maxCount){
				m_takeScreenShot = false;
				m_StepCount = 0; 
				return; 
			}
			
			// Vector3f camPos = 4 * circleCameraStep(m_maxCount, m_StepCount) + Vector3f(0.0f, 1.5f, 0.0f); // m_Cam.lookAt(camPos,  Vector3f(0.0f, 1.5f, 0.0f)); 
			Vector3f camPos = 3.25f * circleCameraStep(m_maxCount, m_StepCount) + Vector3f(0.0f, 2.5f, 0.0f);
			m_Cam.lookAt(camPos,  Vector3f(0.0f, 1.2f, 0.0f));
			// the camera needs a frame (?) to be updated - so we need to wait 
			if(m_waitFrameBeforeScreenshot > 0){
				m_waitFrameBeforeScreenshot--;
				return;
			}	
			
			
			//std::cout<<"Take Screenshot: "<<m_StepCount<<std::endl;
			//std::cout<<m_Cam.position().x()<<", "<<m_Cam.position().z()<<std::endl;
			// From ExampleSceneBase.hpp
			m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
			std::string ScreenshotURI = "Screenshots/Screenshot_" + std::to_string(m_ScreenshotCount++) + "." + m_ScreenshotExtension;
			takeScreenshot(ScreenshotURI);
			m_StepCount++;
			m_waitFrames+=2;
			m_waitFrameBeforeScreenshot += 2; 

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

		int32_t m_StepCount = 0; 
		int32_t m_maxCount = 8;   
		int32_t m_waitFrames = 2; 
		int32_t m_waitFrameBeforeScreenshot = 2;
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


	};//ExampleScanScreenshot

}//name space

#endif