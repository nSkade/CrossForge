/*****************************************************************************\
*                                                                           *
* File(s): MuscleAnalysis.hpp                                            *
*                                                                           *
* Content: Example scene that shows minimum setup with an OpenGL capable   *
*          window, lighting setup, and a single moving object.              *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* FreeBSD License without any warranty or guaranty to work properly.        *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_VERTEXCOLORTESTSCENE_HPP__
#define __CFORGE_VERTEXCOLORTESTSCENE_HPP__

#include "../../CForge/AssetIO/SAssetIO.h"
#include "../../CForge/AssetIO/MuscleDataInput.h"
#include "../../CForge/Graphics/Shader/SShaderManager.h"
#include "../../CForge/Graphics/STextureManager.h"


#include "../../CForge/Graphics/GLWindow.h"
#include "../../CForge/Graphics/GraphicsUtility.h"
#include "../../CForge/Graphics/RenderDevice.h"

#include "../../CForge/Graphics/Lights/DirectionalLight.h"
#include "../../CForge/Graphics/Lights/PointLight.h"

#include "../../CForge/Graphics/SceneGraph/SceneGraph.h"
#include "../../CForge/Graphics/SceneGraph/SGNGeometry.h"
#include "../../CForge/Graphics/SceneGraph/SGNTransformation.h"

#include "../../CForge/Graphics/Actors/StaticActor.h"

#include "../../Examples/SceneUtilities.hpp"
#include <memory>

using namespace Eigen;
using namespace std;

namespace CForge {




	void vertexColorTestScene(void) {
		SShaderManager* pSMan = SShaderManager::instance();

		std::string WindowTitle = "CForge - Skeletal Animation Example";
		float FPS = 60.0f;

		bool const LowRes = false;

		uint32_t WinWidth = 1280;
		uint32_t WinHeight = 720;

		if (LowRes) {
			WinWidth = 720;
			WinHeight = 576;
		}

		// create an OpenGL capable windows
		GLWindow RenderWin;
		RenderWin.init(Vector2i(100, 100), Vector2i(WinWidth, WinHeight), WindowTitle);

		// configure and initialize rendering pipeline
		RenderDevice RDev;
		RenderDevice::RenderDeviceConfig Config;
		Config.DirectionalLightsCount = 1;
		Config.PointLightsCount = 1;
		Config.SpotLightsCount = 0;
		Config.ExecuteLightingPass = true;
		Config.GBufferHeight = WinHeight;
		Config.GBufferWidth = WinWidth;
		Config.pAttachedWindow = &RenderWin;
		Config.PhysicallyBasedShading = true;
		Config.UseGBuffer = true;
		RDev.init(&Config);

		// configure and initialize shader configuration device
		ShaderCode::LightConfig LC;
		LC.DirLightCount = 1;
		LC.PointLightCount = 1;
		LC.SpotLightCount = 0;
		LC.PCFSize = 0;
		LC.ShadowBias = 0.00001f;
		LC.ShadowMapCount = 1;
		pSMan->configShader(LC);

		// initialize camera
		VirtualCamera Cam;
		Cam.init(Vector3f(20.0f, 5.0f, 45.0f), Vector3f::UnitY());
		Cam.lookAt(Vector3f(10.0f, 5.0f, 35.0f), Vector3f(0.0f, 4.0f, 25.0f), Vector3f::UnitY());
		Cam.projectionMatrix(WinWidth, WinHeight, GraphicsUtility::degToRad(45.0f), 0.1f, 1000.0f);

		// initialize sun (key lights) and back ground light (fill light)
		Vector3f SunPos = Vector3f(-25.0f, 50.0f, -20.0f);
		Vector3f SunLookAt = Vector3f(0.0f, 0.0f, 30.0f);
		Vector3f BGLightPos = Vector3f(0.0f, 5.0f, 60.0f);
		DirectionalLight Sun;
		PointLight BGLight;
		Sun.init(SunPos, (SunLookAt - SunPos).normalized(), Vector3f(1.0f, 1.0f, 1.0f), 5.0f);
		// sun will cast shadows
		Sun.initShadowCasting(1024, 1024, GraphicsUtility::orthographicProjection(20.0f, 20.0f, 0.1f, 1000.0f));
		BGLight.init(BGLightPos, -BGLightPos.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 5.5f, Vector3f(0.0f, 0.0f, 0.0f));

		// set camera and lights
		RDev.activeCamera(&Cam);
		RDev.addLight(&Sun);
		RDev.addLight(&BGLight);

		// load skydome and a textured cube
		T3DMesh<float> M;
		StaticActor Skydome;
		SkeletalActor Eric;
		StaticActor Human;
		SkeletalAnimationController Controller;

		SAssetIO::load("Assets/ExampleScenes/SimpleSkydome.fbx", &M);
		SceneUtilities::setMeshShader(&M, 0.8f, 0.04f);
		M.computePerVertexNormals();
		Skydome.init(&M);
		M.clear();

		//initilize Data and Animation
		// 
		//BAT FILE PATH ABSOLUTE HAS TO BE CHANGED
		// Change Path in bat file to its current location + set Path to Blender 
		//std::system("C:/Users/Megaport/Documents/STUDIUM/CrossForge/crossForge/CrossForge/Assets/MuscleAnalysisPL/pl.bat");


		std::vector<MuscleDataInput::Muscledata> muscledata;
		int frames = 0;
		std::vector<MuscleDataInput::Muscle_allocdata> ma_data;

		// initialize skeletal actor (Eric) and its animation controller
		T3DMesh<float> SkelAnim;
		SAssetIO::load("Assets/tmp/mbmalerigmuscles.fbx", &M);
		SAssetIO::load("Assets/MuscleAnalysisPL/animation_scaled.fbx", &SkelAnim);
		MuscleDataInput::loadDataFile("Assets/MuscleAnalysisPL/3DGaitModel2354-scaled_MuscleAnalysis_FiberForce.sto", &muscledata, &frames);
		MuscleDataInput::loadMuscleAllocation("Assets/MuscleAnalysisPL/Muscle_Allocation.txt", &ma_data);

		M.clearSkeletalAnimations();
		M.addSkeletalAnimation(SkelAnim.getSkeletalAnimation(0), true);

		SceneUtilities::setMeshShader(&M, 0.7f, 0.04f);
		M.computePerVertexNormals();
		Controller.init(&M);

		// assign random collor to material
		for (uint32_t i = 0; i < M.materialCount(); ++i) {
			Vector4f C;
			C.x() = 1.0f;
			C.y() = 1.0f;
			C.z() = 1.0f;
			C.w() = 1.0f;
			M.getMaterial(i)->Color = C;
		}//for[materials]
		Eric.init(&M, &Controller);
		int thei = 0;

		M.bones(nullptr, false);
		Human.init(&M);
		M.clear();

		// build scene graph
		SceneGraph SG;
		SGNTransformation RootSGN;
		RootSGN.init(nullptr);
		SG.init(&RootSGN);

		// add skydome
		SGNGeometry SkydomeSGN;
		SkydomeSGN.init(&RootSGN, &Skydome);
		SkydomeSGN.scale(Vector3f(5.0f, 5.0f, 5.0f));

		// add human model
		SGNGeometry HumanSGN;
		SGNTransformation HumanTransformSGN;
		float HumanScale = 4.0f;
		HumanTransformSGN.init(&RootSGN, Vector3f(0.0f, 1.0f, 0.0f), Quaternionf::Identity());
		//HumanSGN.init(&HumanTransformSGN, &Human);
		//HumanSGN.scale(Vector3f(HumanScale, HumanScale, HumanScale));

		// add skeletal actor to scene graph (Eric)
		SGNGeometry EricSGN;
		SGNTransformation EricTransformSGN;
		EricTransformSGN.init(&RootSGN, Vector3f(0.0f, 0.0f, 0.0f));
		EricSGN.init(&EricTransformSGN, &Eric);
		EricSGN.scale(Vector3f(HumanScale, HumanScale, HumanScale));
		Quaternionf R;
		R = AngleAxisf(GraphicsUtility::degToRad(-90.0f), Vector3f::UnitX());
		EricSGN.rotation(R);

		// stuff for performance monitoring
		uint64_t LastFPSPrint = CoreUtility::timestamp();
		int32_t FPSCount = 0;

		// check wheter a GL error occurred
		std::string GLError = "";
		GraphicsUtility::checkGLError(&GLError);
		if (!GLError.empty()) printf("GLError occurred: %s\n", GLError.c_str());

		uint64_t LastColorChange = CoreUtility::timestamp();
		std::vector<Vector4f> Colors1;
		std::vector<Vector4f> Colors2;
		for (uint32_t i = 0; i < Eric.materialCount(); ++i) {
			Vector4f C1 = Vector4f(CoreUtility::randRange(0.0f, 1.0f), CoreUtility::randRange(0.0f, 1.0f), CoreUtility::randRange(0.0f, 1.0f), 1.0f);
			Vector4f C2 = Vector4f(CoreUtility::randRange(0.0f, 1.0f), CoreUtility::randRange(0.0f, 1.0f), CoreUtility::randRange(0.0f, 1.0f), 1.0f);
			Colors1.push_back(C1);
			Colors2.push_back(C2);
		}
		float AnimationSpeed = 1.0f;
		SkeletalAnimationController::Animation* pAnim = Controller.createAnimation(0, AnimationSpeed, 0.0f);

		// start main loop
		while (!RenderWin.shutdown()) {
			RenderWin.update();
			SG.update(FPS / 60.0f);

			// this will progress all active skeletal animations for this controller
			Controller.update(FPS / 60.0f);

			SceneUtilities::defaultCameraUpdate(&Cam, RenderWin.keyboard(), RenderWin.mouse());

			// if user hits key 1, animation will be played
			// if user also presses shift, animation speed is doubled
			//float AnimationSpeed = 1.0f;

			if (RenderWin.keyboard()->keyPressed(Keyboard::KEY_LEFT_SHIFT)) AnimationSpeed = 2.0f;
			if (RenderWin.keyboard()->keyPressed(Keyboard::KEY_3)) AnimationSpeed = 0.5f;
			if (RenderWin.keyboard()->keyPressed(Keyboard::KEY_4)) AnimationSpeed = 0.25f;
			if (RenderWin.keyboard()->keyPressed(Keyboard::KEY_2)) AnimationSpeed = 1.0f;
			if (RenderWin.keyboard()->keyPressed(Keyboard::KEY_1, true)) {
				pAnim->t = 0.0f;
				Eric.activeAnimation(pAnim);
			}
			pAnim->Speed = AnimationSpeed;

			// Change Color according to muscle values
			//float Alpha = float(CoreUtility::timestamp() - LastColorChange) / 2500.0f;
			for (uint32_t i = 0; i < Eric.materialCount(); ++i) {
				//if()
				Vector4f C = { 1.0f,1.0f,1.0f,1.0f };
				if (ma_data.size() - 1 < i)break;
				if (int(pAnim->t) > frames - 1)break;

				for (int g = 0; g < muscledata.size(); g++) {
					if (muscledata.at(g).name == ma_data.at(i).name) {
						//transform muscledata in Color with bounds
						
						
					

						float muscle_value = 0.0f;
							float abs = ma_data.at(i).max - ma_data.at(i).min;
							muscle_value = (muscledata.at(g).data.at(int(pAnim->t)) - ma_data.at(i).min) / abs; //TODO: CAP to FRAMES
					

						if (muscle_value > 1.0f) muscle_value = 1.0f;
						else if (muscle_value < 0.0f) muscle_value = 0.0f;

						if (muscle_value > 0.5f) C = { 1.0f,1.0f - muscle_value,0.0f,1.0f };
						else C = { muscle_value * 2,1.0f,0.0f,1.0f };

						Eric.material(i)->color(C);
						break;
					}



				}





			}//for[materials]







			RDev.activePass(RenderDevice::RENDERPASS_SHADOW, &Sun);
			SG.render(&RDev);

			RDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
			SG.render(&RDev);

			RDev.activePass(RenderDevice::RENDERPASS_LIGHTING);

			RenderWin.swapBuffers();

			FPSCount++;
			if (CoreUtility::timestamp() - LastFPSPrint > 1000U) {
				char Buf[64];
				sprintf(Buf, "FPS: %d\n", FPSCount);
				FPS = float(FPSCount);
				FPSCount = 0;
				LastFPSPrint = CoreUtility::timestamp();

				RenderWin.title(WindowTitle + "[" + std::string(Buf) + "]");
			}

			if (RenderWin.keyboard()->keyState(Keyboard::KEY_ESCAPE)) {
				RenderWin.closeWindow();
			}
		}//while[main loop]

		pSMan->release();

	}//exampleMinimumGraphicsSetup

}

#endif