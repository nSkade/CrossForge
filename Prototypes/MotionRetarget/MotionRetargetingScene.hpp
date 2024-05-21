#pragma once

#include <Examples/exampleSceneBase.hpp>

#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>

#include <Prototypes/Assets/GLTFIO/GLTFIO.hpp>

#include <crossforge/Graphics/Actors/SkeletalActor.h>

#include <Prototypes/GUI/ImGuiUtility.h>

#include <Prototypes/MotionRetarget/UI/IKImGui.hpp>
#include <Prototypes/MotionRetarget/UI/Guizmo.hpp>

#include <Prototypes/MotionRetarget/Config/Config.hpp>

#include "IKSkeletalActor.h"
#include "IKStickFigureActor.h"

#include "UI/ImGuiStyle.hpp"
#include "UI/IKSequencer.hpp"

namespace CForge {

class MotionRetargetingScene : public ExampleSceneBase {
public:
	MotionRetargetingScene(void) {
		m_WindowTitle = "CForge Motion Retarget Editor";
	}//Constructor

	~MotionRetargetingScene(void) {
		clear();
	}//Destructor

	void init() override{

		initWindowAndRenderDevice();
		gladLoadGL();
		initCameraAndLights();

		// build scene graph
		m_RootSGN.init(nullptr);
		m_SG.init(&m_RootSGN);

		//initGroundPlane(&m_RootSGN, 100.0f, 20.0f);
		initFPSLabel();
		initSkybox();
		initCharacter(); // initialize character & animation controller
		initEndEffectorMarkers(); // initialize end-effector & target markers
		
		LineOfText* pKeybindings = new LineOfText();
		pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Movement:(Shift) + W,A,S,D  | Rotation: LMB/RMB + Mouse | F1: Toggle help text");
		m_HelpTexts.push_back(pKeybindings);
		pKeybindings->color(0.0f, 0.0f, 0.0f, 1.0f);
		m_DrawHelpTexts = false;

		// check whether a GL error occurred
		std::string GLError = "";
		CForgeUtility::checkGLError(&GLError);
		if (!GLError.empty()) printf("GLError occurred: %s\n", GLError.c_str());

		ImGuiUtility::initImGui(&m_RenderWin);

		T3DMesh<float> M;
		SAssetIO::load("MyAssets/ccd-ik/joint.obj", &M);
		m_JointVisActor.init(&M);
		
		glClearColor(.3f,.3f,.3f,.0f);
		SetupImGuiStyle(true,0.5f);

		m_RenderWin.position(0,31);
		m_RenderWin.size(1920,1009);

		m_config.baseLoad();
		m_config.load(&m_Cam);
	}//initialize

	void clear(void) override{
		ExampleSceneBase::clear();

		m_EndEffectors.clear();

		for (auto& EffTransforms : m_EffectorTransformSGNs) {
			for (auto pSGN : EffTransforms.second) if (pSGN != nullptr) delete pSGN;
		}
		m_EffectorTransformSGNs.clear();

		for (auto& EffGeoms : m_EffectorGeomSGNs) {
			for (auto pSGN : EffGeoms.second) if (pSGN != nullptr) delete pSGN;
		}
		m_EffectorGeomSGNs.clear();

		for (auto& TargetTransforms : m_TargetTransformSGNs) {
			for (auto pSGN : TargetTransforms.second) if (pSGN != nullptr) delete pSGN;
		}
		m_TargetTransformSGNs.clear();

		for (auto& TargetGeoms : m_TargetGeomSGNs) {
			for (auto pSGN : TargetGeoms.second) if (pSGN != nullptr) delete pSGN;
		}
		m_TargetGeomSGNs.clear();

		ImGuiUtility::shutdownImGui();

		m_config.store(&m_Cam);
		m_config.baseStore();
	}

	void mainLoop(void)override {
		m_RenderWin.update();
		m_SG.update(60.0f / m_FPS);

		if (m_IKCupdate || m_IKCupdateSingle) {
			m_IKController->update(60.0f / m_FPS);
			m_IKCupdateSingle = false;
		}
		updateEndEffectorMarkers();
		
		if (m_useGuizmo && m_LastSelectedEffectorTarget != -1) {
			m_guizmo.active(true);
			dragTarget(m_LastSelectedEffectorTarget);
		}
		else {
			m_guizmo.active(false);
		}
		if (!m_RenderWin.mouse()->buttonState(Mouse::BTN_LEFT) && m_LMBDownLastFrame) {
			pickTarget();
			//dragTarget(m_SelectedEffectorTarget);
		} else {
			defaultCameraUpdate(&m_Cam, m_RenderWin.keyboard(), m_RenderWin.mouse(), 0.05f, .7f, 32.0f);
		}
		m_LMBDownLastFrame = m_RenderWin.mouse()->buttonState(Mouse::BTN_LEFT);

		m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
		m_RenderDev.activeCamera(const_cast<VirtualCamera*>(m_Sun.camera()));
		m_SG.render(&m_RenderDev);
		
		m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
		m_RenderDev.activeCamera(&m_Cam);
		m_SG.render(&m_RenderDev);

		m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);
		
		m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
		//m_SkyboxSG.render(&m_RenderDev);

		renderVisualizers();

		if (m_pAnimCurr) {
			if (m_animAutoplay) {
				m_animFrameCurr = m_pAnimCurr->t*m_pAnimCurr->TicksPerSecond;
				m_pAnimCurr->t += 1.f/m_FPS*m_pAnimCurr->Speed;
				if (m_pAnimCurr->t > m_pAnimCurr->Duration)
					m_pAnimCurr->t = 0.f;
			} else {
				m_pAnimCurr->t = m_animFrameCurr/m_pAnimCurr->TicksPerSecond;
			}
			m_IKController->updateBones(m_pAnimCurr);
			m_IKController->updateEndEffectorPoints();
		}

		renderUI();
		m_RenderWin.swapBuffers();
		
		updateFPS();
		defaultKeyboardUpdate(m_RenderWin.keyboard());
	}//mainLoop

	//TODO(skade) put in seperate class
	void renderUI();
	void renderUIAnimation();

private:
	void rayCast(Eigen::Vector3f* ro, Eigen::Vector3f* rd) {
		Vector4f Viewport = Vector4f(0.0f, 0.0f, float(m_RenderWin.width()), float(m_RenderWin.height())); 
		Vector2f CursorPos = Vector2f(m_RenderWin.mouse()->position().x(), Viewport(3) - m_RenderWin.mouse()->position().y());
		Matrix4f View = m_Cam.cameraMatrix();
		Matrix4f Projection = m_Cam.projectionMatrix();
		
		igl::unproject_ray(CursorPos, View, Projection, Viewport, *ro, *rd);
		rd->normalize();
	}

	//TODO(skade) implement target bone
	void pickTarget(void) {
		Eigen::Vector3f RayOrigin, RayDirection;
		rayCast(&RayOrigin, &RayDirection);

		for (int32_t i = 0; i < m_EndEffectors.size(); ++i) {
			auto EndEffector = m_EndEffectors[i];
			auto& TargetTransforms = m_TargetTransformSGNs.at(EndEffector.segmentName);

			//TODO(skade) adapt to new structure
			int32_t AABBIndex = EndEffector.jointIK->pEndEffectorData->TargetPosGlobal.cols()-1;
			Vector3f AABBPos = TargetTransforms[0]->translation(); // get end-effector pos

			AlignedBox3f TranslatedAABB = AlignedBox3f(m_TargetMarkerAABB.min() + AABBPos, m_TargetMarkerAABB.max() + AABBPos);

			//TODOf(skade) implement mesh intersection test
			//igl::ray_mesh_intersect()
			//TODOf(skade) check for distance in case on screenspace overlap of multiple aabb

			const float T0 = m_Cam.nearPlane();
			const float T1 = m_Cam.farPlane();
			float TMin, TMax; // minimum and maximum of interval of overlap within [T0, T1] -> not actually used here, but required by function
			if (igl::ray_box_intersect(RayOrigin, RayDirection, TranslatedAABB, T0, T1, TMin, TMax)) {
				m_SelectedEffectorTarget = i;
				m_LastSelectedEffectorTarget = i;

				Vector3f DragPlaneNormal = m_Cam.dir();
				float IntersectionDist = (AABBPos - RayOrigin).dot(DragPlaneNormal) / RayDirection.dot(DragPlaneNormal);
				m_DragStart = RayOrigin + (RayDirection * IntersectionDist);
				m_guizmoMat = CForgeMath::translationMatrix(AABBPos);
				return;
			}
		}
		
		// Only deselect when last pick was nothing.
		if (m_SelectedEffectorTarget == -1)
			m_LastSelectedEffectorTarget = -1;
		m_SelectedEffectorTarget = -1; // assume nothing will be picked
	}//pickTarget

	void dragTarget(int target) {
		if (target == -1)
			return;

		IKController::SkeletalEndEffector EndEffector = m_EndEffectors[target];
		std::vector<SGNTransformation*>& TargetTransforms = m_TargetTransformSGNs.at(EndEffector.segmentName);

		// apply translation to target points in character controller
		m_IKController->translateTarget(EndEffector.segmentName, m_guizmoMat.block<3,1>(0,3));
		
		//Eigen::Matrix3Xf targetPoints = m_IKController->getTargetPoints(pEndEffector->segmentName);
		Eigen::Matrix3Xf targetPoints = m_IKController->getTargetPoints(EndEffector.segmentName); //TODO(skade) remove

		// apply translation to target marker transformation SGNs
		for (int32_t i = 0; i < TargetTransforms.size(); ++i) {
			if (TargetTransforms[i] == nullptr)
				continue;
			TargetTransforms[i]->translation(targetPoints.col(i));
		}
	}//dragTarget

	void initCharacter(void) {
		//TODO(skade) file load gui
		T3DMesh<float> M;
		//SAssetIO::load("MyAssets/ccd-ik/ces/CesiumMan.gltf", &M);
		
		GLTFIO gltfio;
		gltfio.load("MyAssets/ccd-ik/ces/CesiumManZ-90.gltf", &M);
		
		//TODO(skade) fix assimp fbx
		//SAssetIO::load("MyAssets/ccd-ik/ces/ces.fbx", &M);

		//TODO(skade) test
		//gltfio.load("MyAssets/ccd-ik/BrainStem/glTF/BrainStem.gltf", &M);

		setMeshShader(&M, 0.7f, 0.04f);
		M.computePerVertexNormals();

		//TODO(skade) option to export skeletal config

		m_IKController = std::make_unique<IKController>();
		m_IKController->init(&M, "MyAssets/ccd-ik/ces/SkeletonConfig.json");

		//TODO(skade) write option without json
		//m_IKController->init(&M);

		for (uint32_t i=0;i<M.skeletalAnimationCount();++i)
			m_IKController->addAnimationData(M.getSkeletalAnimation(i));

		m_IKActor = std::make_unique<IKSkeletalActor>();
		m_IKActor->init(&M, m_IKController.get());

		m_CharacterSGN.init(&m_RootSGN, m_IKActor.get());

		//TODO(skade) remove/rewrite
		m_IKStickActor.init(&M, m_IKController.get());
		m_IKStickActorSGN.init(&m_RootSGN, &m_IKStickActor);
		m_IKStickActorSGN.enable(true, false);
		M.clear();
	}//initActors

	void initEndEffectorMarkers(void) {
		m_EndEffectors = m_IKController->retrieveEndEffectors();

		T3DMesh<float> M;

		// end-effector actors
		PrimitiveShapeFactory::uvSphere(&M, Vector3f(0.05f, 0.05f, 0.05f), 8, 8);
		for (uint32_t i = 0; i < M.materialCount(); ++i) {
			auto* pMat = M.getMaterial(i);
			pMat->Color = Vector4f(1.0f, 1.0f, 0.0f, 1.0f);
			pMat->Metallic = 0.3f;
			pMat->Roughness = 0.2f;
			pMat->VertexShaderForwardPass.push_back("Shader/ForwardPassPBS.vert");
			pMat->FragmentShaderForwardPass.push_back("Shader/ForwardPassPBS.frag");
			pMat->VertexShaderGeometryPass.push_back("Shader/BasicGeometryPass.vert");
			pMat->FragmentShaderGeometryPass.push_back("Shader/BasicGeometryPass.frag");
			pMat->VertexShaderShadowPass.push_back("Shader/ShadowPassShader.vert");
			pMat->FragmentShaderShadowPass.push_back("Shader/ShadowPassShader.frag");
		}
		M.computePerVertexNormals();
		m_EffectorPos.init(&M);

		for (uint32_t i = 0; i < M.materialCount(); ++i) {
			auto* pMat = M.getMaterial(i);
			pMat->Color = Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
		}
		m_EffectorX.init(&M);

		for (uint32_t i = 0; i < M.materialCount(); ++i) {
			auto* pMat = M.getMaterial(i);
			pMat->Color = Vector4f(0.0f, 1.0f, 0.0f, 1.0f);
		}
		m_EffectorY.init(&M);

		for (uint32_t i = 0; i < M.materialCount(); ++i) {
			auto* pMat = M.getMaterial(i);
			pMat->Color = Vector4f(0.0f, 0.0f, 1.0f, 1.0f);
		}
		m_EffectorZ.init(&M);
		M.clear();

		// target actors
		PrimitiveShapeFactory::cuboid(&M, Vector3f(0.05f, 0.05f, 0.05f), Vector3i(1, 1, 1));
		for (uint32_t i = 0; i < M.materialCount(); ++i) {
			auto* pMat = M.getMaterial(i);
			pMat->Color = Vector4f(1.0f, 0.4f, 0.9f, 1.0f);
			pMat->Metallic = 0.3f;
			pMat->Roughness = 0.2f;
			pMat->VertexShaderForwardPass.push_back("Shader/ForwardPassPBS.vert");
			pMat->FragmentShaderForwardPass.push_back("Shader/ForwardPassPBS.frag");
			pMat->VertexShaderGeometryPass.push_back("Shader/BasicGeometryPass.vert");
			pMat->FragmentShaderGeometryPass.push_back("Shader/BasicGeometryPass.frag");
			pMat->VertexShaderShadowPass.push_back("Shader/ShadowPassShader.vert");
			pMat->FragmentShaderShadowPass.push_back("Shader/ShadowPassShader.frag");
		}
		M.computePerVertexNormals();
		m_TargetPos.init(&M);

		//TODO(skade)
		//for (uint32_t i = 0; i < M.materialCount(); ++i) {
		//	auto* pMat = M.getMaterial(i);
		//	pMat->Color = Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
		//}
		//m_TargetX.init(&M);

		M.clear();

		// target aabb actor
		PrimitiveShapeFactory::cuboid(&M, Vector3f(0.1f, 0.1f, 0.1f), Vector3i(1, 1, 1));
		for (uint32_t i = 0; i < M.materialCount(); ++i) {
			auto* pMat = M.getMaterial(i);
			pMat->Color = Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
			pMat->Metallic = 0.3f;
			pMat->Roughness = 0.2f;
			pMat->VertexShaderForwardPass.push_back("Shader/ForwardPassPBS.vert");
			pMat->FragmentShaderForwardPass.push_back("Shader/ForwardPassPBS.frag");
			pMat->VertexShaderGeometryPass.push_back("Shader/BasicGeometryPass.vert");
			pMat->FragmentShaderGeometryPass.push_back("Shader/BasicGeometryPass.frag");
			pMat->VertexShaderShadowPass.push_back("Shader/ShadowPassShader.vert");
			pMat->FragmentShaderShadowPass.push_back("Shader/ShadowPassShader.frag");
		}
		M.computePerVertexNormals();
		M.computeAxisAlignedBoundingBox();
		m_TargetAABB.init(&M);
		m_TargetMarkerAABB = AlignedBox3f(M.aabb().Min, M.aabb().Max);
		M.clear();

		//TODO(skade) cleanup and abstract
		for (int32_t i = 0; i < m_EndEffectors.size(); ++i) {
			m_EffectorTransformSGNs.try_emplace(m_EndEffectors[i].segmentName);
			m_EffectorGeomSGNs.try_emplace(m_EndEffectors[i].segmentName);
			m_TargetTransformSGNs.try_emplace(m_EndEffectors[i].segmentName);
			m_TargetGeomSGNs.try_emplace(m_EndEffectors[i].segmentName);

			auto& EffectorTransforms = m_EffectorTransformSGNs.at(m_EndEffectors[i].segmentName);
			auto& EffectorGeoms = m_EffectorGeomSGNs.at(m_EndEffectors[i].segmentName);
			auto& TargetTransforms = m_TargetTransformSGNs.at(m_EndEffectors[i].segmentName);
			auto& TargetGeoms = m_TargetGeomSGNs.at(m_EndEffectors[i].segmentName);

			//TODO(skade) memory management
			for (uint32_t i = 0; i < EffectorTransforms.size(); ++i) {
				delete EffectorTransforms[i];
			}
			EffectorTransforms.clear();
			for (uint32_t i = 0; i < EffectorGeoms.size(); ++i) {
				delete EffectorGeoms[i];
			}
			EffectorGeoms.clear();
			for (uint32_t i = 0; i < TargetTransforms.size(); ++i) {
				delete TargetTransforms[i];
			}
			TargetTransforms.clear();
			for (uint32_t i = 0; i < TargetGeoms.size(); ++i) {
				delete TargetGeoms[i];
			}
			TargetGeoms.clear();

			// assign end effector transforms and initialize geometry for every joint in joint chain
			Eigen::Matrix3Xf EndEffectorPoints = m_EndEffectors[i].jointIK->pEndEffectorData->EEPosGlobal;
			Eigen::Matrix3Xf TargetPoints = m_EndEffectors[i].jointIK->pEndEffectorData->TargetPosGlobal;
			for (int32_t j = 0; j < EndEffectorPoints.cols(); ++j) {
				EffectorTransforms.push_back(new SGNTransformation());
				EffectorGeoms.push_back(new SGNGeometry());
				EffectorTransforms.back()->init(&m_EffectorVis, EndEffectorPoints.col(j));
				
				StaticActor* efac = &m_EffectorPos;

				if (j == 0)
					efac = &m_EffectorZ;
				else if (j==EndEffectorPoints.cols()-1)
					efac = &m_EffectorY;
				else
					efac = &m_EffectorX;

				EffectorGeoms[j]->init(EffectorTransforms[j], efac);
			}

			for (int32_t j = 0; j < EndEffectorPoints.cols(); ++j) {
				TargetTransforms.push_back(new SGNTransformation());
				TargetGeoms.push_back(new SGNGeometry());
				EffectorGeoms.back()->init(EffectorTransforms[j], &m_EffectorPos);
				TargetTransforms.back()->init(&m_TargetVis, TargetPoints.col(j));
				TargetGeoms.back()->init(TargetTransforms.back(), &m_TargetPos);
			}

			int32_t AABBIndex = EndEffectorPoints.cols()-1;

			TargetTransforms[AABBIndex] = new SGNTransformation();
			TargetGeoms[AABBIndex] = new SGNGeometry();

			//TODO(skade)
			//Vector3f AABBPos = m_EndEffectors[i]->TargetPoints.rowwise().sum();
			//AABBPos /= m_EndEffectors[i]->TargetPoints.cols();

			Vector3f AABBPos = TargetPoints.col(0);
			
			TargetTransforms[AABBIndex]->init(&m_TargetVis, AABBPos);
			TargetGeoms[AABBIndex]->init(TargetTransforms[AABBIndex], &m_TargetAABB);
			TargetGeoms[AABBIndex]->visualization(SGNGeometry::Visualization::VISUALIZATION_WIREFRAME);
		}
	}//initEndEffectorMarkers

	void updateEndEffectorMarkers(void) {
		for (int32_t i = 0; i < m_EndEffectors.size(); ++i) {
			auto& EffectorTransforms = m_EffectorTransformSGNs.at(m_EndEffectors[i].segmentName);
			Eigen::Matrix3Xf EndEffectorPoints = m_EndEffectors[i].jointIK->pEndEffectorData->EEPosGlobal;
			for (int32_t j = 0; j < EndEffectorPoints.cols(); ++j) {
				EffectorTransforms[j]->translation(EndEffectorPoints.col(j));
			}
		}
	}//updateEndEffectorMarkers

	/**
	 * @brief Render Joints and Constraints
	*/
	void renderVisualizers();

	void defaultCameraUpdate(VirtualCamera* pCamera, Keyboard* pKeyboard, Mouse* pMouse,
	                         const float MovementSpeed = 0.4f, const float RotationSpeed = 1.0f, const float SpeedScale = 4.0f);
private:
	SGNTransformation m_RootSGN;

	// character & controller
	std::unique_ptr<IKSkeletalActor> m_IKActor;
	std::unique_ptr<IKController> m_IKController;
	int m_current_anim_item = 0;

	SGNGeometry m_CharacterSGN;

	IKStickFigureActor m_IKStickActor;
	SGNGeometry m_IKStickActorSGN;

	StaticActor m_JointVisActor;

	SkeletalAnimationController m_SkeletalController;
	SkeletalActor m_SkeletalActor;
	
	// end-effector & target markers
	std::vector<IKController::SkeletalEndEffector> m_EndEffectors;

	SGNTransformation m_EffectorVis;
	std::map<std::string, std::vector<SGNTransformation*>> m_EffectorTransformSGNs;
	std::map<std::string, std::vector<SGNGeometry*>> m_EffectorGeomSGNs;
	StaticActor m_EffectorPos, m_EffectorX, m_EffectorY, m_EffectorZ;

	SGNTransformation m_TargetVis;
	
	std::map<std::string, std::vector<SGNTransformation*>> m_TargetTransformSGNs;
	std::map<std::string, std::vector<SGNGeometry*>> m_TargetGeomSGNs;

	StaticActor m_TargetPos, m_TargetAABB;
	AlignedBox3f m_TargetMarkerAABB;

	//TODO(skade) comment
	int32_t m_SelectedEffectorTarget = -1; // index into m_EndEffectors vector; NOT id of joint inside m_CharacterController
	int32_t m_LastSelectedEffectorTarget = -1; // index into m_EndEffectors vector; NOT id of joint inside m_CharacterController

	bool m_LMBDownLastFrame = false;
	Vector3f m_DragStart = Vector3f::Zero();

	//TODO(skade) sort
	bool m_IKCupdate = false;
	bool m_IKCupdateSingle = false;
	bool m_useGuizmo = true;
	bool m_renderDebugGrid = true;
	float m_gridSize = 3.f;
	bool m_visualizeJoints = true;
	bool m_showEffector = true;
	bool m_showTarget = true;
	Eigen::Matrix4f m_guizmoMat = Eigen::Matrix4f::Identity();

	// Anim Gui
	int m_animFrameCurr = 0;
	SkeletalAnimationController::Animation* m_pAnimCurr = nullptr; //TODO(skade) rename
	int m_animAutoplay = false;

	Vector2f m_prevScroll = Vector2f::Zero();

	IKImGui m_gui;
	Guizmo m_guizmo;
	Config m_config;
};//MotionRetargetingScene

}//CForge

