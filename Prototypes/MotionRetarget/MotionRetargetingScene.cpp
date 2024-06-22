#include "MotionRetargetingScene.hpp"

#include "UI/ImGuiStyle.hpp"
#include "UI/IKSequencer.hpp"

#include "IKSolver/IKChain.hpp"

#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>
#include <Prototypes/GUI/ImGuiUtility.h>
#include <Prototypes/Assets/GLTFIO/GLTFIO.hpp>

namespace ImGui {

auto ComboStr = [](const char* label, int* current_item, const std::vector<std::string>& items, int items_count, int height_in_items = -1)
{
	return ImGui::Combo(label, current_item, [](void* data, int idx, const char** out_text) { *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str(); return true; }, (void*)&items, items_count, height_in_items);
};

}//ImGui

namespace CForge {

void MotionRetargetingScene::init() {
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
	m_JointVisActor.material(0)->color(Vector4f(1.,1.,1.,1.));
	
	glClearColor(.3f,.3f,.3f,.0f);
	SetupImGuiStyle(true,0.5f);

	m_RenderWin.position(0,31);
	m_RenderWin.size(1920,1009);

	m_config.baseLoad();
	m_config.load(&m_Cam);
	m_config.load("m_editCam.m_CamIsProj", &m_editCam.m_CamIsProj);
	m_config.load(&m_RenderWin);
	m_editCam.setCamProj(&m_Cam,&m_RenderWin);
}//initialize

void MotionRetargetingScene::clear() {
	m_config.store(m_Cam);
	m_config.store("m_editCam.m_CamIsProj", m_editCam.m_CamIsProj);
	m_config.baseStore();

	ExampleSceneBase::clear();

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
}

void MotionRetargetingScene::initCameraAndLights(bool CastShadows) {
	// initialize camera
	m_Cam.init(Vector3f(0.0f, 3.0f, 8.0f), Vector3f::UnitY());
	m_Cam.projectionMatrix(m_WinWidth, m_WinHeight, CForgeMath::degToRad(45.0f), 0.1f, 1000.0f);

	// initialize sun (key light) and back ground light (fill light)
	Vector3f SunDir = Vector3f(-5.0f, 15.0f, 35.0f);
	Vector3f SunPos = Vector3f(-5.0f, 15.0f, 35.0f);
	m_Sun.init(SunPos, -SunDir.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 5.0f);
	if(CastShadows)
		m_Sun.initShadowCasting(2048, 2048, Vector2i(2, 2), 0.1f, 1000.0f);

	Vector3f BGLightPos = Vector3f(0.0f, 5.0f, -30.0f);
	m_BGLight.init(BGLightPos, -BGLightPos.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 1.5f, Vector3f(0.0f, 0.0f, 0.0f));

	// set camera and lights
	m_RenderDev.activeCamera(&m_Cam);
	m_RenderDev.addLight(&m_Sun);
	m_RenderDev.addLight(&m_BGLight);
}//initCameraAndLights


void MotionRetargetingScene::mainLoop() {

	static uint32_t frameActionIdx = 0;
	static bool frameAction = true;
	frameAction = keyboardAnyKeyPressed() || m_IKCupdate || m_animAutoplay
	              || ImGui::IsAnyItemHovered()
	              || ImGuizmo::IsUsing() || m_guizmoViewManipChanged;  //TODO(skade) guizmo viewManipulate not correct

	//TODO(skade) abstract viewManipulate position into class
	Vector2f mp = m_RenderWin.mouse()->position();
	float viewManipulateRight = m_RenderWin.width();
	float viewManipulateTop = 0.f;
	if (mp.x() > viewManipulateRight-128 && mp.y() < viewManipulateTop+128)
		frameAction = true;

	frameActionIdx = frameAction ? 0 : frameActionIdx+1;
	updateFPS(); //TODOf(skade) improve deltaTime
	if (frameActionIdx < 3) { // render 1 frame after action to update ui
		m_RenderWin.update();
	} else {
		m_RenderWin.updateWait();
		return;
	}

	m_SG.update(60.0f / m_FPS);

	if (m_IKCupdate || m_IKCupdateSingle) {
		m_IKController->update(60.0f / m_FPS);
		m_IKCupdateSingle = false;
	}

	if (m_pAnimCurr) {
		if (m_animAutoplay) {
			m_animFrameCurr = m_pAnimCurr->t*m_pAnimCurr->TicksPerSecond;
			m_pAnimCurr->t += 1.f/m_FPS*m_pAnimCurr->Speed;
			if (m_pAnimCurr->t > m_pAnimCurr->Duration)
				m_pAnimCurr->t = 0.f;
		} else {
			m_pAnimCurr->t = m_animFrameCurr/m_pAnimCurr->TicksPerSecond;
		}
		//TODO(skade) reinitialisiere IK controller joint positions on animation keyframe change
		m_IKController->updateBones(m_pAnimCurr);
	}
	
	// Handle Picking and camera
	if (!ImGui::IsAnyItemHovered() && !ImGui::IsAnyItemActive()) { // !ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)
		if (!m_RenderWin.mouse()->buttonState(Mouse::BTN_LEFT) && m_LMBDownLastFrame) {

			if (!ImGuizmo::IsUsing()) {
				std::vector<std::weak_ptr<IPickable>> p;
				std::vector<std::weak_ptr<IKTarget>> t = m_IKController->getTargets();
				p.assign(t.begin(),t.end());
				m_picker.pick(p);

				std::vector<std::weak_ptr<JointPickable>> jp = m_IKController->getJointPickables();
				p.assign(jp.begin(),jp.end());
				m_picker.pick(p);
				
				m_guizmoMat = m_picker.m_guizmoMat;
			}
		} else
			m_editCam.defaultCameraUpdate(&m_Cam, &m_RenderWin, 0.05f, .7f, 32.0f);

		//if (m_useGuizmo && m_LastSelectedEffectorTarget != -1) {
		if (m_useGuizmo && m_picker.getLastPick())
			m_guizmo.active(true);
		else
			m_guizmo.active(false);

		m_picker.update(m_guizmoMat);

		m_LMBDownLastFrame = m_RenderWin.mouse()->buttonState(Mouse::BTN_LEFT);
	}

	// Render Scene
	m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
	m_RenderDev.activeCamera(const_cast<VirtualCamera*>(m_Sun.camera()));
	m_SG.render(&m_RenderDev);
	
	m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
	m_RenderDev.activeCamera(&m_Cam);
	m_SG.render(&m_RenderDev);

	m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);
	
	m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);

	// Render UI
	renderVisualizers();
	renderUI();

	m_RenderWin.swapBuffers();
	
	m_config.store(m_RenderWin);
	defaultKeyboardUpdate(m_RenderWin.keyboard());
}//mainLoop

void MotionRetargetingScene::initCharacter() {
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
	//m_IKController->init(&M);

	for (uint32_t i=0;i<M.skeletalAnimationCount();++i)
		m_IKController->addAnimationData(M.getSkeletalAnimation(i));

	m_IKActor = std::make_unique<IKSkeletalActor>();
	m_IKActor->init(&M, m_IKController.get());

	m_CharacterSGN.init(&m_RootSGN, m_IKActor.get());
	M.clear();
}//initActors

void MotionRetargetingScene::initEndEffectorMarkers() {
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
	//TODO(skade) dim SPOT IKTarget
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
	m_TargetMarkerAABB = AlignedBox3f(M.aabb().min(), M.aabb().max());
	M.clear();
}//initEndEffectorMarkers

void MotionRetargetingScene::renderUIAnimation() {
	if (!m_IKActor || !m_IKController)
		return;
	//ImGui::SetNextWindowSize(ImVec2(m_WinWidth,m_WinHeight));
	//ImGui::SetNextWindow

	std::vector<std::string> items;
	items.push_back("none");
	for (uint32_t i=0;i<m_IKActor->getController()->animationCount();++i) {
		items.push_back(m_IKActor->getController()->animation(i)->Name);
	}
	ImGui::ComboStr("select animation",&m_current_anim_item,items,items.size());

	if (m_current_anim_item > 0) { // anim selected
		if (m_pAnimCurr && m_current_anim_item - 1 != m_pAnimCurr->AnimationID) { // Animation changed
			m_IKController->destroyAnimation(m_pAnimCurr);
			m_IKActor->activeAnimation(nullptr);
			m_pAnimCurr = nullptr;
		}
		if (!m_pAnimCurr) { // create animation if not existing
			m_pAnimCurr = m_IKController->createAnimation(m_current_anim_item-1,1.f,0.f);
			m_IKActor->activeAnimation(m_pAnimCurr);
		}

		T3DMesh<float>::SkeletalAnimation* anim = m_IKActor->getController()->animation(m_current_anim_item-1);
		ImGui::Text("Duration: %f",anim->Duration);
		ImGui::Text("SamplesPerSecond: %f",anim->SamplesPerSecond);
		if(ImGui::Button("Play")) {
			m_animAutoplay = true;
		}
		if (m_animAutoplay) {
			if(ImGui::Button("Stop")) {
				m_animAutoplay = false;
			}
			ImGui::InputScalar("animSpeed", ImGuiDataType_Float, &(m_pAnimCurr->Speed));
		}
		ImGui::Text("pAnim->t: %f",m_pAnimCurr->t);
	}
	else {
		m_IKController->destroyAnimation(m_pAnimCurr);
		m_IKActor->activeAnimation(nullptr);
		m_pAnimCurr = nullptr;
	}
}

void MotionRetargetingScene::renderUI() {
	ImGuiUtility::newFrame();
	ImGuizmo::BeginFrame();

	static int selectedEntry = -1;
	static int firstFrame = 0;
	static bool expanded = true;

	//m_gui.render(); //TODO(skade)
	{
		ImGui::Begin("Animation");
		renderUIAnimation();
		ImGui::End();
	}

	{
		ImGui::Begin("Sequencer");

		// sequence with default values
		static MySequence mySequence;
		static bool init = false;
		if (!init) {
			mySequence.mFrameMin = -100;
			mySequence.mFrameMax = 1000;
			//mySequence.myItems.push_back(MySequence::MySequenceItem{ 0, 0, int(pAnim->Duration*pAnim->TicksPerSecond), false }); //TODO(skade)
			mySequence.myItems.push_back(MySequence::MySequenceItem{ 0, 0, 14*2, false}); //TODO(skade)
			//mySequence.myItems.push_back(MySequence::MySequenceItem{ 1, 20, 30, true });
			//mySequence.myItems.push_back(MySequence::MySequenceItem{ 3, 12, 60, false });
			//mySequence.myItems.push_back(MySequence::MySequenceItem{ 2, 61, 90, false });
			//mySequence.myItems.push_back(MySequence::MySequenceItem{ 4, 90, 99, false });
			init = true;
		}

		ImGui::PushItemWidth(130);
		ImGui::InputInt("Frame Min", &mySequence.mFrameMin);
		ImGui::SameLine();
		ImGui::InputInt("Frame ", &m_animFrameCurr);
		ImGui::SameLine();
		ImGui::InputInt("Frame Max", &mySequence.mFrameMax);
		ImGui::PopItemWidth();
		Sequencer(&mySequence, &m_animFrameCurr, &expanded, &selectedEntry, &firstFrame,
		          ImSequencer::SEQUENCER_EDIT_STARTEND | ImSequencer::SEQUENCER_ADD | ImSequencer::SEQUENCER_DEL |
		          ImSequencer::SEQUENCER_COPYPASTE | ImSequencer::SEQUENCER_CHANGE_FRAME);
		
		//TODO(skade)
		// add a UI to edit that particular item
		//if (selectedEntry != -1) {
		//	const MySequence::MySequenceItem &item = mySequence.myItems[selectedEntry];
		//	ImGui::Text("I am a %s, please edit me", SequencerItemTypeNames[item.mType]);
		//	// switch (type) ....
		//}
		ImGui::End();
	}

	{
		ImGui::Begin("Tools");
		if (ImGui::CollapsingHeader("Guizmo", ImGuiTreeNodeFlags_Selected)) {
			ImGui::Checkbox("Use guizmo",&m_useGuizmo);
			if (m_useGuizmo)
				m_guizmo.renderOptions();
		}
		if (m_useGuizmo) {
			m_guizmo.setMat(&m_guizmoMat);
			m_guizmo.setCam(&m_Cam);
			m_guizmo.render();
		}

		ImGuizmo::SetRect(0, 0, m_RenderWin.width(), m_RenderWin.height());
		Eigen::Matrix4f cameraMat = m_Cam.cameraMatrix();
		float* cameraView = cameraMat.data();
		float* cameraProjection = m_Cam.projectionMatrix().data();
		Eigen::Matrix4f identityMatrix = Eigen::Matrix4f::Identity();

		float viewManipulateRight = m_RenderWin.width();
		float viewManipulateTop = 0.f;
		float camDistance = cameraMat.block<3,1>(0,3).norm();

		//TODO(skade) improve focus point
		Matrix4f oldView = cameraMat;
		ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(viewManipulateRight - 128, viewManipulateTop), ImVec2(128, 128), 0x10101010);
		if (oldView != cameraMat)
			m_guizmoViewManipChanged = true;
		else
			m_guizmoViewManipChanged = false;
		m_Cam.cameraMatrix(cameraMat);

		if (ImGui::CollapsingHeader("Visualizers", ImGuiTreeNodeFlags_None)) {
			ImGui::Checkbox("Visualize Joints", &m_visualizeJoints);
			ImGui::Checkbox("Visualize Effectors", &m_showEffector);
			ImGui::Checkbox("Show Targets", &m_showTarget);
			ImGui::Checkbox("Render Debug Grid",&m_renderDebugGrid);
			ImGui::InputScalar("Grid Size",ImGuiDataType_Float,&m_gridSize);
			if (m_FPSLabelActive)
				m_FPSLabel.render(&m_RenderDev);
			if (m_DrawHelpTexts)
				drawHelpTexts();
		}

		if (m_renderDebugGrid)
			ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix.data(), m_gridSize);

		ImGui::End();
	}

	{
		ImGui::Begin("IK");
		IKTarget* lp = dynamic_cast<IKTarget*>(m_picker.getLastPick());
		if (lp) {
			IKSegment* seg = m_IKController->getSegment(lp);
			assert(seg);
			ImGui::Text("IK segmentName: %s",seg->name.c_str());
		} else
			ImGui::Text("IK segmentName none");

		std::vector<std::string> items = {
			"IKSS_CCD_F",
			"IKSS_CCD_B",
			"IKSS_CCD_FABRIK"
		};
		int currItem = m_IKController->testIKslvSelect;
		ImGui::ComboStr("ik method",&currItem,items,items.size());
		m_IKController->testIKslvSelect = static_cast<IKController::TestIKslvSelect>(currItem);

		ImGui::Checkbox("enable IK",&m_IKCupdate);
		if (ImGui::Button("singleIK"))
			m_IKCupdateSingle = true;
		ImGui::End();
	}

	//TODO(skade) load/store skeleton, bvh
	//{
	//	ImGui::Begin("main");
	//}

	ImGuiUtility::render();
}//renderUI

void MotionRetargetingScene::renderVisualizers() {
	if (m_visualizeJoints) { //TODO(skade) put in function
		glClear(GL_DEPTH_BUFFER_BIT);
		//glDisable(GL_DEPTH_TEST);
		//glEnable(GL_BLEND);

		//TODO(skade) cleanup
		//m_IKController->renderJointPickables(&m_RenderDev);
		auto& joints = m_IKController->getJointPickables();
		for (auto j : joints) {
			auto jl = j.lock();
			jl->update(Matrix4f::Identity());
			jl->render(&m_RenderDev);
		}

		//glDisable(GL_BLEND);
		//glEnable(GL_DEPTH_TEST);
	}
	glDisable(GL_DEPTH_TEST);
	//glEnable(GL_BLEND); //TODOf(skade) change blendmode in active material forward pass
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	if (m_showEffector)
		m_EffectorVis.render(&m_RenderDev,Eigen::Vector3f::Zero(),Eigen::Quaternionf::Identity(),Eigen::Vector3f(1.f,1.f,1.f));
	if (m_showTarget) {
		std::vector<std::shared_ptr<IKTarget>> t;
		m_IKController->getTargets(&t);
		
		for (uint32_t i = 0; i < t.size(); ++i) {
			//Box aabb = t[i]->bv.aabb();
			m_RenderDev.modelUBO()->modelMatrix(t[i]->pckTransPickin());
			m_TargetPos.render(&m_RenderDev,Quaternionf(),Vector3f(),Vector3f());
		}
	}
		//m_TargetVis.render(&m_RenderDev,Eigen::Vector3f::Zero(),Eigen::Quaternionf::Identity(),Eigen::Vector3f(1.f,1.f,1.f));
	for (uint32_t j=0;j<m_IKController->m_iksFABRIK.size();++j) {
		std::vector<Vector3f> fbrkPoints = m_IKController->m_iksFABRIK[j].fbrkPoints;
		for (uint32_t i = 0; i < fbrkPoints.size(); ++i) {
			Eigen::Matrix4f cubeTransform = CForgeMath::translationMatrix(fbrkPoints[i]);
			float r = .5;
			Eigen::Matrix4f cubeScale = CForgeMath::scaleMatrix(Eigen::Vector3f(r,r,r)*1.0f);
			m_RenderDev.modelUBO()->modelMatrix(cubeTransform*cubeScale);
			bool cmr = (i+1) & 1;
			bool cmg = (i+1) & 2;
			bool cmb = (i+1) & 4;
			
			//TODO(skade) reads drive due to shader init
			T3DMesh<float> M;
			PrimitiveShapeFactory::cuboid(&M, Vector3f(0.05f, 0.05f, 0.05f), Vector3i(1, 1, 1));
			auto* pMat = M.getMaterial(0);
			pMat->Color = Vector4f(cmr,cmg,cmb, 1.0f);
			pMat->Metallic = 0.3f;
			pMat->Roughness = 0.2f;
			
			M.computePerVertexNormals();
			StaticActor a; a.init(&M);
			
			//glColorMask(cmr,cmg,cmb,1);
			a.render(&m_RenderDev,Eigen::Quaternionf::Identity(),Eigen::Vector3f(),Eigen::Vector3f(1.f,1.f,1.f));
			//glColorMask(1,1,1,1);
		}
	}
	//glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
}

bool MotionRetargetingScene::keyboardAnyKeyPressed() {
	Mouse* pMouse = m_RenderWin.mouse();
	bool r = false;
	for (uint32_t i=0;i<Keyboard::KEY_COUNT;++i)
		r |= m_RenderWin.keyboard()->keyPressed(Keyboard::Key(i));
	for (uint32_t i=0;i<Mouse::BTN_COUNT;++i)
		r |= pMouse->buttonState(Mouse::Button(i));

	static Vector2f prevScroll = Vector2f::Zero();
	Vector2f scrollDelta = pMouse->wheel()-prevScroll;
	prevScroll = pMouse->wheel();
	if (scrollDelta.x() != 0. || scrollDelta.y() != 0.)
		r = true;
	
	return r;
}

}//CForge
