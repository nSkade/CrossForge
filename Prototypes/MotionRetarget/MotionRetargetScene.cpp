#include "MotionRetargetScene.hpp"

#include "IKSolver/IKChain.hpp"

#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>
#include <Prototypes/Assets/GLTFIO/GLTFIO.hpp>

#include <fstream>

namespace CForge {

void MotionRetargetScene::init() {
	initWindowAndRenderDevice();
	gladLoadGL();
	initCameraAndLights();

	// build scene graph
	m_sgnRoot.init(nullptr);
	m_SG.init(&m_sgnRoot);

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

	initUI();
	
	//TODOf(skade) better clear color
	//	glClearColor(.3f,.3f,.3f,.0f);
	m_RenderDev.m_clearColor[0] = .2 * 10;
	m_RenderDev.m_clearColor[1] = .2 * 10;
	m_RenderDev.m_clearColor[2] = .2 * 10;
	m_RenderWin.position(0,31);
	m_RenderWin.size(1920,1009);

	m_config.baseLoad();
	m_config.load(&m_Cam);
	m_config.load("m_editCam.m_CamIsProj", &m_editCam.m_CamIsProj);
	m_config.load(&m_RenderWin);
	m_editCam.setCamProj(&m_Cam,&m_RenderWin);
}//initialize

void MotionRetargetScene::clear() {
	m_config.store(m_Cam);
	m_config.store("m_editCam.m_CamIsProj", m_editCam.m_CamIsProj);
	m_config.baseStore();

	ExampleSceneBase::clear();

	//TODO(skade)
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
	//TODO(skade)
	cleanUI();
}

void MotionRetargetScene::initCameraAndLights(bool CastShadows) {
	// initialize camera
	m_Cam.init(Vector3f(0.0f, 3.0f, 8.0f), Vector3f::UnitY());
	m_Cam.projectionMatrix(m_WinWidth, m_WinHeight, CForgeMath::degToRad(45.0f), 0.1f, 1000.0f);

	// initialize sun (key light) and back ground light (fill light)
	Vector3f SunDir = Vector3f(-5.0f, 15.0f, 35.0f);
	Vector3f SunPos = Vector3f(-5.0f, 15.0f, 35.0f);
	m_Sun.init(SunPos, -SunDir.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 5.0f);

	//TODOf(skade) depth test for shadow map sometimes wrong because gl clear color affects gPosition
	if(CastShadows)
		m_Sun.initShadowCasting(2048, 2048, Vector2i(2, 2), 0.1f, 1000.0f);

	Vector3f BGLightPos = Vector3f(0.0f, 5.0f, -30.0f);
	m_BGLight.init(BGLightPos, -BGLightPos.normalized(), Vector3f(1.0f, 1.0f, 1.0f), 1.5f, Vector3f(0.0f, 0.0f, 0.0f));

	// set camera and lights
	m_RenderDev.activeCamera(&m_Cam);
	m_RenderDev.addLight(&m_Sun);
	m_RenderDev.addLight(&m_BGLight);
}//initCameraAndLights


void MotionRetargetScene::mainLoop() {

	updateFPS(); //TODOf(skade) improve deltaTime

	{ // passive rendering, only render on update to save energy
		//TODOf(skade) member
		static uint32_t frameActionIdx = 0;
		static bool frameAction = true;
		frameAction = keyboardAnyKeyPressed() || m_IKCupdate || m_animAutoplay
					  || ImGui::IsAnyItemHovered()
					  || ImGuizmo::IsUsing() || m_guizmoViewManipChanged;  //TODO(skade) guizmo viewManipulate not correct

		// View Manipulate from imguizmo
		frameAction |= m_viewManipulate.isInside(m_RenderWin.mouse()->position());

		frameActionIdx = frameAction ? 0 : frameActionIdx+1;
		if (frameActionIdx < 3) { // render 1 frame after action to update ui
			m_RenderWin.update();
		} else {
			m_RenderWin.updateWait();
			return;
		}
	}

	m_SG.update(60.0f / m_FPS);

	if (m_IKCupdate || m_IKCupdateSingle) {
		m_IKControllerPrim->update(60.0f / m_FPS);
		m_IKCupdateSingle = false;
	}

	if (m_pAnimCurr) {
		//m_pAnimCurr->Speed = 1./60.; //TODO(skade)
		//m_pAnimCurr->Duration = 2000.; //TODO(skade) unused when applied?

		if (m_animAutoplay) {
			m_animFrameCurr = m_pAnimCurr->t*m_pAnimCurr->SamplesPerSecond;
			m_pAnimCurr->t += 1./m_FPS * m_pAnimCurr->Speed;
			if (m_pAnimCurr->t > m_pAnimCurr->Duration) //TODO(skade) duration sometimes not max
				m_pAnimCurr->t = 0.;
		} else
			m_pAnimCurr->t = m_animFrameCurr / m_pAnimCurr->SamplesPerSecond;
	}
	
	// Handle Picking and camera
	if (!ImGui::IsAnyItemHovered() && !ImGui::IsAnyItemActive()) {
		//TODO(skade) docking fullscreen viewport sets this to true // && !ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) {
		if (!m_RenderWin.mouse()->buttonState(Mouse::BTN_LEFT) && m_LMBDownLastFrame) {

			if (!ImGuizmo::IsUsing()) {
				std::vector<std::weak_ptr<IPickable>> p;

				//TODO(skade) ikc prim to other func
				if (m_IKControllerPrim) {
					if (m_showTarget) {
						std::vector<std::weak_ptr<IKTarget>> t = m_IKControllerPrim->getTargets();
						p.assign(t.begin(),t.end());
						m_picker.pick(p);
					}
					if (m_showJoints) {
						std::vector<std::weak_ptr<JointPickable>> jp = m_IKControllerPrim->getJointPickables();
						p.assign(jp.begin(),jp.end());
						m_picker.pick(p);
					}
				}
				
				m_guizmoMat = m_picker.m_guizmoMat;
			}
		} else
			m_editCam.defaultCameraUpdate(&m_Cam, &m_RenderWin, 0.05f, .7f, 32.0f);

		if (m_picker.getLastPick())
			m_guizmo.active(true);
		else
			m_guizmo.active(false);
	}
	m_LMBDownLastFrame = m_RenderWin.mouse()->buttonState(Mouse::BTN_LEFT);
	m_picker.update(m_guizmoMat);

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
	if (m_exitCalled)
		m_RenderWin.closeWindow();
}//mainLoop

void MotionRetargetScene::initCharacter() {
	if (!m_MeshCharPrim) {
		SLogger::log("MotionRetargetin: initCharacter: no mesh loaded");
		return;
	}

	setMeshShader(m_MeshCharPrim.get(), 0.7f, 0.04f); //TODO(skade) check not modified export
	m_MeshCharPrim->computePerVertexNormals(); //TODO(skade) remove
	m_IKControllerPrim = std::make_unique<IKController>();
//	m_IKControllerPrim->init(m_MeshCharPrim.get(), "MyAssets/ccd-ik/ces/SkeletonConfig.json");
	if (m_MeshCharPrim.get()->rootBone()) {
		m_IKControllerPrim->init(m_MeshCharPrim.get());

		for (uint32_t i=0;i<m_MeshCharPrim->skeletalAnimationCount();++i)
			m_IKControllerPrim->addAnimationData(m_MeshCharPrim->getSkeletalAnimation(i));
		m_IKActorPrim = std::make_unique<IKSkeletalActor>();
		m_IKActorPrim->init(m_MeshCharPrim.get(), m_IKControllerPrim.get());

		m_sgnCharPrim.init(&m_sgnRoot, m_IKActorPrim.get());
	}
	else {
		m_StaticActorPrim = std::make_unique<StaticActor>();
		m_StaticActorPrim->init(m_MeshCharPrim.get());
		m_sgnCharPrim.init(&m_sgnRoot, m_StaticActorPrim.get());
	}

	// autoscale
	m_MeshCharPrim.get()->computeAxisAlignedBoundingBox();
	Box aabb = m_MeshCharPrim.get()->aabb();
	Vector3f scale = Vector3f(2.,2.,2.)/(aabb.diagonal().maxCoeff()); //TODO(skade) standard size
	m_sgnCharPrim.scale(scale);
}//initActors

void MotionRetargetScene::initCesiumMan() {
	std::string path ="MyAssets/ccd-ik/CesiumMan/glTF/CesiumMan.gltf"; 
	std::ifstream f(path.c_str());
	if (!f.good()) {
		SLogger::log("initCesiumMan: not available");
		return;
	}
	
	m_MeshCharPrim = std::make_unique<T3DMesh<float>>();
	GLTFIO::load(path, m_MeshCharPrim.get());
	initCharacter();
}

void MotionRetargetScene::initEndEffectorMarkers() {
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

//TODO(skade)
void MotionRetargetScene::loadCharPrim(std::string path) {
	if (GLTFIO::accepted(path, I3DMeshIO::Operation::OP_LOAD)) {
		m_MeshCharPrim = std::make_unique<T3DMesh<float>>();
		GLTFIO::load(path,m_MeshCharPrim.get());
		initCharacter();
	}
}
void MotionRetargetScene::storeCharPrim(std::string path) {
	if (GLTFIO::accepted(path, I3DMeshIO::Operation::OP_STORE))
		GLTFIO::store(path,m_MeshCharPrim.get());
}

void MotionRetargetScene::renderVisualizers() {
	if (!m_IKControllerPrim)
		return;

	glClear(GL_DEPTH_BUFFER_BIT);
	if (m_showJoints) { //TODO(skade) put in function
		//glDisable(GL_DEPTH_TEST);
		//glEnable(GL_BLEND);

		//TODO(skade) cleanup
		//m_IKController->renderJointPickables(&m_RenderDev);
		
		auto& joints = m_IKControllerPrim->getJointPickables();
		for (auto j : joints) {
			if (auto jl = j.lock()) {
				Vector3f pos;
				Quaternionf rot;
				Vector3f scale;
				m_sgnCharPrim.buildTansformation(&pos,&rot,&scale);
				
				//TODO(skade)
				//Matrix4f t = Matrix4f::Identity();
				Matrix4f t = CForgeMath::translationMatrix(pos) * CForgeMath::rotationMatrix(rot) * CForgeMath::scaleMatrix(scale);
				
				jl->update(t);
				jl->render(&m_RenderDev);
			}
		}

		//glDisable(GL_BLEND);
		//glEnable(GL_DEPTH_TEST);
	}
	glDisable(GL_DEPTH_TEST);
	//glEnable(GL_BLEND); //TODOf(skade) change blendmode in active material forward pass
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	if (m_showTarget) {
		std::vector<std::shared_ptr<IKTarget>> t;
		m_IKControllerPrim->getTargets(&t);
		
		for (uint32_t i = 0; i < t.size(); ++i) {
			//Box aabb = t[i]->bv.aabb();
			m_RenderDev.modelUBO()->modelMatrix(t[i]->pckTransPickin());
			m_TargetPos.render(&m_RenderDev,Quaternionf(),Vector3f(),Vector3f());
		}
	}

	//TODO(skade)
	// render fabrik points
	for (uint32_t j=0;j<m_IKControllerPrim->m_iksFABRIK.size();++j) {
		std::vector<Vector3f> fbrkPoints = m_IKControllerPrim->m_iksFABRIK[j].fbrkPoints;
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

void MotionRetargetScene::defaultKeyboardUpdate(Keyboard* pKeyboard) {
	if (!pKeyboard)
		throw NullpointerExcept("pKeyboard");

	if (pKeyboard->keyPressed(Keyboard::KEY_F1, true))
		m_DrawHelpTexts = !m_DrawHelpTexts;

	if (pKeyboard->keyPressed(Keyboard::KEY_F9, true))
		m_RenderWin.vsync(!m_RenderWin.vsync());
	if (m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_F10, true)) {
		m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
		std::string ScreenshotURI = "Screenshots/Screenshot_" + std::to_string(m_ScreenshotCount++) + "." + m_ScreenshotExtension;
		takeScreenshot(ScreenshotURI);
	}

	if (pKeyboard->keyPressed(Keyboard::KEY_F11, true)) {
		m_RenderWin.toggleFullscreen();
		m_RenderWin.vsync(true);
	}

	if (pKeyboard->keyPressed(Keyboard::KEY_ESCAPE))
		m_exitCalled = true;
}

bool MotionRetargetScene::keyboardAnyKeyPressed() {
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
