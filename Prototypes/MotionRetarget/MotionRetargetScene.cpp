#include "MotionRetargetScene.hpp"

#include "IK/IKChain.hpp"
#include "CMN/MRMutil.hpp"

#include <crossforge/MeshProcessing/PrimitiveShapeFactory.h>
#include <Prototypes/Assets/GLTFIO/GLTFIO.hpp>
#include <crossforge/AssetIO/SAssetIO.h>

#include <Prototypes/objImport/objImport.h>

#include <fstream>
#include <filesystem>

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
	initIKTargetActor(); // initialize end-effector & target markers
	
	//TODOfff(skade)
	//LineOfText* pKeybindings = new LineOfText();
	//pKeybindings->init(CForgeUtility::defaultFont(CForgeUtility::FONTTYPE_SANSERIF, 18), "Movement:(Shift) + W,A,S,D  | Rotation: LMB/RMB + Mouse | F1: Toggle help text");
	//m_HelpTexts.push_back(pKeybindings);
	//pKeybindings->color(0.0f, 0.0f, 0.0f, 1.0f);
	//m_DrawHelpTexts = false;

	// check whether a GL error occurred
	std::string GLError = "";
	CForgeUtility::checkGLError(&GLError);
	if (!GLError.empty()) printf("GLError occurred: %s\n", GLError.c_str());

	initUI();
	
	//TODOfff(skade) better clear color impl
	m_RenderDev.m_clearColor[0] = .1 * 10;
	m_RenderDev.m_clearColor[1] = .1 * 10;
	m_RenderDev.m_clearColor[2] = .1 * 10;
	m_RenderWin.position(0,31);
	m_RenderWin.size(1920,1009);

	m_config.baseLoad();
	m_config.load(&m_Cam);
	m_config.load(&m_RenderWin);

	m_config.load("m_editCam.m_CamIsProj", &m_editCam.m_CamIsProj);
	m_config.load(m_cesStartupStr.c_str(), &m_settings.cesStartup);

	m_config.load("path.anaconda", &m_settings.pathAnaconda);
	m_config.load("path.rignet", &m_settings.pathRignet);

	if (m_settings.cesStartup)
		initCesiumMan();

	m_editCam.setCamProj(&m_Cam,&m_RenderWin);
	m_lineBox.init();
	m_editGrid.init();
}//initialize

void MotionRetargetScene::clear() {
	m_config.store(m_Cam);
	m_config.store("m_editCam.m_CamIsProj", m_editCam.m_CamIsProj);

	m_config.baseStore();

	ExampleSceneBase::clear();
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

	//TODOf(skade) make shadow light toggable in preferences
	//TODOfff(skade) depth test for shadow map sometimes wrong because gl clear color affects gPosition
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
	//TODOfff(skade) split brackets into function

	updateFPS(); //TODOfff(skade) improve deltaTime

	{ // passive rendering, only render on update to save energy
		//TODOfff(skade) member
		static uint32_t frameActionIdx = 0;
		static bool frameAction = true;
		bool IKCupdate = false, animAutoplay = false;
		for (auto& c : m_charEntities) {
			IKCupdate |= c->m_IKCupdate;
			animAutoplay |= c->m_animAutoplay;
		}

		frameAction = keyboardAnyKeyPressed() || IKCupdate || animAutoplay
					  || ImGui::IsAnyItemHovered()
					  || ImGuizmo::IsUsing() || m_guizmoViewManipChanged;
		// need to render on window resize
		static Vector2i winRes;
		Vector2i winResNew;
		winResNew.x() = m_RenderWin.width();
		winResNew.y() = m_RenderWin.height();
		if (winRes != winResNew)
			frameAction = true;
		winRes = winResNew;

		// View Manipulate from imguizmo
		frameAction |= m_viewManipulate.isInside(m_RenderWin.mouse()->position());

		frameActionIdx = frameAction ? 0 : frameActionIdx+1;
		if (frameActionIdx < 4) { // render 1 frame after action to update ui
			m_RenderWin.update();
		} else {
			m_RenderWin.updateWait();
			return;
		}
	}

	for (auto ce : m_charEntities) {
		if (ce->controller)
			ce->controller->forwardKinematics();
	}
	m_MRlimb.update();
	m_SG.update(60.0f / m_FPS);
	{ // animation update
		for (uint32_t i=0;i<m_charEntities.size();++i) {
			auto c = m_charEntities[i];
			if (c->m_IKCupdate || c->m_IKCupdateSingle) {
				c->controller->update(60.0f / m_FPS);
				c->m_IKCupdateSingle = false;
			}
			if (!c->pAnimCurr)
				continue;
			//m_pAnimCurr->Speed = 1./60.; //TODOf(skade)
			//m_pAnimCurr->Duration = 2000.; //TODOf(skade) unused when applied?

			//TODOf(skade) move into char entity
			auto* pA = c->pAnimCurr;
			if (c->m_animAutoplay) {
				c->animFrameCurr = pA->t * pA->SamplesPerSecond;
				pA->t += 1./m_FPS * pA->Speed;
				if (pA->t > pA->Duration) //TODOf(skade) duration sometimes not max
					pA->t = 0.;
			} else
				pA->t = c->animFrameCurr / pA->SamplesPerSecond; //TODO(skade) make pose configurable
		}
	}

	{ // edit mode logic
		// hide all char entities
		static bool prevIsEditMode = false;
		if (m_isEditMode != prevIsEditMode) {
			for (auto c : m_charEntities)
				c->visibility = float(!m_isEditMode);
		}prevIsEditMode = m_isEditMode;

		if (m_isEditMode) {
			static std::shared_ptr<CharEntity> prevC;
			std::shared_ptr<CharEntity> currC = std::dynamic_pointer_cast<CharEntity>(m_picker.getCurrPick().lock());
			if (prevC.get() != currC.get()) {
				// restore old char Entity transform
				if (auto pc = prevC) {
					pc->sgn.position(m_editModeCachePos);
					pc->sgn.scale(m_editModeCacheScale);
					pc->sgn.rotation(m_editModeCacheRot);
					m_editModeCachePos = Vector3f::Zero();
					m_editModeCacheScale = Vector3f::Ones();
					m_editModeCacheRot = Quaternionf::Identity();
					pc->visibility = 0.;
					if (!currC)
						m_picker.reset();
				}
				// set cache to new charEntity
				if (auto c = currC) {
					m_editModeCachePos = c->sgn.position();
					m_editModeCacheScale = c->sgn.scale();
					m_editModeCacheRot = c->sgn.rotation();
					c->sgn.position(Vector3f::Zero());
					c->sgn.scale(Vector3f::Ones());
					c->sgn.rotation(Quaternionf::Identity());
					//m_picker.update(MRMutil::buildTransformation(c->sgn));
					//m_guizmoMat = m_picker.m_guizmoMat;
					forcePickCharEntity(c);
					c->visibility = 1.;
				}
				prevC = currC;
			}
		}
	}

	// handle visibility
	{
		for (auto c : m_charEntities) {
			IRenderableActor* actor;
			actor = c->actor.get();
			if (!actor)
				actor = c->actorStatic.get();
			if (auto a = actor) {
				for (int i = 0; i < a->materialCount(); ++i) {
 //TODO(skade) assumes that amount of renderable material and material is the same might break
					auto col = c->mesh.getMaterial(i)->Color;
					col.w() *= c->visibility;
					a->material(i)->color(col);
				}
			}
			
			//TODOfff(skade) not every frame
			//bool oldVis = c->visible;
			//TODO(skade) dont detach from rendering because of anim controller update?
			//if (oldVis != c->visible) {
			//if (c->visible)
			//	m_sgnRoot.addChild(&c->sgn);
			//else
			//	m_sgnRoot.removeChild(&c->sgn);
			//}
		}
	}

	bool hoveredImgui = ImGui::IsAnyItemHovered() || ImGui::IsAnyItemActive() || ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);

	// removing objects
	if (!hoveredImgui && m_RenderWin.keyboard()->keyPressed(Keyboard::KEY_DELETE,true)) {
		if (auto p = std::dynamic_pointer_cast<CharEntity>(m_picker.getLastPick().lock())) {
			m_sgnRoot.removeChild(&p->sgn);

			auto pos = std::find(m_charEntities.begin(),m_charEntities.end(),p);
			if (pos != m_charEntities.end())
				m_charEntities.erase(pos);

			m_picker.reset();
		}
		else if (auto p = std::dynamic_pointer_cast<IKTarget>(m_picker.getLastPick().lock())) {
			//TODOff(skade) IKTargets better global? for delete but also when letting the target change by another char entity

			for (auto c : m_charEntities) {
				auto& ts = c->controller->m_targets;
				auto pos = std::find(ts.begin(),ts.end(),p);
				if (pos != ts.end())
					ts.erase(pos);
			}
			m_picker.reset();
		}
		// reset material in render device to avoid updating it
		m_RenderDev.activeMaterial(nullptr);
	} // removing objects
	
	{ // Handle Picking
		if (!hoveredImgui) {
			if (!m_RenderWin.mouse()->buttonState(Mouse::BTN_LEFT) && m_LMBDownLastFrame) {

				if (!ImGuizmo::IsUsing()) {
					std::vector<std::weak_ptr<IPickable>> p;
					m_picker.start();
					for (uint32_t i=0;i<m_charEntities.size();++i) {
						auto c = m_charEntities[i];
						//if (!c->visible)
						//	continue;

						p.clear(); p.push_back(c);
						m_picker.pick(p);
						if (!c->controller)
							continue;
						if (m_settings.showJoints) {
							std::vector<std::weak_ptr<JointPickable>> jp = c->controller->getJointPickables();
							p.assign(jp.begin(),jp.end());
							m_picker.pick(p);
						}
						if (m_settings.showTargets) {
							std::vector<std::shared_ptr<IKTarget>> t = c->controller->m_targets;
							p.assign(t.begin(),t.end());
							m_picker.pick(p);
						}
					}

					m_picker.resolve();
					m_guizmoMat = m_picker.m_guizmoMat;
					if (auto e = std::dynamic_pointer_cast<CharEntity>(m_picker.getLastPick().lock())) {
						m_charEntitySec = m_charEntityPrim;
						m_charEntityPrim = e;
					}
				}
			}

			if (auto lp = m_picker.getLastPick().lock())
				m_guizmo.active(true);
			else
				m_guizmo.active(false);
		}
		m_LMBDownLastFrame = m_RenderWin.mouse()->buttonState(Mouse::BTN_LEFT);
		m_picker.update(m_guizmoMat);
	} // Handle Picking

	{ // View and Rendering
		m_editCam.defaultCameraUpdate(&m_Cam, &m_RenderWin, !hoveredImgui, 0.05f, .7f, 32.0f);
		m_viewManipulate.update(m_RenderWin.mouse()->position(),m_RenderWin.mouse()->buttonState(Mouse::BTN_LEFT),!ImGui::IsDragDropActive());

		// Render Scene
		m_RenderDev.activeMaterial(nullptr);
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
	}
	
	m_config.store(m_RenderWin); // need to store every frame because, clear has m_RenderWin already destructed
	defaultKeyboardUpdate(m_RenderWin.keyboard());
	if (m_exitCalled)
		m_RenderWin.closeWindow();
}//mainLoop

void MotionRetargetScene::initCharacter(std::weak_ptr<CharEntity> charEntity) {
	std::shared_ptr<CharEntity> c = charEntity.lock();
	//setMeshShader(mesh, 0.7f, 0.04f); //TODOff(skade) check not modified export

	c->init(&m_sgnRoot);

	//TODOff(skade) autoscale import option in preferences
	// autoscale
	Vector3f scale = Vector3f(2.f,2.f,2.f)/(c->bv.aabb().diagonal().maxCoeff()); //TODOff(skade) standard size
	c->sgn.scale(scale);
}

//TODOfff(skade) better pos
void MotionRetargetScene::forcePickCharEntity(std::weak_ptr<CharEntity> c) {
	m_picker.forcePick(c);
	m_picker.m_guizmoMat = MRMutil::buildTransformation(c.lock()->sgn); // this shouldnt be necessary
	m_guizmoMat = m_picker.m_guizmoMat;
	m_charEntitySec = m_charEntityPrim;
	m_charEntityPrim = c;
}

void MotionRetargetScene::initCesiumMan() {

	auto isPathGood = [](std::string path) {
		std::ifstream f(path.c_str());
		if (!f.good()) {
			SLogger::log("path not good D:");
			return false;
		}
		return true;
	};

	//TODOff(skade) config
	std::string p1 ="MyAssets/ccd-ik/Longbow Locomotion Pack/MRtest.fbx";
	std::string p1skl ="MyAssets/ccd-ik/Longbow Locomotion Pack/sklc.json";
	AngleAxisf r1 = AngleAxisf(CForgeMath::degToRad(90.),Vector3f(0.,1.,0.));
	IOmeth l1 = IOmeth::IOM_ASSIMP;

	std::string p2 ="MyAssets/ccd-ik/CesiumMan/glTF/CesiumMan.gltf";
	std::string p2skl ="MyAssets/ccd-ik/ces0/SkeletonConfig.json";
	IOmeth l2 = IOmeth::IOM_GLTFIO;
	AngleAxisf r2 = AngleAxisf(CForgeMath::degToRad(-90.),Vector3f(1.,0.,0.));
	//r2 = AngleAxisf(CForgeMath::degToRad(-90.),Vector3f(0.,1.,0.)) * r2;

	if (!isPathGood(p1)
		|| !isPathGood(p1skl)
		|| !isPathGood(p2)
		|| !isPathGood(p2skl)
		)
		return;

	loadCharPrim(p1,l1);
	std::shared_ptr<CharEntity> c1 = m_charEntities.back();
	c1->sgn.rotation(Quaternionf(r1));
	//c1->sgn.scale(c1->sgn.scale()*.8); //TODO(skade)
	c1->sgn.scale(c1->sgn.scale()*.8); //TODO(skade)
	c1->applyTransformToMesh(&m_sgnRoot);
	c1->importArmature(p1skl);
	c1->parseArmature();
	c1->sgn.position({0.,0.,1.});
	c1->controller->forwardKinematics();
	c1->controller->initTargetPoints();

	loadCharPrim(p2,l2);
	std::shared_ptr<CharEntity> c2 = m_charEntities.back();
	c2->sgn.rotation(Quaternionf(r2));
	c2->applyTransformToMesh(&m_sgnRoot);
	c2->importArmature(p2skl);
	c2->parseArmature();

	c2->sgn.position({0.,0.,-1.});
	
c2->controller->forwardKinematics();
	c2->controller->initTargetPoints();
}

void MotionRetargetScene::initIKTargetActor() {
	T3DMesh<float> M;

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
	for (uint32_t i = 0; i < M.materialCount(); ++i) {
		auto* pMat = M.getMaterial(i);
		pMat->Color = Vector4f(1.0f, 1.0f, 0.0f, 1.0f);
		pMat->Metallic = 0.3f;
		pMat->Roughness = 0.2f;
	}
	m_TargetPosForeign.init(&M);
}//initIKTargetActor

void MotionRetargetScene::loadCharPrim(std::string path, IOmeth ioM) {
	switch (ioM)
	{
	case CForge::MotionRetargetScene::IOM_ASSIMP:
		if (SAssetIO::accepted(path, I3DMeshIO::Operation::OP_LOAD)) {
			std::shared_ptr<CharEntity> c = std::make_shared<CharEntity>();
			std::string name = std::filesystem::path(path).filename().string();
			c->name = name;
			//TODOfff(skade) make m_charEntities std::map to avoid name confilcts?
			// for now avoid name conflicts via checking
			bool nameUnique = false;
			int nameNum = 0;
			while (!nameUnique) {
				nameUnique = true;
				for (auto oc : m_charEntities) {
					if (oc->name == c->name) {
						c->name = name + std::to_string(nameNum);
						nameUnique = false; // need to check again
					}
				}
				nameNum++;
			}
			SAssetIO::load(path,&c->mesh);
			initCharacter(c);
			m_charEntities.emplace_back(std::move(c));
		}
		break;
	case CForge::MotionRetargetScene::IOM_GLTFIO:
		if (GLTFIO::accepted(path, I3DMeshIO::Operation::OP_LOAD)) {
			std::shared_ptr<CharEntity> c = std::make_shared<CharEntity>();
			std::string name = std::filesystem::path(path).filename().string();
			c->name = name;
			//TODOfff(skade) make m_charEntities std::map to avoid name confilcts?
			// for now avoid name conflicts via checking
			bool nameUnique = false;
			int nameNum = 0;
			while (!nameUnique) {
				nameUnique = true;
				for (auto oc : m_charEntities) {
					if (oc->name == c->name) {
						c->name = name + std::to_string(nameNum);
						nameUnique = false; // need to check again
					}
				}
				nameNum++;
			}
			GLTFIO::load(path,&c->mesh);
			initCharacter(c);
			m_charEntities.emplace_back(std::move(c));
		}
		break;
	case CForge::MotionRetargetScene::IOM_OBJIMP:
		break;
	default:
		break;
	}
}
void MotionRetargetScene::storeCharPrim(std::string path, IOmeth ioM) {
	auto c = m_charEntityPrim.lock();
	if (!c)
		return;

	switch (ioM)
	{
	case CForge::MotionRetargetScene::IOM_ASSIMP:
		//if (SAssetIO::accepted(path, I3DMeshIO::Operation::OP_STORE)) //TODO(skade)
			SAssetIO::store(path,&c.get()->mesh);
		break;
	case CForge::MotionRetargetScene::IOM_GLTFIO:
		//if (GLTFIO::accepted(path, I3DMeshIO::Operation::OP_STORE)) //TODO(skade)
			GLTFIO::store(path,&c.get()->mesh);
		break;
	case CForge::MotionRetargetScene::IOM_OBJIMP:
		objImportExport::exportAsObjFile(path,&c->mesh);
		break;
	default:
		break;
	}
}
void MotionRetargetScene::renderVisualizers() {
	if (m_settings.renderAABB) {
		if (auto c = std::dynamic_pointer_cast<CharEntity>(m_picker.getLastPick().lock())) {
			Matrix4f m = MRMutil::buildTransformation(c->sgn);
			m_lineBox.color = Vector4f(227./255,142./255,48./255,.75);
			m_lineBox.render(&m_RenderDev,c->bv.aabb(),m);
		}
		if (auto c = std::dynamic_pointer_cast<CharEntity>(m_charEntitySec.lock())) {
			Matrix4f m = MRMutil::buildTransformation(c->sgn);
			m_lineBox.color = Vector4f(227./255,142./255,48./255,.25);
			m_lineBox.render(&m_RenderDev,c->bv.aabb(),m);
		}
	}

	//TODOf(skade) rotate grid when ortographic view depending on largest cam view vector component
		//if (!m_editCam.m_CamIsProj)
	if (m_settings.renderDebugGrid)
		m_editGrid.render(&m_RenderDev,m_settings.gridSize);

	for (uint32_t i=0;i<m_charEntities.size();++i) {
		//if (m_charEntities[i]->visible)
			renderVisualizers(m_charEntities[i].get());
	}
}

void MotionRetargetScene::renderVisualizers(CharEntity* c) {
	Vector3f pos; Quaternionf rot; Vector3f scale;
	c->sgn.buildTansformation(&pos,&rot,&scale);
	Matrix4f t = CForgeMath::translationMatrix(pos) * CForgeMath::rotationMatrix(rot) * CForgeMath::scaleMatrix(scale);
	glClear(GL_DEPTH_BUFFER_BIT);
	if (c->controller && m_settings.showJoints) { //TODOff(skade) put in function
		auto& joints = c->controller->getJointPickables();
		if (joints.size() > 0 && joints[0].lock()->getOpacity() != 0.f) {
			for (auto j : joints) {
				if (auto jl = j.lock()) {
					//TODOf(skade) option to seperate skeleton with translation for visualization
					
					jl->update(t);
					jl->render(&m_RenderDev);
				}
			}
		}
	}
	glDisable(GL_DEPTH_TEST);

	if (c->controller && m_settings.showTargets) {
		std::vector<std::shared_ptr<IKTarget>>& tar = c->controller->m_targets;
		
		for (uint32_t i = 0; i < tar.size(); ++i) {
			//TODOff(skade) clean sgnT update loc
			tar[i]->update(t);
			
			//Box aabb = t[i]->bv.aabb();
			m_RenderDev.modelUBO()->modelMatrix(tar[i]->pckTransPickin());
			m_TargetPos.render(&m_RenderDev,Quaternionf(),Vector3f(),Vector3f());
		}

		// foreign targets
		auto& ikcs = c->controller->m_ikArmature.m_jointChains;
		//for (auto& ikc : ikcs) {
		//	IKTarget* ikct = ikc.target.lock().get();
		//	bool isForeign = true;
		//	for (auto& st : tar) {
		//		if (ikct && st.get() && st.get() == ikct) {
		//			isForeign = false;
		//			break;
		//		}
		//	}
		//	if (!isForeign)
		//		continue;

		//	IKTarget nikct = *ikct;
		//	nikct.update(t);
		//	
		//	m_RenderDev.modelUBO()->modelMatrix(nikct.pckTransPickin());
		//	m_TargetPosForeign.render(&m_RenderDev,Quaternionf(),Vector3f(),Vector3f());
		//}
		for (auto& t : m_MRlimb.m_targets) {
			m_RenderDev.modelUBO()->modelMatrix(t->pckTransPickin());
			m_TargetPosForeign.render(&m_RenderDev,Quaternionf(),Vector3f(),Vector3f());
		}
	}

	//TODOf(skade) make toggable
	// render fabrik points
	if (c->controller) {
		auto fbrkChains = c->controller->getFABRIKpoints();
		for (auto fbrkPoints : fbrkChains) {
			for (uint32_t i = 0; i < fbrkPoints.size(); ++i) {
				Matrix4f cubeTransform = CForgeMath::translationMatrix(fbrkPoints[i]);
				float r = .5;
				Matrix4f cubeScale = CForgeMath::scaleMatrix(Vector3f(r,r,r)*1.0f);
				m_RenderDev.modelUBO()->modelMatrix(t*cubeTransform*cubeScale);
				bool cmr = (i+1) & 1;
				bool cmg = (i+1) & 2;
				bool cmb = (i+1) & 4;
				
				//TODOff(skade) reads drive due to shader init
				T3DMesh<float> M;
				PrimitiveShapeFactory::cuboid(&M, Vector3f(0.05f, 0.05f, 0.05f), Vector3i(1, 1, 1));
				auto* pMat = M.getMaterial(0);
				pMat->Color = Vector4f(cmr,cmg,cmb, 1.0f);
				pMat->Metallic = 0.3f;
				pMat->Roughness = 0.2f;
				
				M.computePerVertexNormals();
				StaticActor a; a.init(&M);
				
				//glColorMask(cmr,cmg,cmb,1);
				a.render(&m_RenderDev,Quaternionf::Identity(),Vector3f(),Vector3f(1.f,1.f,1.f));
				//glColorMask(1,1,1,1);
			}
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

	if (pKeyboard->keyPressed(Keyboard::KEY_ESCAPE,true)) {
		// check if popup is open and close instead of exit
		for (uint32_t i=0;i<POP_COUNT;++i) {
			if (m_showPop[i]) {
				m_showPop[i] = false;
				return;
			}
		}

		m_exitCalled = true;
	}
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
