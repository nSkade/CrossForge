#include "MotionRetargetScene.hpp"

#include <Prototypes/GUI/ImGuiUtility.h>
#include "UI/ImGuiStyle.hpp"
#include "Animation/IKSequencer.hpp"
#include <crossforge/AssetIO/UserDialog.h>
//TODO(skade) for ImGuiUtility::initImGui replace
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
//TODO(skade)

#include <imgui_stdlib.h>

namespace ImGui {

auto ComboStr = [](const char* label, int* current_item, const std::vector<std::string>& items, int items_count, int height_in_items = -1)
{
	return ImGui::Combo(label, current_item, [](void* data, int idx, const char** out_text) { *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str(); return true; }, (void*)&items, items_count, height_in_items);
};

}//ImGui

namespace CForge {

void MotionRetargetScene::initUI() {
	//TODO(skade) copied out of for now, order important?
	//ImGuiUtility::initImGui(&m_RenderWin);
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	ImGuiIO& io = ImGui::GetIO();

	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
	//io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows

	// When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
	//ImGuiStyle& style = ImGui::GetStyle();
	//if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
	//{
	//	style.WindowRounding = 0.0f;
	//	style.Colors[ImGuiCol_WindowBg].w = 1.0f;
	//}

	SetupImGuiStyle(true,0.5f);

	io.Fonts->AddFontDefault();

	ImGui_ImplGlfw_InitForOpenGL(static_cast<::GLFWwindow*>(m_RenderWin.handle()), true);
	ImGui_ImplOpenGL3_Init("#version 330 core");
	//TODO(skade) end
}
void MotionRetargetScene::cleanUI() {
	ImGuiUtility::shutdownImGui();
}

void MotionRetargetScene::renderUI() {
	ImGuiUtility::newFrame();
	ImGuizmo::BeginFrame();
	ImGuiViewport* igViewPort = ImGui::GetMainViewport();
	ImGui::DockSpaceOverViewport(igViewPort,ImGuiDockNodeFlags_PassthruCentralNode);

	renderUI_menuBar();
	renderUI_Outliner();
	renderUI_animation();
	renderUI_Sequencer();
	renderUI_tools();
	renderUI_ik();

	ImGuiUtility::render();

	//TODOf(skade) move imgui outside glfw viewport, windows only
	//ImGuiIO& io = ImGui::GetIO();
	//if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
	//{
	//	::GLFWwindow* backup_current_context = glfwGetCurrentContext();
	//	ImGui::UpdatePlatformWindows();
	//	ImGui::RenderPlatformWindowsDefault();
	//	glfwMakeContextCurrent(backup_current_context);
	//}
}//renderUI

IKController::SkeletalJoint* MotionRetargetScene::renderUI_OutlinerJoints(std::shared_ptr<CharEntity> c, IKController::SkeletalJoint* selectedJoint) {

	std::function<void(CharEntity* c, IKController::SkeletalJoint* joint)> renderJointNode;
	IKController::SkeletalJoint* clickedNode = nullptr;

	renderJointNode = [&](CharEntity* c, IKController::SkeletalJoint* joint) {
		if (!joint)
			return;
	
		static ImGuiTreeNodeFlags base_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanAvailWidth;
		ImGuiTreeNodeFlags node_flags = base_flags;
		if (joint == selectedJoint)
			node_flags |= ImGuiTreeNodeFlags_Selected;
		if (joint->Children.size() > 0) { // tree
			bool node_open = ImGui::TreeNodeEx((void*)(intptr_t)joint, node_flags,joint->Name.c_str());
			if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
				clickedNode = joint;
			if (node_open) {
				for (uint32_t i=0;i<joint->Children.size();++i)
					renderJointNode(c,c->controller->getBone(joint->Children[i]));
				ImGui::TreePop();
			}
		}
		else { // leaf
			node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen; // ImGuiTreeNodeFlags_Bullet
			//ImGui::TreeNodeEx((void*)(intptr_t)i, node_flags, "Selectable Leaf %d", i);
			ImGui::TreeNodeEx((void*) (intptr_t)joint,node_flags,joint->Name.c_str());
			if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
				clickedNode = joint;
		}
	};
	renderJointNode(c.get(),c->controller->getRoot());
	return clickedNode;
}

//TODO(skade) cleanup
void MotionRetargetScene::renderUI_Outliner() {

	ImGui::Begin("Outliner");
	{ // currently selected item
		std::string item = "";
		if (auto lp = m_picker.getLastPick().lock()) {
			if (auto p = std::dynamic_pointer_cast<CharEntity>(lp))
				item = "CharEntity "+p->name;
			if (auto p = std::dynamic_pointer_cast<JointPickable>(lp))
				item = "Joint "+p->m_pJoint->Name;
			if (auto p = std::dynamic_pointer_cast<IKTarget>(lp))
				item = "Target "+p->name;
		}
		ImGui::Text(item.c_str());
	}
	ImGui::Separator();
	//TODO(skade) make selectable
	for (uint32_t i=0;i<m_charEntities.size();++i) {
		auto c = m_charEntities[i];
		if (!c)
			continue;
		if (ImGui::TreeNode(c->name.c_str())) {
			if (ImGui::CollapsingHeader("visiblity options")) {
				bool oldVis = c->visible;
				ImGui::Checkbox("visible",&c->visible);
				if (oldVis != c->visible) {
					if (c->visible)
						m_sgnRoot.addChild(&c->sgn);
					else
						m_sgnRoot.removeChild(&c->sgn);
				}
				if (c->controller) {
					ImGui::SameLine();
					auto jps = c->controller->getJointPickables();
					float jpo = jps[0].lock()->getOpacity();
					ImGui::SetNextItemWidth(ImGui::GetWindowWidth()*.5);
					ImGui::DragFloat("Joint Opacity",&jpo,.005,0.,1.);
					for (auto jp : jps)
						jp.lock()->setOpacity(jpo);

					static bool hullHighlight = true;
					ImGui::Checkbox("hull highlight",&hullHighlight);
					for (auto jp : jps)
						jp.lock()->highlightBehind = hullHighlight;
				}
			} // visibility
			if (!c->controller) {
				ImGui::TreePop();
				continue;
			}
			if (ImGui::TreeNode("Armature")) {
				IKController::SkeletalJoint* clickedNode = renderUI_OutlinerJoints(c,m_outlinerSelJoint);

				if (clickedNode) {
					if (ImGui::GetIO().KeyCtrl) { // CTRL+click to toggle
						if (m_outlinerSelJoint == clickedNode) {
							m_outlinerSelJoint = nullptr;
							m_picker.reset();
						}
						else {
							m_outlinerSelJoint = clickedNode;
							m_picker.forcePick(c->controller->getJointPickable(clickedNode));
							m_guizmoMat = m_picker.m_guizmoMat;
						}
					}
					else {
						m_outlinerSelJoint = clickedNode;
						m_picker.forcePick(c->controller->getJointPickable(clickedNode));
						m_guizmoMat = m_picker.m_guizmoMat;
					}
				}
				ImGui::TreePop();
			} // Armature
			if (ImGui::TreeNode("Targets")) {
				for (uint32_t j = 0; j < c->controller->m_targets.size(); ++j) {
					auto t = c->controller->m_targets[j];
					static ImGuiTreeNodeFlags base_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanAvailWidth;
					ImGuiTreeNodeFlags node_flags = base_flags;
					node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen; // ImGuiTreeNodeFlags_Bullet
					ImGui::TreeNodeEx(t->name.c_str(),node_flags);
					if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
						m_picker.forcePick(t);
						m_guizmoMat = m_picker.m_guizmoMat;
					}
				}
				ImGui::TreePop();
			} // Targets
			ImGui::TreePop();
		} // if main tree node
	} // for charEntities
	
	//TODO(skade) make Modal window for joint matching
	
	ImGui::End();
}

//TODO(skade)
void MotionRetargetScene::renderUI_animation() {
	ImGui::Begin("Animation");
	auto c = m_charEntityPrim.lock();

	if (!c || !c->actor.get()) {
		ImGui::End();
		return;
	}

	std::vector<std::string> items;
	items.push_back("none");
	for (uint32_t i=0;i<c->controller->animationCount();++i)
		items.push_back(c->controller->animation(i)->Name);
	ImGui::ComboStr("select animation",&c->animIdx,items,items.size());

	if (c->animIdx > 0) { // anim selected
		if (c->pAnimCurr && c->animIdx - 1 != c->pAnimCurr->AnimationID) { // Animation changed
			c->controller->destroyAnimation(c->pAnimCurr);
			c->actor->activeAnimation(nullptr);
			c->pAnimCurr = nullptr;
		}
		if (!c->pAnimCurr) { // create animation if not existing
			c->pAnimCurr = c->controller->createAnimation(c->animIdx-1,1.f,0.f);
			c->actor->activeAnimation(c->pAnimCurr);
		}

		T3DMesh<float>::SkeletalAnimation* anim = c->actor->getController()->animation(c->animIdx-1);
		ImGui::Text("Duration: %f",anim->Duration);
		ImGui::Text("SamplesPerSecond: %f",anim->SamplesPerSecond);
		if (!m_animAutoplay) {
			if(ImGui::Button("Play")) {
				m_animAutoplay = true;
			}
		} else {
			if(ImGui::Button("Stop")) {
				m_animAutoplay = false;
			}
			ImGui::SameLine();
			ImGui::DragFloat("animSpeed", &(c->pAnimCurr->Speed), 0.01f);
		}
		ImGui::Text("pAnim->t: %f",c->pAnimCurr->t);
	}
	else {
		c->controller->destroyAnimation(c->pAnimCurr);
		c->actor->activeAnimation(nullptr);
		c->pAnimCurr = nullptr;
	}
	ImGui::End();
}

void MotionRetargetScene::renderUI_Sequencer() {
	ImGui::Begin("Sequencer");

	// sequence with default values
	static MySequence mySequence;
	static bool init = false;
	if (!init) {
		mySequence.mFrameMin = 0;
		mySequence.mFrameMax = 1000;
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 0, 0, 10, false}); //TODO(skade)
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 0, 0, int(pAnim->Duration*pAnim->SamplesPerSecond), false }); //TODO(skade)
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 1, 20, 30, true });
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 3, 12, 60, false });
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 2, 61, 90, false });
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 4, 90, 99, false });
		init = true;
	}

	int animFrameCurr = 0;
	mySequence.myItems.clear();
	if (auto c = m_charEntityPrim.lock()) {
		if (c->pAnimCurr) {
			int animFrameCount = c->pAnimCurr->Duration * c->pAnimCurr->SamplesPerSecond;
			//int animFrameCount = c->controller->animation(c->pAnimCurr->AnimationID)->Keyframes[0]->Positions.size(); //TODO(skade)
			animFrameCount -= 1; // visualizer is inclusive
			mySequence.myItems.push_back(MySequence::MySequenceItem{0,0,animFrameCount,false});
			animFrameCurr = c->animFrameCurr;
		}
	}

	ImGui::PushItemWidth(130);
	ImGui::InputInt("Frame \t\t", &animFrameCurr);
	ImGui::SameLine();
	ImGui::InputInt("Min \t\t", &mySequence.mFrameMin);
	ImGui::SameLine();
	ImGui::InputInt("Max", &mySequence.mFrameMax);
	ImGui::PopItemWidth();

	//TODO(skade) make member
	static int selectedEntry = -1;
	static int firstFrame = 0;
	static bool expanded = true;
	Sequencer(&mySequence, &animFrameCurr, &expanded, &selectedEntry, &firstFrame,
	          ImSequencer::SEQUENCER_EDIT_STARTEND | ImSequencer::SEQUENCER_ADD | ImSequencer::SEQUENCER_DEL |
	          ImSequencer::SEQUENCER_COPYPASTE | ImSequencer::SEQUENCER_CHANGE_FRAME);
	if (auto c = m_charEntityPrim.lock())
		c->animFrameCurr = animFrameCurr;
	
	//TODO(skade)
	// add a UI to edit that particular item
	//if (selectedEntry != -1) {
	//	const MySequence::MySequenceItem &item = mySequence.myItems[selectedEntry];
	//	ImGui::Text("I am a %s, please edit me", SequencerItemTypeNames[item.mType]);
	//	// switch (type) ....
	//}
	ImGui::End();
}

void MotionRetargetScene::renderUI_menuBar() {
	//TODO(skade) implement menu bar
	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("File"))
		{
			//ImGui::MenuItem("(demo menu)", NULL, false, false);
			//if (ImGui::MenuItem("New")) {}
			if (ImGui::BeginMenu("Import")) {
				if (ImGui::MenuItem("GLTF", ".gltf, .glb")) {
					//TODO(skade) name
					std::string path = UserDialog::OpenFile("load primary char", "gltf", "*.gltf *.glb");
					loadCharPrim(path,true);
				}
				if (ImGui::MenuItem("Assimp")) {
					//TODO(skade) list assimp types
					std::string path = UserDialog::OpenFile("load primary char", "assimp");
					loadCharPrim(path,false);
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Export")) {
				if (ImGui::MenuItem("GLTF", ".gltf, .glb")) {
					//TODO(skade) name
					std::string path = UserDialog::SaveFile("store primary char", "gltf", "*.gltf *.glb");
					storeCharPrim(path,true);
				}
				if (ImGui::MenuItem("Assimp")) {
					//TODO(skade) list assimp types
					std::string path = UserDialog::SaveFile("store primary char", "assimp");
					storeCharPrim(path,false);
				}
				ImGui::EndMenu();
			}

//
			
			//if (ImGui::BeginMenu("Open Recent"))
			//{
			//	ImGui::MenuItem("fish_hat.c");
			//	ImGui::MenuItem("fish_hat.inl");
			//	ImGui::MenuItem("fish_hat.h");
			//	if (ImGui::BeginMenu("More.."))
			//	{
			//		ImGui::MenuItem("Hello");
			//		ImGui::MenuItem("Sailor");
			//		if (ImGui::BeginMenu("Recurse.."))
			//		{
			//			//ShowExampleMenuFile();
			//			ImGui::EndMenu();
			//		}
			//		ImGui::EndMenu();
			//	}
			//	ImGui::EndMenu();
			//}
			//if (ImGui::MenuItem("Save", "Ctrl+S")) {}
			//if (ImGui::MenuItem("Save As..")) {}

			//ImGui::Separator();
			////IMGUI_DEMO_MARKER("Examples/Menu/Options");
			//if (ImGui::BeginMenu("Options"))
			//{
			//	static bool enabled = true;
			//	ImGui::MenuItem("Enabled", "", &enabled);
			//	ImGui::BeginChild("child", ImVec2(0, 60), true);
			//	for (int i = 0; i < 10; i++)
			//		ImGui::Text("Scrolling Text %d", i);
			//	ImGui::EndChild();
			//	static float f = 0.5f;
			//	static int n = 0;
			//	ImGui::SliderFloat("Value", &f, 0.0f, 1.0f);
			//	ImGui::InputFloat("Input", &f, 0.1f);
			//	ImGui::Combo("Combo", &n, "Yes\0No\0Maybe\0\0");
			//	ImGui::EndMenu();
			//}

			////IMGUI_DEMO_MARKER("Examples/Menu/Colors");
			//if (ImGui::BeginMenu("Colors"))
			//{
			//	float sz = ImGui::GetTextLineHeight();
			//	for (int i = 0; i < ImGuiCol_COUNT; i++)
			//	{
			//		const char* name = ImGui::GetStyleColorName((ImGuiCol)i);
			//		ImVec2 p = ImGui::GetCursorScreenPos();
			//		ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + sz, p.y + sz), ImGui::GetColorU32((ImGuiCol)i));
			//		ImGui::Dummy(ImVec2(sz, sz));
			//		ImGui::SameLine();
			//		ImGui::MenuItem(name);
			//	}
			//	ImGui::EndMenu();
			//}

			//// Here we demonstrate appending again to the "Options" menu (which we already created above)
			//// Of course in this demo it is a little bit silly that this function calls BeginMenu("Options") twice.
			//// In a real code-base using it would make senses to use this feature from very different code locations.
			//if (ImGui::BeginMenu("Options")) // <-- Append!
			//{
			//	//IMGUI_DEMO_MARKER("Examples/Menu/Append to an existing menu");
			//	static bool b = true;
			//	ImGui::Checkbox("SomeOption", &b);
			//	ImGui::EndMenu();
			//}

			//if (ImGui::BeginMenu("Disabled", false)) // Disabled
			//{
			//	IM_ASSERT(0);
			//}


			//if (ImGui::MenuItem("Checked", NULL, true)) {}
//
			if (ImGui::MenuItem("Quit", "ESC"))
				m_exitCalled = true;
			ImGui::EndMenu();
		}
		//if (ImGui::BeginMenu("View")) {
		//	if (ImGui::MenuItem("Load cesium man"))
		//		initCesiumMan();
		//	ImGui::EndMenu();
		//}
		if (ImGui::BeginMenu("Debug")) {
			if (ImGui::MenuItem("Load cesium man"))
				initCesiumMan();
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Tools")) {
			if (ImGui::MenuItem("Reset Camera")) {
				Vector3f c = Vector3f(.5,0.,-.5);
				m_Cam.lookAt(Vector3f(4.,2.5,4.)+c,c);
			}
			if (ImGui::MenuItem("Preferences"))
				m_showPopPreferences = true;
			
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();

		//TODO(skade) modal, popup prompt (e.g. delete?
		{
			// Always center this window when appearing
			//ImVec2 center = ImGui::GetMainViewport()->GetCenter();
			//ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
			
			//if (m_showPopPreferences)
			//	ImGui::OpenPopup("Preferences Popup");

			//if (ImGui::BeginPopupModal("Preferences Popup",0,ImGuiWindowFlags_AlwaysAutoResize)) {
			//	ImGui::Text("hi there");
			//	//ImGui::CloseButton()

			//	if (ImGui::Button("Close")) {
			//		m_showPopPreferences = false;
			//		ImGui::CloseCurrentPopup();
			//	}
			//	ImGui::EndPopup();
			//}
		}

		if (m_showPopPreferences) {
			ImGui::SetNextWindowSize(ImVec2(520, 600), ImGuiCond_FirstUseEver);
			//if (!ImGui::Begin("Preferences", &m_showPopPreferences)) {
			//	ImGui::End();
			//	return; //TODO(skade)
			//}
			if (ImGui::Begin("Preferences", &m_showPopPreferences)) {
				ImGui::Text("ImGuizmo");
				ImGui::Separator();
				ImGui::Text("Grid");
				ImGui::Checkbox("Render Debug Grid",&m_settings.renderDebugGrid);
				ImGui::InputScalar("Grid Size",ImGuiDataType_Float,&m_settings.gridSize);
				ImGui::Separator();
				ImGui::Text("Debug");
				ImGui::Checkbox("Render Debug Cube",&m_guizmo.m_renderDebugCube);
				ImGui::Separator();
				ImGui::Separator();
				ImGui::Checkbox(m_cesStartupStr.c_str(),&m_settings.cesStartup);

				//if (ImGui::BeginPopupContextItem()) { //TODO(skade)
				//	if (ImGui::MenuItem("Close"))
				//		m_showPopPreferences = false;
				//	ImGui::EndPopup();
				//}
			}
			ImGui::End();
		}
	}
}//renderUI_menuBar

void MotionRetargetScene::renderUI_tools() {
	ImGui::Begin("Tools");
	//TODOf(skade) settings tab
	if (ImGui::CollapsingHeader("Visualizers", ImGuiTreeNodeFlags_None)) {
		ImGui::Checkbox("Show Joints", &m_settings.showJoints);
		ImGui::SameLine();
		ImGui::Checkbox("Show Targets", &m_settings.showTargets);
		//TODOf(skade)
		//if (m_FPSLabelActive)
		//	m_FPSLabel.render(&m_RenderDev);
		//if (m_DrawHelpTexts)
		//	drawHelpTexts();
	}

	//TODO(skade) cleanup
	if (ImGui::CollapsingHeader("Guizmo", ImGuiTreeNodeFlags_Selected)) {
		m_guizmo.renderOptions();
	}
	m_guizmo.setMat(&m_guizmoMat);
	m_guizmo.setCam(&m_Cam);
	m_guizmo.render();

	ImGuizmo::SetRect(0, 0, m_RenderWin.width(), m_RenderWin.height());
	Eigen::Matrix4f cameraMat = m_Cam.cameraMatrix();
	float* cameraView = cameraMat.data();
	float* cameraProjection = m_Cam.projectionMatrix().data();
	Eigen::Matrix4f identityMatrix = Eigen::Matrix4f::Identity();

	float camDistance = cameraMat.block<3,1>(0,3).norm();

	//TODO(skade) improve focus point
	Matrix4f oldView = cameraMat;
	m_viewManipulate.render(&cameraMat,camDistance);

	if (oldView != cameraMat)
		m_guizmoViewManipChanged = true;
	else
		m_guizmoViewManipChanged = false;
	m_Cam.cameraMatrix(cameraMat);

	if (m_settings.renderDebugGrid) {
		//TODOf(skade) rotate grid when ortographic view depending on largest cam view vector component
		//if (!m_editCam.m_CamIsProj)
		
		ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix.data(), m_settings.gridSize);
	}

	ImGui::End();
}

void MotionRetargetScene::renderUI_ik() {
	ImGui::Begin("IK");

	auto c = m_charEntityPrim.lock();
	if (!c || !c->controller) {
		ImGui::End();
		return;
	}
	static CharEntity* prevC = nullptr;
	if (prevC != c.get()) { // m_charEntityPrim changed
		m_selChainIdx = -1;
		m_selChainIdxPrev = -1;
	}
	prevC = c.get();

	std::vector<IKChain>& chains = c->controller->getJointChains();

	{ // ik method
		ImGui::Checkbox("enable IK",&m_IKCupdate);
		ImGui::SameLine();
		if (ImGui::Button("singleIK"))
			m_IKCupdateSingle = true;

		std::vector<std::string> ikMstr = {
			"IKSS_NONE",
			"IKSS_CCD_B",
			"IKSS_CCD_F",
			"IKSS_CCD_FABRIK"
		};
		//TODO(skade) enum
		auto ikToIdx = [](IIKSolver* s) -> int {
			if (auto iksCCD = dynamic_cast<IKSolverCCD*>(s)) {
				if (iksCCD->m_type == IKSolverCCD::BACKWARD)
					return 1;
				else
					return 2;
			} else if (dynamic_cast<IKSolverFABRIK*>(s))
				return 3;
			return 0;
		};

		if (m_selChainIdx != -1) {
			IKChain& chain = chains[m_selChainIdx];
			int idx = ikToIdx(chain.ikSolver.get());
			int prevIdx = idx;

			ImGui::ComboStr("ik method",&idx,ikMstr,ikMstr.size());
			if (prevIdx != idx) {
				switch (idx)
				{
				case 0:
					chain.ikSolver.release();
					break;
				case 1: {
					auto ns = std::make_unique<IKSolverCCD>();
					ns->m_type = IKSolverCCD::BACKWARD;
					chain.ikSolver = std::move(ns);
				}
					break;
				case 2: {
					auto ns = std::make_unique<IKSolverCCD>();
					ns->m_type = IKSolverCCD::FORWARD;
					chain.ikSolver = std::move(ns);
				}
					break;
				case 3:
					chain.ikSolver = std::make_unique<IKSolverFABRIK>();
					break;
				default:
					break;
				}
			}
		}
	}

	if (ImGui::BeginListBox("IK Chains")) {
		for (int n = 0; n < chains.size(); n++) {
			const bool is_selected = (m_selChainIdx == n);
			if (ImGui::Selectable(chains[n].name.c_str(), is_selected)) {
				if (ImGui::GetIO().KeyCtrl) { // CTRL+click to toggle
					if (m_selChainIdx == n)
						m_selChainIdx = -1;
				}
				else
					m_selChainIdx = n;
			}

			if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
				int n_next = n + (ImGui::GetMouseDragDelta(0).y < 0.f ? -1 : 1);
				if (n_next >= 0 && n_next < chains.size()) {
					std::swap(chains[n],chains[n_next]);
					ImGui::ResetMouseDragDelta();
				}
			}

			// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
			if (is_selected)
				ImGui::SetItemDefaultFocus();
		}
		ImGui::EndListBox();
	}

	//TODOf(skade) more efficient?
	//for (uint32_t i = 0; i < items.size(); ++i) {
	//	ikChains[items[i]].
	//}

	//TODO(skade) adjust when changing to other rigged char
	// highlight chain joints green
	if (m_selChainIdx != -1) {
		auto js = chains[m_selChainIdx].joints;
		for (uint32_t i = 0; i < js.size(); ++i) {
			auto jp = c->controller->getJointPickable(js[i]).lock();
			jp->colorSelect = Vector4f(0.,1.,0.,1.);
			jp->m_highlight = true;
		}
	}
	if (m_selChainIdxPrev != m_selChainIdx && m_selChainIdxPrev != -1) {
		auto js = chains[m_selChainIdxPrev].joints;
		for (uint32_t i = 0; i < js.size(); ++i) {
			auto jp = c->controller->getJointPickable(js[i]).lock();
			jp->colorSelect = jp->colorSelect0;
			jp->m_highlight = false;
		}
	}
	m_selChainIdxPrev = m_selChainIdx;
	
	if (ImGui::CollapsingHeader("IKChain operations")) {
		//if (m_selChainIdx != -1) {
		//	auto& chain = chains[m_selChainIdx];
		//	ImGui::DragFloat("weight",&chain.weight,.005,0.,1.);
		//}

		renderUI_ikChainEditor(&m_selChainIdx);

		std::string targetName = "none";
		if (m_selChainIdx != -1)
			if (auto t = chains[m_selChainIdx].target.lock())
				targetName = t->name;
		if (ImGui::Button("set target")) {
			if (auto p = std::dynamic_pointer_cast<IKTarget>(m_picker.getLastPick().lock())) {
				chains[m_selChainIdx].target = p; //TODO(skade) targets need to be smart ptr when decoupled
				// problem when p is from other charEntity
			}
		}
		ImGui::SameLine();
		ImGui::Text((std::string("target: ") + targetName).c_str());
		
		if (ImGui::Button("delete selected chain")) {
			if (chains.size() > 0 && m_selChainIdx != -1) {

				//TODO(skade) improve location
				auto js = chains[m_selChainIdx].joints;
				for (uint32_t i = 0; i < js.size(); ++i) {
					auto jp = c->controller->getJointPickable(js[i]).lock();
					jp->colorSelect = jp->colorSelect0;
					jp->m_highlight = false;
				}

				chains.erase(chains.begin() + m_selChainIdx);
				
				m_selChainIdx = -1;
				m_selChainIdxPrev = -1;
			}
		}
	}
	if (ImGui::CollapsingHeader("Target operations")) {
		renderUI_ikTargetEditor();
	}
	ImGui::End();
}

void MotionRetargetScene::renderUI_ikChainEditor(int* item_current_idx) {
	if (ImGui::Button("edit chain")) {
		m_showPopChainEdit = true;
	}
	auto c = m_charEntityPrim.lock();
	if (!c || !c->controller) {
		ImGui::End();
		return;
	}
	static CharEntity* prevC = nullptr;
	if (prevC != c.get()) { // m_charEntityPrim changed
		m_ikceNameInit = false;
		m_ikceName = "new";
		m_ikceRootJoint = nullptr;
		m_ikceEndEffJoint = nullptr;
	}
	prevC = c.get();
	auto& chains = c->controller->getJointChains();

	if (m_showPopChainEdit) {
		// Always center this window when appearing
		ImVec2 center = ImGui::GetMainViewport()->GetCenter();
		ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

		if (ImGui::Begin("add ik chain", &m_showPopChainEdit)) {
			if (!m_ikceNameInit) {
				if (*item_current_idx != -1)
					m_ikceName = chains[*item_current_idx].name;
				m_ikceNameInit = true;
			}
			ImGui::InputText("chain name:", &m_ikceName);

			ImVec2 subSize = ImGui::GetWindowSize();
			subSize.y *= .4;

			{
				ImGui::BeginChild("root joint",subSize,true);
				std::string rootJointName = "none";
				if (m_ikceRootJoint)
					rootJointName = m_ikceRootJoint->Name;
				ImGui::Text((std::string("root joint: ") + rootJointName).c_str());
				ImGui::Separator();
				IKController::SkeletalJoint* clickedJoint = renderUI_OutlinerJoints(c,m_ikceRootJoint);

				// change color
				if (clickedJoint) {
					if (m_ikceRootJoint) {
						auto jp = c->controller->getJointPickable(m_ikceRootJoint).lock();
						jp->restoreColor();
						jp->m_highlight = false;
					}
					m_ikceRootJoint = clickedJoint;
					auto jp = c->controller->getJointPickable(m_ikceRootJoint).lock();
					jp->colorSelect = Vector4f(1.,0.,0.,1.);
					jp->m_highlight = true;
				}
				ImGui::EndChild();
			}

			{
				ImGui::BeginChild("endEff joint",subSize, true);
				std::string endEffJointName = "none";
				if (m_ikceEndEffJoint)
					endEffJointName = m_ikceEndEffJoint->Name;
				ImGui::Text((std::string("endEff joint: ") + endEffJointName).c_str());
				ImGui::Separator();
				IKController::SkeletalJoint* clickedJoint = renderUI_OutlinerJoints(c,m_ikceEndEffJoint);
				if (clickedJoint) {
					if (m_ikceEndEffJoint) {
						auto jp = c->controller->getJointPickable(m_ikceEndEffJoint).lock();
						jp->restoreColor();
						jp->m_highlight = false;
					}
					m_ikceEndEffJoint = clickedJoint;
					auto jp = c->controller->getJointPickable(m_ikceEndEffJoint).lock();
					jp->colorSelect = Vector4f(0.,0.,1.,1.);
					jp->m_highlight = true;
				}
				ImGui::EndChild();
			}

			if (ImGui::Button("Confirm")) {
				//IKChain nChain = c->controller->getJointChains()[m_ikceName];
				IKChain* nChain = c->controller->getIKChain(m_ikceName);
				if (!nChain) {
					c->controller->getJointChains().emplace_back();
					nChain = &c->controller->getJointChains().back();
				}
				nChain->name = m_ikceName;
				
				nChain->joints.clear();
				{
					IKController::SkeletalJoint* j = m_ikceEndEffJoint;
					nChain->joints.push_back(j);
					do {
						j = c->controller->getBone(j->Parent);
						nChain->joints.push_back(j);
					} while (j != m_ikceRootJoint);
				}
				
				nChain->pRoot = &c->controller->m_IKJoints[m_ikceRootJoint];
				//c->controller->getJointChains()[m_ikceName] = nChain;
				
				m_ikceName = "new"; m_ikceNameInit = false;
				m_showPopChainEdit = false;
			}
			ImGui::End();
		}
	} // if addChainPopup
	else {
		m_ikceName = "new"; m_ikceNameInit = false;
		//TODO(skade) bug rootJoint still from other charentity on swap
		if (m_ikceRootJoint) {
			auto jp = c->controller->getJointPickable(m_ikceRootJoint).lock();
			if (jp) {
				jp->restoreColor();
				jp->m_highlight = false;
			}
		}
		if (m_ikceEndEffJoint) {
			auto jp = c->controller->getJointPickable(m_ikceEndEffJoint).lock();
			if (jp) {
				jp->restoreColor();
				jp->m_highlight = false;
			}
		}
	}
}
void MotionRetargetScene::renderUI_ikTargetEditor() {
	auto c = m_charEntityPrim.lock(); //TODO(skade) targets global?
	if (!c || !c->controller) {
		ImGui::End();
		return;
	}
	auto& chains = c->controller->getJointChains();
	static std::string newTargetName = "new target";
	ImGui::InputText("new target name",&newTargetName);
	if (ImGui::Button("add new target")) {
		std::string name = newTargetName;
		BoundingVolume bv;
		Vector3f d =Vector3f(0.05f, 0.05f, 0.05f);
		Box b; b.init(-d*.5,d*.5);
		bv.init(b);

		c->controller->m_targets.emplace_back(std::make_shared<IKTarget>(name,bv));
	}

	if (ImGui::Button("remove selected target")) {
		if (auto t = std::dynamic_pointer_cast<IKTarget>(m_picker.getLastPick().lock())) {
			auto& targets = c->controller->m_targets;
			auto tPos = std::find(targets.begin(),targets.end(),t);
			if (tPos != targets.end())
				targets.erase(tPos);
		}
	}
}

}//CForge
