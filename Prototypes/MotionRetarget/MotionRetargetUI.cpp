#include "MotionRetargetScene.hpp"

#include <Prototypes/GUI/ImGuiUtility.h>
#include "UI/ImGuiStyle.hpp"
#include "UI/IKSequencer.hpp"
#include <crossforge/AssetIO/UserDialog.h>
//TODO(skade) for ImGuiUtility::initImGui replace
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
//TODO(skade)

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

	//TODOf(skade) improvable dockbar/
	//ImGui::DockSpace(ImGui::GetID("ig_dock_main"),ImVec2(0.,0.),ImGuiDockNodeFlags_None);
	ImGui::DockSpaceOverViewport(igViewPort,ImGuiDockNodeFlags_PassthruCentralNode);

	//m_gui.render(); //TODO(skade) move into other class?

	renderUI_menuBar();

	{
		ImGui::Begin("Outliner");
		//TODO(skade) abstract to scenegraph
		
		//TODO(skade) make Modal window for joint matching?
		
		ImGui::End();
	}

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


void MotionRetargetScene::renderUI_animation() {
	ImGui::Begin("Animation");
	if (!m_IKActorPrim || !m_IKControllerPrim) {
		ImGui::End();
		return;
	}

	std::vector<std::string> items;
	items.push_back("none");
	for (uint32_t i=0;i<m_IKActorPrim->getController()->animationCount();++i) {
		items.push_back(m_IKActorPrim->getController()->animation(i)->Name);
	}
	ImGui::ComboStr("select animation",&m_current_anim_item,items,items.size());

	if (m_current_anim_item > 0) { // anim selected
		if (m_pAnimCurr && m_current_anim_item - 1 != m_pAnimCurr->AnimationID) { // Animation changed
			m_IKControllerPrim->destroyAnimation(m_pAnimCurr);
			m_IKActorPrim->activeAnimation(nullptr);
			m_pAnimCurr = nullptr;
		}
		if (!m_pAnimCurr) { // create animation if not existing
			m_pAnimCurr = m_IKControllerPrim->createAnimation(m_current_anim_item-1,1.f,0.f);
			m_IKActorPrim->activeAnimation(m_pAnimCurr);
		}

		T3DMesh<float>::SkeletalAnimation* anim = m_IKActorPrim->getController()->animation(m_current_anim_item-1);
		ImGui::Text("Duration: %f",anim->Duration);
		ImGui::Text("SamplesPerSecond: %f",anim->SamplesPerSecond);
		if(ImGui::Button("Play")) {
			m_animAutoplay = true;
		}
		if (m_animAutoplay) {
			if(ImGui::Button("Stop")) {
				m_animAutoplay = false;
			}
			//ImGui::InputScalar("animSpeed", ImGuiDataType_Float, &(m_pAnimCurr->Speed));
			ImGui::DragFloat("animSpeed", &(m_pAnimCurr->Speed), 0.01f);
		}
		ImGui::Text("pAnim->t: %f",m_pAnimCurr->t);
	}
	else {
		m_IKControllerPrim->destroyAnimation(m_pAnimCurr);
		m_IKActorPrim->activeAnimation(nullptr);
		m_pAnimCurr = nullptr;
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

	mySequence.myItems.clear();
	if (m_pAnimCurr) {
		int animFrameCount = m_pAnimCurr->Duration * m_pAnimCurr->SamplesPerSecond;
		//int animFrameCount = m_IKControllerPrim->animation(m_pAnimCurr->AnimationID)->Keyframes[0]->Positions.size(); //TODO(skade)
		animFrameCount -= 1; // visualizer is inclusive
		mySequence.myItems.push_back(MySequence::MySequenceItem{0,0,animFrameCount,false});
	}

	ImGui::PushItemWidth(130);
	ImGui::InputInt("Frame \t\t", &m_animFrameCurr);
	ImGui::SameLine();
	ImGui::InputInt("Min \t\t", &mySequence.mFrameMin);
	ImGui::SameLine();
	ImGui::InputInt("Max", &mySequence.mFrameMax);
	ImGui::PopItemWidth();

	//TODO(skade) make member
	static int selectedEntry = -1;
	static int firstFrame = 0;
	static bool expanded = true;
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
					loadCharPrim(path);
				}
				if (ImGui::MenuItem("Assimp")) {
					//TODO(skade) list assimp types
					std::string path = UserDialog::OpenFile("load primary char", "assimp");

					if (SAssetIO::accepted(path, I3DMeshIO::Operation::OP_LOAD)) {
						m_MeshCharPrim = std::make_unique<T3DMesh<float>>();
						SAssetIO::load(path,m_MeshCharPrim.get());
						initCharacter();
					}
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Export")) {
				if (ImGui::MenuItem("GLTF", ".gltf, .glb")) {
					//TODO(skade) name
					std::string path = UserDialog::SaveFile("store primary char", "gltf", "*.gltf *.glb");
					storeCharPrim(path);
				}
				if (ImGui::MenuItem("Assimp")) {
					//TODO(skade) list assimp types
					std::string path = UserDialog::SaveFile("store primary char", "assimp");

					if (SAssetIO::accepted(path, I3DMeshIO::Operation::OP_STORE))
						SAssetIO::store(path,m_MeshCharPrim.get());
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
		ImGui::EndMainMenuBar();
	}
}//renderUI_menuBar

void MotionRetargetScene::renderUI_tools() {
	ImGui::Begin("Tools");
	//TODOf(skade) settings tab
	if (ImGui::CollapsingHeader("Visualizers", ImGuiTreeNodeFlags_None)) {
		ImGui::Checkbox("Show Joints", &m_showJoints);
		if (m_IKControllerPrim) {
			static float jpoPrev = m_IKControllerPrim.get()->getJointOpacity();
			static float jpo = jpoPrev;
			ImGui::DragFloat("Joint Opacity",&jpo,.005,0.,1.);
			if (jpo != jpoPrev) {
				m_IKControllerPrim.get()->setJointOpacity(jpo);
				jpoPrev = jpo;
			}
		}

		ImGui::Checkbox("Show Targets", &m_showTarget);
		ImGui::Checkbox("Render Debug Grid",&m_renderDebugGrid);
		ImGui::InputScalar("Grid Size",ImGuiDataType_Float,&m_gridSize);

		//TODO(skade)
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

	if (m_renderDebugGrid)
		ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix.data(), m_gridSize);

	ImGui::End();
}

void MotionRetargetScene::renderUI_ik() {
	ImGui::Begin("IK");

	if (m_IKControllerPrim) {
		IKTarget* lp = dynamic_cast<IKTarget*>(m_picker.getLastPick());
		if (lp) {
			IKChain* chain = m_IKControllerPrim->getIKChain(lp);
			assert(chain);
			ImGui::Text("IK segmentName: %s",chain->name.c_str());
		} else
			ImGui::Text("IK segmentName none");

		std::vector<std::string> items = {
			"IKSS_CCD_F",
			"IKSS_CCD_B",
			"IKSS_CCD_FABRIK"
		};
		int currItem = m_IKControllerPrim->testIKslvSelect;
		ImGui::ComboStr("ik method",&currItem,items,items.size());
		m_IKControllerPrim->testIKslvSelect = static_cast<IKController::TestIKslvSelect>(currItem);

		ImGui::Checkbox("enable IK",&m_IKCupdate);
		if (ImGui::Button("singleIK"))
			m_IKCupdateSingle = true;
	}

	ImGui::End();
}

}//CForge
