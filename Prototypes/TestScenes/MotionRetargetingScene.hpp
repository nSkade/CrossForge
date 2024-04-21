#pragma once

#include "../InverseKinematics/IKSkeletalActor.h"
#include "../InverseKinematics/IKStickFigureActor.h"
#include "../../crossforge/MeshProcessing/PrimitiveShapeFactory.h"

#include "../../Examples/ExampleSceneBase.hpp"

#include <Prototypes/Assets/GLTFIO/GLTFIO.hpp>

#include <crossforge/Graphics/Actors/SkeletalActor.h>

#include <Prototypes/GUI/ImGuiUtility.h>
#include <Prototypes/InverseKinematics/IKImGui.hpp>
#include <Prototypes/InverseKinematics/Guizmo.hpp>

#include <Prototypes/InverseKinematics/Config.hpp>

using namespace Eigen;
using namespace std;

//TODO(skade) put in config / settings
// https://gist.github.com/dougbinks/8089b4bbaccaaf6fa204236978d165a9#file-imguiutils-h-L9-L93
inline void SetupImGuiStyle(bool is_dark_style, float alpha_threshold) {
	//Use a ternary operator
	is_dark_style ? ImGui::StyleColorsDark() : ImGui::StyleColorsLight();

	ImGuiStyle& style = ImGui::GetStyle();

	// Adjusts the alpha values of the ImGui colors based on the alpha threshold.
	for (int i = 0; i < ImGuiCol_COUNT; i++) {
		const auto color_id = static_cast<ImGuiCol>(i);
		auto& color = style.Colors[i];
		if (color.w < alpha_threshold || color_id == ImGuiCol_FrameBg || color_id == ImGuiCol_WindowBg || color_id == ImGuiCol_ChildBg)
			color.w *= alpha_threshold;
	}

	// Sets the border sizes and rounding.
	style.ChildBorderSize = 1.0f;
	style.FrameBorderSize = 0.0f;
	style.PopupBorderSize = 1.0f;
	style.WindowBorderSize = 0.0f;
	style.FrameRounding = 3.0f;
	style.Alpha = 1.0f;
}

namespace CForge {

	struct RampEdit : public ImCurveEdit::Delegate
	{
		RampEdit()
		{
			mPts[0][0] = ImVec2(-10.f, 0);
			mPts[0][1] = ImVec2(20.f, 0.6f);
			mPts[0][2] = ImVec2(25.f, 0.2f);
			mPts[0][3] = ImVec2(70.f, 0.4f);
			mPts[0][4] = ImVec2(120.f, 1.f);
			mPointCount[0] = 5;

			mPts[1][0] = ImVec2(-50.f, 0.2f);
			mPts[1][1] = ImVec2(33.f, 0.7f);
			mPts[1][2] = ImVec2(80.f, 0.2f);
			mPts[1][3] = ImVec2(82.f, 0.8f);
			mPointCount[1] = 4;


			mPts[2][0] = ImVec2(40.f, 0);
			mPts[2][1] = ImVec2(60.f, 0.1f);
			mPts[2][2] = ImVec2(90.f, 0.82f);
			mPts[2][3] = ImVec2(150.f, 0.24f);
			mPts[2][4] = ImVec2(200.f, 0.34f);
			mPts[2][5] = ImVec2(250.f, 0.12f);
			mPointCount[2] = 6;
			mbVisible[0] = mbVisible[1] = mbVisible[2] = true;
			mMax = ImVec2(1.f, 1.f);
			mMin = ImVec2(0.f, 0.f);
		}
		size_t GetCurveCount()
		{
			return 3;
		}

		bool IsVisible(size_t curveIndex)
		{
			return mbVisible[curveIndex];
		}
		size_t GetPointCount(size_t curveIndex)
		{
			return mPointCount[curveIndex];
		}

		uint32_t GetCurveColor(size_t curveIndex)
		{
			uint32_t cols[] = { 0xFF0000FF, 0xFF00FF00, 0xFFFF0000 };
			return cols[curveIndex];
		}
		ImVec2* GetPoints(size_t curveIndex)
		{
			return mPts[curveIndex];
		}
		virtual ImCurveEdit::CurveType GetCurveType(size_t curveIndex) const { return ImCurveEdit::CurveSmooth; }
		virtual int EditPoint(size_t curveIndex, int pointIndex, ImVec2 value)
		{
			mPts[curveIndex][pointIndex] = ImVec2(value.x, value.y);
			SortValues(curveIndex);
			for (size_t i = 0; i < GetPointCount(curveIndex); i++)
			{
				if (mPts[curveIndex][i].x == value.x)
					return (int)i;
			}
			return pointIndex;
		}
		virtual void AddPoint(size_t curveIndex, ImVec2 value)
		{
			if (mPointCount[curveIndex] >= 8)
				return;
			mPts[curveIndex][mPointCount[curveIndex]++] = value;
			SortValues(curveIndex);
		}
		virtual ImVec2& GetMax() { return mMax; }
		virtual ImVec2& GetMin() { return mMin; }
		virtual unsigned int GetBackgroundColor() { return 0; }
		ImVec2 mPts[3][8];
		size_t mPointCount[3];
		bool mbVisible[3];
		ImVec2 mMin;
		ImVec2 mMax;
	private:
		void SortValues(size_t curveIndex)
		{
			auto b = std::begin(mPts[curveIndex]);
			auto e = std::begin(mPts[curveIndex]) + GetPointCount(curveIndex);
			std::sort(b, e, [](ImVec2 a, ImVec2 b) { return a.x < b.x; });
		}
	};

	static const char* SequencerItemTypeNames[] = { "Animation" };
	struct MySequence : public ImSequencer::SequenceInterface
	{
		// interface with sequencer

		virtual int GetFrameMin() const {
			return mFrameMin;
		}
		virtual int GetFrameMax() const {
			return mFrameMax;
		}
		virtual int GetItemCount() const { return (int)myItems.size(); }

		virtual int GetItemTypeCount() const { return sizeof(SequencerItemTypeNames) / sizeof(char*); }
		virtual const char* GetItemTypeName(int typeIndex) const { return SequencerItemTypeNames[typeIndex]; }
		virtual const char* GetItemLabel(int index) const
		{
			static char tmps[512];
			snprintf(tmps, 512, "[%02d] %s", index, SequencerItemTypeNames[myItems[index].mType]);
			return tmps;
		}

		virtual void Get(int index, int** start, int** end, int* type, unsigned int* color)
		{
			MySequenceItem& item = myItems[index];
			if (color)
				*color = 0xFFAA8080; // same color for everyone, return color based on type
			if (start)
				*start = &item.mFrameStart;
			if (end)
				*end = &item.mFrameEnd;
			if (type)
				*type = item.mType;
		}
		virtual void Add(int type) { myItems.push_back(MySequenceItem{ type, 0, 10, false }); };
		virtual void Del(int index) { myItems.erase(myItems.begin() + index); }
		virtual void Duplicate(int index) { myItems.push_back(myItems[index]); }

		virtual size_t GetCustomHeight(int index) { return myItems[index].mExpanded ? 300 : 0; }

		// my datas
		MySequence() : mFrameMin(0), mFrameMax(0) {}
		int mFrameMin, mFrameMax;
		struct MySequenceItem
		{
			int mType;
			int mFrameStart, mFrameEnd;
			bool mExpanded;
		};
		std::vector<MySequenceItem> myItems;
		RampEdit rampEdit;

		virtual void DoubleClick(int index) {
			if (myItems[index].mExpanded)
			{
				myItems[index].mExpanded = false;
				return;
			}
			for (auto& item : myItems)
				item.mExpanded = false;
			myItems[index].mExpanded = !myItems[index].mExpanded;
		}

		virtual void CustomDraw(int index, ImDrawList* draw_list, const ImRect& rc, const ImRect& legendRect, const ImRect& clippingRect, const ImRect& legendClippingRect)
		{
			static const char* labels[] = { "Translation", "Rotation" , "Scale" };

			rampEdit.mMax = ImVec2(float(mFrameMax), 1.f);
			rampEdit.mMin = ImVec2(float(mFrameMin), 0.f);
			draw_list->PushClipRect(legendClippingRect.Min, legendClippingRect.Max, true);
			for (int i = 0; i < 3; i++)
			{
				ImVec2 pta(legendRect.Min.x + 30, legendRect.Min.y + i * 14.f);
				ImVec2 ptb(legendRect.Max.x, legendRect.Min.y + (i + 1) * 14.f);
				draw_list->AddText(pta, rampEdit.mbVisible[i] ? 0xFFFFFFFF : 0x80FFFFFF, labels[i]);
				if (ImRect(pta, ptb).Contains(ImGui::GetMousePos()) && ImGui::IsMouseClicked(0))
					rampEdit.mbVisible[i] = !rampEdit.mbVisible[i];
			}
			draw_list->PopClipRect();

			ImGui::SetCursorScreenPos(rc.Min);
			ImCurveEdit::Edit(rampEdit, rc.Max - rc.Min, 137 + index, &clippingRect);
		}

		virtual void CustomDrawCompact(int index, ImDrawList* draw_list, const ImRect& rc, const ImRect& clippingRect)
		{
			rampEdit.mMax = ImVec2(float(mFrameMax), 1.f);
			rampEdit.mMin = ImVec2(float(mFrameMin), 0.f);
			draw_list->PushClipRect(clippingRect.Min, clippingRect.Max, true);
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < rampEdit.mPointCount[i]; j++)
				{
					float p = rampEdit.mPts[i][j].x;
					if (p < myItems[index].mFrameStart || p > myItems[index].mFrameEnd)
						continue;
					float r = (p - mFrameMin) / float(mFrameMax - mFrameMin);
					float x = ImLerp(rc.Min.x, rc.Max.x, r);
					draw_list->AddLine(ImVec2(x, rc.Min.y + 6), ImVec2(x, rc.Max.y - 4), 0xAA000000, 4.f);
				}
			}
			draw_list->PopClipRect();
		}
	};

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
			initTestActors();
			initRotationTest();
			
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

			for (auto& i : m_EndEffectors) if (nullptr != i) delete i;
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

			m_config.store(m_Cam);
			m_config.baseStore();
		}

		void mainLoop(void)override {
			m_RenderWin.update();
			m_SG.update(60.0f / m_FPS);

			if (m_IKCupdate)
				m_IKController.update(60.0f / m_FPS);
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

			////TEST
			//moveTargetKeyboard(InverseKinematicsController::HEAD);
			////TEST

			m_RenderDev.activePass(RenderDevice::RENDERPASS_SHADOW, &m_Sun);
			m_RenderDev.activeCamera(const_cast<VirtualCamera*>(m_Sun.camera()));
			m_SG.render(&m_RenderDev);
			
			m_RenderDev.activePass(RenderDevice::RENDERPASS_GEOMETRY);
			m_RenderDev.activeCamera(&m_Cam);
			m_SG.render(&m_RenderDev);

			m_RenderDev.activePass(RenderDevice::RENDERPASS_LIGHTING);
			
			m_RenderDev.activePass(RenderDevice::RENDERPASS_FORWARD, nullptr, false);
			//m_SkyboxSG.render(&m_RenderDev);

			glDisable(GL_DEPTH_TEST);
			//glEnable(GL_BLEND); //TODO(skade)f change blendmode in active material forward pass
			//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			if (m_showEffector)
				m_EffectorVis.render(&m_RenderDev,Eigen::Vector3f::Zero(),Eigen::Quaternionf::Identity(),Eigen::Vector3f(1.f,1.f,1.f));
			if (m_showTarget)
				m_TargetVis.render(&m_RenderDev,Eigen::Vector3f::Zero(),Eigen::Quaternionf::Identity(),Eigen::Vector3f(1.f,1.f,1.f));
			//glDisable(GL_BLEND);
			glEnable(GL_DEPTH_TEST);


			if (m_visualizeJoints) { //TODO(skade) put in function
				glDisable(GL_DEPTH_TEST);
				glEnable(GL_BLEND);

				for (uint32_t i = 0; i < m_IKController.boneCount(); i++) {
					Eigen::Matrix4f loct = CForgeMath::translationMatrix(m_IKController.getBone(i)->LocalPosition);
					Eigen::Matrix4f cubeTransform = m_IKController.getBone(i)->SkinningMatrix
													* m_IKController.getBone(i)->OffsetMatrix.inverse(); // Transform to restpose Space
					float r = 0.1f;
					Eigen::Matrix4f cubeScale = CForgeMath::scaleMatrix(Eigen::Vector3f(r,r,r)*1.0f);
					m_RenderDev.modelUBO()->modelMatrix(cubeTransform*cubeScale);
					m_JointVisActor.render(&m_RenderDev,Eigen::Quaternionf::Identity(),Eigen::Vector3f(),Eigen::Vector3f(1.f,1.f,1.f));
				}
				glDisable(GL_BLEND);
				glEnable(GL_DEPTH_TEST);
			}

			ImGuiUtility::newFrame();
			ImGuizmo::SetOrthographic(false); //TODO(skade)
			ImGuizmo::BeginFrame();

			static int current_anim_item = 0;

			// let's create the sequencer
			static int selectedEntry = -1;
			static int firstFrame = 0;
			static bool expanded = true;
			static int currentFrame = 100;
			static SkeletalAnimationController::Animation* pAnim = nullptr; //TODO(skade) rename
			static int animAutoplay = false;

			if (pAnim) {
				if (animAutoplay) {
					currentFrame = pAnim->t*pAnim->TicksPerSecond;
					pAnim->t += 1.f/m_FPS*pAnim->Speed;
					if (pAnim->t > pAnim->Duration)
						pAnim->t = 0.f;
				} else {
					pAnim->t = currentFrame/pAnim->TicksPerSecond;
				}
				m_IKController.updateBones(pAnim);
				//m_IKController.updateEndEffectorPoints();
			}

			//m_gui.render(); //TODO(skade)
			{
				ImGui::Begin("Animation");
				//ImGui::SetNextWindowSize(ImVec2(m_WinWidth,m_WinHeight));
				//ImGui::SetNextWindow

				std::vector<std::string> items;
				items.push_back("none");
				for (uint32_t i=0;i<m_IKActor.getController()->animationCount();++i) {
					items.push_back(m_IKActor.getController()->animation(i)->Name);
				}
				auto ComboStr = [](const char* label, int* current_item, const std::vector<std::string>& items, int items_count, int height_in_items = -1)
				{
					return ImGui::Combo(label, current_item, [](void* data, int idx, const char** out_text) { *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str(); return true; }, (void*)&items, items_count, height_in_items);
				};
				ComboStr("select animation",&current_anim_item,items,items.size());

				if (current_anim_item > 0) { // anim selected
					if (pAnim && current_anim_item - 1 != pAnim->AnimationID) { // Animation changed
						m_IKController.destroyAnimation(pAnim);
						m_IKActor.activeAnimation(nullptr);
						m_IKStickActor.activeAnimation(nullptr);
						pAnim = nullptr;
					}
					if (!pAnim) { // create animation if not existing
						pAnim = m_IKController.createAnimation(current_anim_item-1,1.f,0.f);
						m_IKActor.activeAnimation(pAnim);
						m_IKStickActor.activeAnimation(pAnim);
					}

					T3DMesh<float>::SkeletalAnimation* anim = m_IKActor.getController()->animation(current_anim_item-1);
					ImGui::Text("Duration: %f",anim->Duration);
					ImGui::Text("SamplesPerSecond: %f",anim->SamplesPerSecond);
					if(ImGui::Button("Play")) {
						animAutoplay = true;
					}
					if (animAutoplay) {
						if(ImGui::Button("Stop")) {
							animAutoplay = false;
						}
						ImGui::InputScalar("animSpeed", ImGuiDataType_Float, &(pAnim->Speed));
					}
					ImGui::Text("pAnim->t: %f",pAnim->t);
				}
				else {
					m_IKController.destroyAnimation(pAnim);
					m_IKActor.activeAnimation(nullptr);
					m_IKStickActor.activeAnimation(nullptr);
					pAnim = nullptr;
				}

				ImGui::Checkbox("enable IK",&m_IKCupdate);

				bool sfaEnabled;
				m_IKStickActorSGN.enabled(nullptr, &sfaEnabled);
				ImGui::Checkbox("enable StickFigureActor",&sfaEnabled);
				if (sfaEnabled) {
					m_CharacterSGN.enable(true, false);
					m_IKStickActorSGN.enable(true, true);
				} else {
					m_CharacterSGN.enable(true, true);
					m_IKStickActorSGN.enable(true, false);
				}
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
				ImGui::InputInt("Frame ", &currentFrame);
				ImGui::SameLine();
				ImGui::InputInt("Frame Max", &mySequence.mFrameMax);
				ImGui::PopItemWidth();
				Sequencer(&mySequence, &currentFrame, &expanded, &selectedEntry, &firstFrame,
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
				ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(viewManipulateRight - 128, viewManipulateTop), ImVec2(128, 128), 0x10101010);
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

			//TODO(skade) load/store skeleton, bvh
			//{
			//	ImGui::Begin("main");
			//}

			ImGuiUtility::render();

			m_RenderWin.swapBuffers();
			
			updateFPS();
			defaultKeyboardUpdate(m_RenderWin.keyboard());
		}//mainLoop

	protected:
		void rayCast(Eigen::Vector3f* ro, Eigen::Vector3f* rd) {
			Vector4f Viewport = Vector4f(0.0f, 0.0f, float(m_RenderWin.width()), float(m_RenderWin.height())); 
			Vector2f CursorPos = Vector2f(m_RenderWin.mouse()->position().x(), Viewport(3) - m_RenderWin.mouse()->position().y());
			Matrix4f View = m_Cam.cameraMatrix();
			Matrix4f Projection = m_Cam.projectionMatrix();
			
			igl::unproject_ray(CursorPos, View, Projection, Viewport, *ro, *rd);
			rd->normalize();
		}

		void pickTarget(void) {
			Eigen::Vector3f RayOrigin, RayDirection;
			rayCast(&RayOrigin, &RayDirection);

			for (int32_t i = 0; i < m_EndEffectors.size(); ++i) {
				auto* pEndEffector = m_EndEffectors[i];
				auto& TargetTransforms = m_TargetTransformSGNs.at(pEndEffector->segmentName);

				int32_t AABBIndex = pEndEffector->TargetPoints.cols() - 1;
				Vector3f AABBPos = TargetTransforms[AABBIndex]->translation();
				AlignedBox3f TranslatedAABB = AlignedBox3f(m_TargetMarkerAABB.min() + AABBPos, m_TargetMarkerAABB.max() + AABBPos);

				const float T0 = 0.0f;
				const float T1 = m_Cam.farPlane();
				float TMin, TMax; // minimum and maximum of interval of overlap within [T0, T1] -> not actually used here, but required by function

				//TODOf(skade) check for distance in case on screenspace overlap of multiple aabb
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
			if (m_SelectedEffectorTarget == -1)
				m_LastSelectedEffectorTarget = -1;
			m_SelectedEffectorTarget = -1; // assume nothing will be picked
		}//pickTarget

		void dragTarget(int target) {
			if (target == -1)
				return;

			InverseKinematicsController::SkeletalEndEffector* pEndEffector = m_EndEffectors[target];
			std::vector<SGNTransformation*>& TargetTransforms = m_TargetTransformSGNs.at(pEndEffector->segmentName);

			// apply translation to target points in character controller
			m_IKController.translateTarget(pEndEffector->segmentName, m_guizmoMat.block<3,1>(0,3));
			
			Eigen::Matrix3Xf targetPoints = m_IKController.getTargetPoints(pEndEffector->segmentName);
			// apply translation to target marker transformation SGNs
			for (int32_t i = 0; i < TargetTransforms.size(); ++i) {
				if (TargetTransforms[i] == nullptr || i >= targetPoints.cols())
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

			m_IKController.init(&M, "MyAssets/ccd-ik/ces/SkeletonConfig.json");
			//TODO(skade) write option without json
			//m_IKController.init(&M);

			for (uint32_t i=0;i<M.skeletalAnimationCount();++i)
				m_IKController.addAnimationData(M.getSkeletalAnimation(i));
			m_IKActor.init(&M, &m_IKController);
			m_CharacterSGN.init(&m_RootSGN, &m_IKActor);

			//TODO(skade) remove/rewrite
			m_IKStickActor.init(&M, &m_IKController);
			m_IKStickActorSGN.init(&m_RootSGN, &m_IKStickActor);
			m_IKStickActorSGN.enable(true, false);
			M.clear();
		}//initActors

		void initEndEffectorMarkers(void) {
			m_EndEffectors = m_IKController.retrieveEndEffectors();

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

			for (uint32_t i = 0; i < M.materialCount(); ++i) {
				auto* pMat = M.getMaterial(i);
				pMat->Color = Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
			}
			m_TargetX.init(&M);

			for (uint32_t i = 0; i < M.materialCount(); ++i) {
				auto* pMat = M.getMaterial(i);
				pMat->Color = Vector4f(0.0f, 1.0f, 0.0f, 1.0f);
			}
			m_TargetY.init(&M);

			for (uint32_t i = 0; i < M.materialCount(); ++i) {
				auto* pMat = M.getMaterial(i);
				pMat->Color = Vector4f(0.0f, 0.0f, 1.0f, 1.0f);
			}
			m_TargetZ.init(&M);
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
				m_EffectorTransformSGNs.try_emplace(m_EndEffectors[i]->segmentName);
				m_EffectorGeomSGNs.try_emplace(m_EndEffectors[i]->segmentName);
				m_TargetTransformSGNs.try_emplace(m_EndEffectors[i]->segmentName);
				m_TargetGeomSGNs.try_emplace(m_EndEffectors[i]->segmentName);

				auto& EffectorTransforms = m_EffectorTransformSGNs.at(m_EndEffectors[i]->segmentName);
				auto& EffectorGeoms = m_EffectorGeomSGNs.at(m_EndEffectors[i]->segmentName);
				auto& TargetTransforms = m_TargetTransformSGNs.at(m_EndEffectors[i]->segmentName);
				auto& TargetGeoms = m_TargetGeomSGNs.at(m_EndEffectors[i]->segmentName);

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
				for (int32_t j = 0; j < m_EndEffectors[i]->EndEffectorPoints.cols(); ++j) {
					EffectorTransforms.push_back(new SGNTransformation());
					EffectorGeoms.push_back(new SGNGeometry());
					EffectorTransforms.back()->init(&m_EffectorVis, m_EndEffectors[i]->EndEffectorPoints.col(j));
					
					StaticActor* m_efac = &m_EffectorPos;

					if (j == 0)
						m_efac = &m_EffectorZ;
					else if (j==m_EndEffectors[i]->EndEffectorPoints.cols()-1)
						m_efac = &m_EffectorY;
					else
						m_efac = &m_EffectorX;

					EffectorGeoms[j]->init(EffectorTransforms[j], m_efac);
				}

				for (int32_t j = 0; j < m_EndEffectors[i]->EndEffectorPoints.cols(); ++j) {
					TargetTransforms.push_back(new SGNTransformation());
					TargetGeoms.push_back(new SGNGeometry());
					EffectorGeoms.back()->init(EffectorTransforms[j], &m_EffectorPos);
					TargetTransforms.back()->init(&m_TargetVis, m_EndEffectors[i]->TargetPoints.col(j));
					TargetGeoms.back()->init(TargetTransforms.back(), &m_TargetPos);
				}

				int32_t AABBIndex = m_EndEffectors[i]->EndEffectorPoints.cols()-1;

				TargetTransforms[AABBIndex] = new SGNTransformation();
				TargetGeoms[AABBIndex] = new SGNGeometry();

				//TODO(skade)
				//Vector3f AABBPos = m_EndEffectors[i]->TargetPoints.rowwise().sum();
				//AABBPos /= m_EndEffectors[i]->TargetPoints.cols();

				Vector3f AABBPos = m_EndEffectors[i]->TargetPoints.col(0);
				
				TargetTransforms[AABBIndex]->init(&m_TargetVis, AABBPos);
				TargetGeoms[AABBIndex]->init(TargetTransforms[AABBIndex], &m_TargetAABB);
				TargetGeoms[AABBIndex]->visualization(SGNGeometry::Visualization::VISUALIZATION_WIREFRAME);
			}
		}//initEndEffectorMarkers

		void updateEndEffectorMarkers(void) {
			m_IKController.updateEndEffectorValues(&m_EndEffectors);

			for (int32_t i = 0; i < m_EndEffectors.size(); ++i) {
				auto& EffectorTransforms = m_EffectorTransformSGNs.at(m_EndEffectors[i]->segmentName);

				for (int32_t j = 0; j < m_EndEffectors[i]->EndEffectorPoints.cols(); ++j) {
					EffectorTransforms[j]->translation(m_EndEffectors[i]->EndEffectorPoints.col(j));
				}
			}
		}//updateEndEffectorMarkers

		void initTestActors(void) {
			T3DMesh<float> M;
			SAssetIO::load("MyAssets/ccd-ik/JointMarker.gltf", &M);
			setMeshShader(&M, 0.7f, 0.04f);
			M.computePerVertexNormals();
			m_CoordAxes.init(&M);
			M.clear();
		}//initDebugActor

		void initRotationTest(void) {
			
		}//initRotationTest

		void defaultCameraUpdate(VirtualCamera* pCamera, Keyboard* pKeyboard, Mouse* pMouse, const float MovementSpeed = 0.4f, const float RotationSpeed = 1.0f, const float SpeedScale = 4.0f) {
			if (nullptr == pCamera) throw NullpointerExcept("pCamera");
			if (nullptr == pKeyboard) throw NullpointerExcept("pKeyboard");
			if (nullptr == pMouse) throw NullpointerExcept("pMouse");

			float S = 1.0f;
			if (pKeyboard->keyPressed(Keyboard::KEY_LEFT_SHIFT)) S = SpeedScale;

			Vector2f scrollDelta = pMouse->wheel()-m_prevScroll;
			m_prevScroll = pMouse->wheel();

			//TODO(skade) implement proper mouse controls
			//if (pKeyboard->keyPressed(Keyboard::KEY_LEFT_CONTROL)) {
				pCamera->forward(scrollDelta.y());
			//} else {
			//	pCamera->rotY(CForgeMath::degToRad(-10.f * RotationSpeed * scrollDelta.x()));
			//	pCamera->pitch(CForgeMath::degToRad(-10.f * RotationSpeed * scrollDelta.y()));
			//}

			if (pKeyboard->keyPressed(Keyboard::KEY_W)) pCamera->forward(S * MovementSpeed);
			if (pKeyboard->keyPressed(Keyboard::KEY_S)) pCamera->forward(S * -MovementSpeed);
			if (pKeyboard->keyPressed(Keyboard::KEY_A)) pCamera->right(-S * MovementSpeed);
			if (pKeyboard->keyPressed(Keyboard::KEY_D)) pCamera->right(S * MovementSpeed);
			if (pKeyboard->keyPressed(Keyboard::KEY_SPACE)) pCamera->up(S * MovementSpeed);
			if (pKeyboard->keyPressed(Keyboard::KEY_C)) pCamera->up(-S * MovementSpeed);

			if (pMouse->buttonState(Mouse::BTN_RIGHT)) {
				if (m_CameraRotation) {
					const Eigen::Vector2f MouseDelta = pMouse->movement();
					pCamera->rotY(CForgeMath::degToRad(-0.1f * RotationSpeed * MouseDelta.x()));
					pCamera->pitch(CForgeMath::degToRad(-0.1f * RotationSpeed * MouseDelta.y()));
				} else
					m_CameraRotation = true;
				pMouse->movement(Eigen::Vector2f::Zero());
			} else
				m_CameraRotation = false;
		}//defaultCameraUpdate

		SGNTransformation m_RootSGN;

		// character & controller
		IKSkeletalActor m_IKActor;
		InverseKinematicsController m_IKController;
		SGNGeometry m_CharacterSGN;

		IKStickFigureActor m_IKStickActor;
		SGNGeometry m_IKStickActorSGN;

		StaticActor m_JointVisActor;

		SkeletalAnimationController m_SkeletalController;
		SkeletalActor m_SkeletalActor;
		
		// end-effector & target markers
		std::vector<InverseKinematicsController::SkeletalEndEffector*> m_EndEffectors;

		SGNTransformation m_EffectorVis;
		std::map<std::string, std::vector<SGNTransformation*>> m_EffectorTransformSGNs;
		std::map<std::string, std::vector<SGNGeometry*>> m_EffectorGeomSGNs;
		StaticActor m_EffectorPos, m_EffectorX, m_EffectorY, m_EffectorZ;

		SGNTransformation m_TargetVis;
		std::map<std::string, std::vector<SGNTransformation*>> m_TargetTransformSGNs;
		std::map<std::string, std::vector<SGNGeometry*>> m_TargetGeomSGNs;
		StaticActor m_TargetPos, m_TargetX, m_TargetY, m_TargetZ, m_TargetAABB;
		AlignedBox3f m_TargetMarkerAABB;

		//TODO comment
		int32_t m_SelectedEffectorTarget = -1; // index into m_EndEffectors vector; NOT id of joint inside m_CharacterController
		int32_t m_LastSelectedEffectorTarget = -1; // index into m_EndEffectors vector; NOT id of joint inside m_CharacterController

		bool m_LMBDownLastFrame = false;
		Vector3f m_DragStart;

		bool m_IKCupdate = false;
		bool m_useGuizmo = true;
		bool m_renderDebugGrid = true;
		float m_gridSize = 3.f;
		bool m_visualizeJoints = true;
		bool m_showEffector = true;
		bool m_showTarget = true;
		Eigen::Matrix4f m_guizmoMat = Eigen::Matrix4f::Identity();

		Vector2f m_prevScroll = Vector2f::Zero();

		// for debugging / testing
		StaticActor m_CoordAxes;

		IKImGui m_gui;
		Guizmo m_guizmo;
		Config m_config;
	};//MotionRetargetingScene

}//CForge
