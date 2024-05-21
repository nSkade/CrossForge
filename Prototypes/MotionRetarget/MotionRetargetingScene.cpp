#include "MotionRetargetingScene.hpp"

namespace CForge {

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
	auto ComboStr = [](const char* label, int* current_item, const std::vector<std::string>& items, int items_count, int height_in_items = -1)
	{
		return ImGui::Combo(label, current_item, [](void* data, int idx, const char** out_text) { *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str(); return true; }, (void*)&items, items_count, height_in_items);
	};
	ComboStr("select animation",&m_current_anim_item,items,items.size());

	if (m_current_anim_item > 0) { // anim selected
		if (m_pAnimCurr && m_current_anim_item - 1 != m_pAnimCurr->AnimationID) { // Animation changed
			m_IKController->destroyAnimation(m_pAnimCurr);
			m_IKActor->activeAnimation(nullptr);
			m_IKStickActor.activeAnimation(nullptr);
			m_pAnimCurr = nullptr;
		}
		if (!m_pAnimCurr) { // create animation if not existing
			m_pAnimCurr = m_IKController->createAnimation(m_current_anim_item-1,1.f,0.f);
			m_IKActor->activeAnimation(m_pAnimCurr);
			m_IKStickActor.activeAnimation(m_pAnimCurr);
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
		m_IKStickActor.activeAnimation(nullptr);
		m_pAnimCurr = nullptr;
	}

	//TODOf(skade) StickFigureActor
	//bool sfaEnabled;
	//m_IKStickActorSGN.enabled(nullptr, &sfaEnabled);
	//ImGui::Checkbox("enable StickFigureActor",&sfaEnabled);
	//if (sfaEnabled) {
	//	m_CharacterSGN.enable(true, false);
	//	m_IKStickActorSGN.enable(true, true);
	//} else {
	//	m_CharacterSGN.enable(true, true);
	//	m_IKStickActorSGN.enable(true, false);
	//}
}

void MotionRetargetingScene::renderUI() {
	ImGuiUtility::newFrame();
	ImGuizmo::SetOrthographic(false); //TODO(skade) Ortograhic cam + numpad orientation
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

	{
		ImGui::Begin("IK");
		if (m_LastSelectedEffectorTarget != -1) {
			IKController::SkeletalEndEffector pEndEffector = m_EndEffectors[m_LastSelectedEffectorTarget];
			ImGui::Text("IK segmentName: %s",pEndEffector.segmentName.c_str());
		} else
			ImGui::Text("IK segmentName none");

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

		for (uint32_t i = 0; i < m_IKController->boneCount(); i++) {
			Eigen::Matrix4f loct = CForgeMath::translationMatrix(m_IKController->getBone(i)->LocalPosition);
			Eigen::Matrix4f cubeTransform = m_IKController->getBone(i)->SkinningMatrix
											* m_IKController->getBone(i)->OffsetMatrix.inverse(); // Transform to restpose Space
			float r = 0.1f;
			Eigen::Matrix4f cubeScale = CForgeMath::scaleMatrix(Eigen::Vector3f(r,r,r)*1.0f);
			m_RenderDev.modelUBO()->modelMatrix(cubeTransform*cubeScale);
			m_JointVisActor.render(&m_RenderDev,Eigen::Quaternionf::Identity(),Eigen::Vector3f(),Eigen::Vector3f(1.f,1.f,1.f));
		}
		glDisable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
	}
}

void MotionRetargetingScene::defaultCameraUpdate(VirtualCamera* pCamera, Keyboard* pKeyboard, Mouse* pMouse,
                                                 const float MovementSpeed, const float RotationSpeed, const float SpeedScale) {
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

}//CForge

