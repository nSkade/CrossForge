#pragma once

#include <Examples/exampleSceneBase.hpp>

#include <Prototypes/Assets/GLTFIO/GLTFIO.hpp>
#include <crossforge/Graphics/Actors/SkeletalActor.h>

#include <Prototypes/MotionRetarget/UI/IKImGui.hpp>
#include <Prototypes/MotionRetarget/UI/Guizmo.hpp>
#include <Prototypes/MotionRetarget/UI/EditCamera.hpp>
#include <Prototypes/MotionRetarget/Config/Config.hpp>

#include "IKSkeletalActor.hpp"
#include "IKStickFigureActor.hpp"
#include "UI/Picking.hpp"

namespace CForge {

class MotionRetargetingScene : public ExampleSceneBase {
public:
	MotionRetargetingScene(void) : m_picker(&m_RenderWin,&m_Cam) {
		SLogger::instance()->printToConsole = true;
		m_WindowTitle = "CForge Motion Retarget Editor";
	}//Constructor

	~MotionRetargetingScene(void) {
		clear();
	}//Destructor

	void init() override;
	void clear() override;
	void mainLoop() override;
	void initCameraAndLights(bool CastShadows = true);

	//TODO(skade) put in seperate class
	void renderUI();
	void renderUIAnimation();

private:
	//void rayCast(Eigen::Vector3f* ro, Eigen::Vector3f* rd);
	//TODO(skade) implement target bone
	//void pickTarget();
	//void dragTarget(int target);

	void initCharacter();
	void initEndEffectorMarkers();
	//void updateEndEffectorMarkers();

	/**
	 * @brief Render Joints and Constraints
	*/
	void renderVisualizers();

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
	//std::vector<IKController::SkeletalEndEffector> m_EndEffectors;

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
	//int32_t m_SelectedEffectorTarget = -1; // index into m_EndEffectors vector; NOT id of joint inside m_CharacterController
	//int32_t m_LastSelectedEffectorTarget = -1; // index into m_EndEffectors vector; NOT id of joint inside m_CharacterController

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
	bool m_guizmoViewManipChanged = false;
	Matrix4f m_guizmoMat = Matrix4f::Identity();

	// Anim Gui
	int m_animFrameCurr = 0;
	SkeletalAnimationController::Animation* m_pAnimCurr = nullptr; //TODO(skade) rename
	int m_animAutoplay = false;

	bool keyboardAnyKeyPressed();

	IKImGui m_gui;
	Guizmo m_guizmo;
	Config m_config;
	EditCamera m_editCam;
	Picker m_picker;
};//MotionRetargetingScene

}//CForge

