#pragma once

#include <Examples/exampleSceneBase.hpp>
#include <crossforge/Graphics/Actors/SkeletalActor.h>

//TODO(skade) move gui outside
#include <Prototypes/MotionRetarget/UI/Guizmo.hpp>
#include <Prototypes/MotionRetarget/UI/EditCamera.hpp>
#include <Prototypes/MotionRetarget/Config/Config.hpp>

#include "IKSkeletalActor.hpp"
#include "Animation/Picking.hpp"
#include "UI/ViewManipulate.hpp"

namespace CForge {

class MotionRetargetScene : public ExampleSceneBase {
public:
	MotionRetargetScene(void) : m_picker(&m_RenderWin,&m_Cam) {
		SLogger::instance()->printToConsole = true;
		m_WindowTitle = "CForge Motion Retarget";
	}//Constructor

	~MotionRetargetScene(void) {
		clear();
	}//Destructor

	void init() override;
	void clear() override;
	void mainLoop() override;
	void initCameraAndLights(bool CastShadows = true);

private:
	void initCharacter();
	void initCesiumMan();
	void initEndEffectorMarkers();

	/**
	 * @brief Render Joints and Constraints
	*/
	void renderVisualizers();

	//TODOf(skade) put in seperate class
	void initUI();
	void cleanUI();
	void renderUI();
	void renderUI_menuBar();
	void renderUI_animation();
	void renderUI_Sequencer();
	void renderUI_tools();
	void renderUI_ik();

	/**
	 * @brief loading logic for primary actor
	*/
	void loadCharPrim(std::string path);
	void storeCharPrim(std::string path);

	bool keyboardAnyKeyPressed();
	void defaultKeyboardUpdate(Keyboard* pKeyboard);

private:
	//TODO(skade) implement multiple entities
	///**
	// * @brief compacts info regarding single character
	//*/
	//struct CharEntity {
	//	IKSkeletalActor actor;
	//	IKController controller;
	//	T3DMesh<float> mesh;
	//	SGNGeometry sgnGeo;
	//	SGNTransformation sgnTrans;
	//	//TODO(skade) function to apply transformation on mesh data
	//};
	//std::vector<CharEntity> m_charEntities;

	SGNTransformation m_sgnRoot;

	// primary character & controller
	std::unique_ptr<StaticActor> m_StaticActorPrim; //TODO(skade)
	std::unique_ptr<IKSkeletalActor> m_IKActorPrim;
	std::unique_ptr<IKController> m_IKControllerPrim;
	std::unique_ptr<T3DMesh<float>> m_MeshCharPrim; // primary character Mesh
	SGNGeometry m_sgnCharPrim;

	// secondary character //TODO(skade)

	int m_current_anim_item = 0;

	std::map<std::string, std::vector<SGNTransformation*>> m_EffectorTransformSGNs;
	std::map<std::string, std::vector<SGNGeometry*>> m_EffectorGeomSGNs;
	StaticActor m_EffectorPos, m_EffectorX, m_EffectorY, m_EffectorZ;
	
	std::map<std::string, std::vector<SGNTransformation*>> m_TargetTransformSGNs;
	std::map<std::string, std::vector<SGNGeometry*>> m_TargetGeomSGNs;

	StaticActor m_TargetPos, m_TargetAABB;
	AlignedBox3f m_TargetMarkerAABB;

	bool m_LMBDownLastFrame = false;
	bool m_exitCalled = false;

	//TODO(skade) sort & put in options struct
	bool m_IKCupdate = false;
	bool m_IKCupdateSingle = false;
	bool m_renderDebugGrid = true;
	float m_gridSize = 3.f;
	bool m_showJoints = true;
	bool m_showTarget = true;
	bool m_guizmoViewManipChanged = false;
	Matrix4f m_guizmoMat = Matrix4f::Identity();

	// Anim Gui
	int m_animFrameCurr = 0;
	SkeletalAnimationController::Animation* m_pAnimCurr = nullptr; //TODO(skade) rename
	int m_animAutoplay = false;

	Guizmo m_guizmo;
	Config m_config;
	EditCamera m_editCam;
	Picker m_picker;
	ViewManipulate m_viewManipulate;
};//MotionRetargetScene

}//CForge

