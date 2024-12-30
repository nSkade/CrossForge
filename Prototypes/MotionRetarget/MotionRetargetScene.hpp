#pragma once

#include <Examples/exampleSceneBase.hpp>
#include <crossforge/Graphics/Actors/SkeletalActor.h>

//TODOff(skade) move gui outside
#include <Prototypes/MotionRetarget/UI/Guizmo.hpp>
#include <Prototypes/MotionRetarget/UI/EditCamera.hpp>
#include <Prototypes/MotionRetarget/Config/Config.hpp>

#include "CharEntity.hpp"

//#include "IK/IKSkeletalActor.hpp"
//#include "CMN/Picking.hpp"

#include "UI/ViewManipulate.hpp"
#include "UI/LineBox.hpp"
#include "UI/EditGrid.hpp"

#include "AutoMoRe/MRlimb.hpp"

namespace CForge {

/**
 * @class MotionRetargetScene
 * @brief Handles rendering the scene and providing the UI interface to data operations.
*/
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
	void initCharacter(std::weak_ptr<CharEntity> charEntity);
	void initCesiumMan();

	// target visualizer
	//TODOff(skade) dim SPOT IKTarget, move actor into IKTarget, + picked highlight
	void initIKTargetActor();

	/**
	 * @brief Render Joints and Constraints
	*/
	void renderVisualizers();
	void renderVisualizers(CharEntity* c);

	//TODOff(skade) put in seperate class
	// UI functions
	void initUI();
	void cleanUI();
	void renderUI();
	void renderUI_menuBar();
	void renderUI_Outliner();
	IKController::SkeletalJoint* renderUI_OutlinerJoints(std::shared_ptr<CharEntity> c, IKController::SkeletalJoint* selectedJoint);
	void renderUI_animation();
	void renderUI_Sequencer();
	void renderUI_tools();
	void renderUI_ik();
	void renderUI_autorig();
	void renderUI_autoMoRe();
	void renderUI_ikChainEditor(int* item_current_idx);
	void renderUI_ikTargetEditor();

	/**
	 * @brief loading logic for primary actor
	*/
	enum IOmeth {
		IOM_ASSIMP,
		IOM_GLTFIO,
		IOM_OBJIMP,
		IOMCOUNT,
	};
	
	void loadCharPrim(std::string path, IOmeth ioM);
	void storeCharPrim(std::string path, IOmeth ioM);

	bool keyboardAnyKeyPressed();
	void defaultKeyboardUpdate(Keyboard* pKeyboard);

	// helper functions for better structure
	void forcePickCharEntity(std::weak_ptr<CharEntity> c);

private:
	struct settings {
		float gridSize = 3.f;
		bool  renderDebugGrid = true;
		bool  showJoints = true;
		bool  showTargets = true;
		bool  cesStartup = false; // start scene with cesium man on startup
		bool  renderAABB = true; // render line aabb around charEntities when selected
		std::string pathAnaconda = "";
		std::string pathRignet = "";
	} m_settings;

	std::vector<std::shared_ptr<CharEntity>> m_charEntities;
	std::weak_ptr<CharEntity> m_charEntityPrim; // currently selected char entity
	std::weak_ptr<CharEntity> m_charEntitySec; // secondary char entity for operations
	bool m_isEditMode = false; // focuses on one charEntity
	// sgn matrix of charEntity in edit mode for restoration
	Vector3f m_editModeCachePos = Vector3f::Zero();
	Vector3f m_editModeCacheScale = Vector3f::Ones();
	Quaternionf m_editModeCacheRot = Quaternionf::Identity();

	SGNTransformation m_sgnRoot;
	StaticActor m_TargetPos;
	StaticActor m_TargetPosForeign;

	Config m_config;
	EditCamera m_editCam;
	Picker m_picker;
	LineBox m_lineBox;
	EditGrid m_editGrid;

	Guizmo m_guizmo;
	ViewManipulate m_viewManipulate;

	// internal variables
	// input
	bool m_LMBDownLastFrame = false;
	bool m_exitCalled = false;
	// cam
	bool m_guizmoViewManipChanged = false;
	Matrix4f m_guizmoMat = Matrix4f::Identity();
	// outliner
	IKController::SkeletalJoint* m_outlinerSelJoint = nullptr;
	// ik edit
	int m_selChainIdx = -1;
	int m_selChainIdxPrev = -1;
	// ik chain edit
	bool m_ikceNameInit = false;
	std::string m_ikceName = "new";
	IKController::SkeletalJoint* m_ikceRootJoint = nullptr;
	IKController::SkeletalJoint* m_ikceEndEffJoint = nullptr;
	// motion retarget TODO(skade) organize better
	MRlimb m_MRlimb;
	// gui popup
	enum AppPopups {
		POP_PREF = 0,
		POP_CHAINED,
		POP_AR_PINOC,
		POP_AR_RIGNET,
		POP_MR_LIMB,
		POP_COUNT,
	};
	std::vector<bool> m_showPop = std::vector<bool>(POP_COUNT,false);

	// consts
	const std::string m_cesStartupStr = "load cesium man on startup";
};//MotionRetargetScene

}//CForge
