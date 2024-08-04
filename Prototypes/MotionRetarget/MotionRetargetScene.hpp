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

#include "CMN/MRMutil.hpp"
#include "UI/LineBox.hpp"

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
	struct CharEntity;
	void initCharacter(std::weak_ptr<CharEntity> charEntity);
	void initCesiumMan();
	void initEndEffectorMarkers();

	/**
	 * @brief Render Joints and Constraints
	*/
	void renderVisualizers();
	void renderVisualizers(CharEntity* c);

	//TODOf(skade) put in seperate class
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
	void renderUI_ikChainEditor(int* item_current_idx);
	void renderUI_ikTargetEditor();

	/**
	 * @brief loading logic for primary actor
	*/
	void loadCharPrim(std::string path, bool useGLTFIO);
	void storeCharPrim(std::string path, bool useGLTFIO);

	bool keyboardAnyKeyPressed();
	void defaultKeyboardUpdate(Keyboard* pKeyboard);

private:
	//TODO(skade) implement multiple entities
	/**
	 * @brief compacts info regarding single character
	*/
	struct CharEntity : public IPickable {
		std::string name;
		T3DMesh<float> mesh;
		SGNGeometry sgn;
		std::unique_ptr<StaticActor> actorStatic;
		std::unique_ptr<IKSkeletalActor> actor;
		std::unique_ptr<IKController> controller;
		bool visible = true;

		int animIdx = 0;
		int animFrameCurr = 0;
		SkeletalAnimationController::Animation* pAnimCurr = nullptr;

		//TODO(skade) function to apply transformation on mesh data
		void pckMove(const Matrix4f& trans) {
			Vector3f p,s; Quaternionf r;
			MRMutil::deconstructMatrix(trans,&p,&r,&s);
			sgn.position(p);
			sgn.rotation(r);
			sgn.scale(s);
		};
		Matrix4f pckTransGuizmo() {
			return MRMutil::buildTransformation(sgn);
		}; // used for guizmo update
		Matrix4f pckTransPickin() {
			return MRMutil::buildTransformation(sgn);
		}; // used for picking evaluation
		BoundingVolume bv;
		const BoundingVolume& pckBV() {
			bv.init(mesh.aabb());
			return bv; 
		};
	};
	std::vector<std::shared_ptr<CharEntity>> m_charEntities;
	std::weak_ptr<CharEntity> m_charEntityPrim; // currently selected char entity
	std::weak_ptr<CharEntity> m_charEntitySec; // currently selected char entity

	// Anim Gui
	int m_animAutoplay = false;

	SGNTransformation m_sgnRoot;

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

	Guizmo m_guizmo;
	Config m_config;
	EditCamera m_editCam;
	Picker m_picker;
	ViewManipulate m_viewManipulate;

	// preferences //TODO(skade) pair in struct
	bool m_showPopPreferences = false;
	bool m_showPopChainEdit = false;
	bool m_cesStartup = false;
	std::string m_cesStartupStr = "load cesium man on startup";

	LineBox m_lineBox;

};//MotionRetargetScene

}//CForge

