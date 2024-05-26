#include "EditCamera.hpp"
#include <crossforge/Graphics/GLWindow.h>
#include <crossforge/Input/Keyboard.h>
#include <crossforge/Input/Mouse.h>
#include <crossforge/Graphics/Camera/VirtualCamera.h>
#include <crossforge/Math/CForgeMath.h>

namespace CForge {

void EditCamera::defaultCameraUpdate(VirtualCamera* pCamera, GLWindow* pRWin,
                                                 const float MovementSpeed, const float RotationSpeed, const float SpeedScale) {

	if (nullptr == pCamera) throw NullpointerExcept("pCamera");
	if (nullptr == pRWin) throw NullpointerExcept("pKeyboard");

	using namespace Eigen;

	Mouse* pMouse = pRWin->mouse();
	Keyboard* pKeyboard = pRWin->keyboard();

	float S = 1.0f;
	if (pKeyboard->keyPressed(Keyboard::KEY_LEFT_SHIFT)) S = SpeedScale;

	Vector2f scrollDelta = pMouse->wheel()-m_prevScroll;
	m_prevScroll = pMouse->wheel();

	//TODO(skade) implement proper mouse controls
	//if (pKeyboard->keyPressed(Keyboard::KEY_LEFT_CONTROL)) {
	float rwWidth = pRWin->width();
	float rwHeight = pRWin->height();
	float aspect = rwWidth/rwHeight;
	if (scrollDelta.y() != 0. || pKeyboard->keyPressed(Keyboard::KEY_W) || pKeyboard->keyPressed(Keyboard::KEY_S)) {
		pCamera->forward(scrollDelta.y());
		if (!m_CamIsProj) {
			float fov = 45.;
			float dist = std::abs(pCamera->position().norm());
			pCamera->orthographicProjection(-aspect*dist*.5, // left
											aspect*dist*.5, // right
											-1.*dist*.5, // bottom
											1.*dist*.5, // top
											.01, // near
											1000.); // far
		}
	}
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
			const Eigen::Vector2f MouseDelta = pMouse->positionDelta();
			pCamera->rotY(CForgeMath::degToRad(-0.1f * RotationSpeed * MouseDelta.x()));
			pCamera->pitch(CForgeMath::degToRad(-0.1f * RotationSpeed * MouseDelta.y()));
		} else
			m_CameraRotation = true;
		pMouse->positionDelta(Eigen::Vector2f::Zero());
	} else
		m_CameraRotation = false;
	
	// numpad camera movement
	if (pKeyboard->keyPressed(Keyboard::KEY_KP_5, true)) {
		//TODO(skade) abstract fov
		float fov = 45.;
		float dist = std::abs(pCamera->position().norm());
		if (m_CamIsProj)
			pCamera->orthographicProjection(-aspect*dist*.5, // left
			                                aspect*dist*.5, // right
			                                -1.*dist*.5, // bottom
			                                1.*dist*.5, // top
			                                .01, // near
			                                1000.); // far
		else
			pCamera->projectionMatrix(rwWidth,rwHeight, CForgeMath::degToRad(fov), 0.1f, 1000.0f);
		m_CamIsProj = !m_CamIsProj;
		ImGuizmo::SetOrthographic(!m_CamIsProj);
	}

	//TODO(skade) move into function
	auto funcNPO = [&](Keyboard::Key key, float angle, Vector3f axis, bool add = false) {
		if (pKeyboard->keyPressed(key,true)) {
			pCamera->orthographicProjection(-aspect,aspect,-1.,1.,.01,1000.);
			Eigen::Matrix4f view = pCamera->cameraMatrix();
			if (!add)
				view.block<3,3>(0,0) = CForgeMath::rotationMatrix(angle,axis).block<3,3>(0,0);
			else
				view = CForgeMath::rotationMatrix(angle,axis)*view;
			pCamera->cameraMatrix(view);
			m_CamIsProj = false;
			ImGuizmo::SetOrthographic(true);
		}
	};
	funcNPO(Keyboard::KEY_KP_1,90.,Vector3f::UnitY());
	funcNPO(Keyboard::KEY_KP_3,0.,Vector3f::UnitX());
	funcNPO(Keyboard::KEY_KP_7,90.,Vector3f::UnitX());

	funcNPO(Keyboard::KEY_KP_2,-15.,pCamera->right(),true);
	funcNPO(Keyboard::KEY_KP_4,15.,Vector3f::UnitY(),true);
	funcNPO(Keyboard::KEY_KP_6,-15.,Vector3f::UnitY(),true);
	funcNPO(Keyboard::KEY_KP_8,15.,pCamera->right(),true);
	if (pKeyboard->keyPressed(Keyboard::KEY_KP_9,true)) {
		//TODO(skade) mirror position?
		pCamera->rotY(CForgeMath::degToRad(180.));
	}
}//defaultCameraUpdate

}//CForge
