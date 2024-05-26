#pragma once

namespace CForge {

class VirtualCamera;
class GLWindow;

class EditCamera {
public:
	void defaultCameraUpdate(VirtualCamera* pCamera, GLWindow* pRWin,
	                         const float MovementSpeed = 0.4f, const float RotationSpeed = 1.0f, const float SpeedScale = 4.0f);
private:
	Eigen::Vector2f m_prevScroll = Eigen::Vector2f::Zero();
	bool m_CameraRotation = false;
	bool m_CamIsProj = true;
};

};//CForge

