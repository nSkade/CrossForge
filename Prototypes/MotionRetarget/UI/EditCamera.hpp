#pragma once

namespace CForge {

class VirtualCamera;
class GLWindow;

class EditCamera {
public:
	void defaultCameraUpdate(VirtualCamera* pCamera, GLWindow* pRWin, bool hasControl,
	                         const float MovementSpeed = 0.4f, const float RotationSpeed = 1.0f, const float SpeedScale = 4.0f);
	void setCamProj(VirtualCamera* pCamera, GLWindow* pRWin);
	bool m_CamIsProj = true;
private:
	Eigen::Vector2f m_prevScroll = Eigen::Vector2f::Zero();
	bool m_CameraRotation = false;
};

};//CForge
