#include <GLFW/glfw3.h>
#include "Mouse.h"
#include "../Utility/CForgeUtility.h"

using namespace Eigen;

namespace CForge {

	Mouse::Mouse(void): CForgeObject("Mouse") {
		m_Position = Vector2f(0.0f, 0.0f);
		m_Delta = Vector2f(0.0f, 0.0f);
		m_Wheel = Vector2f(0.0f, 0.0f);
		m_pWin = nullptr;
		m_BtnState.resize(BTN_COUNT, false);
	}//Constructor

	Mouse::~Mouse(void) {
		clear();
	}//Destructor

	void Mouse::init(GLFWwindow* pWin) {
		clear();
		if (nullptr == pWin) throw NullpointerExcept("pWin");
		m_pWin = pWin;
	}//initialize

	void Mouse::clear(void) {
		m_pWin = nullptr;
		m_Position = Vector2f(0.0f, 0.0f);
		m_Delta = Vector2f(0.0f, 0.0f);
		m_Wheel = Vector2f(0.0f, 0.0f);
		for (auto& i : m_BtnState) i = false;
	}//clear

	Eigen::Vector2f Mouse::position(void)const {
		return m_Position;
	}//position

	Eigen::Vector2f Mouse::positionDelta(void) const{
		return m_Delta;
	}//positionDelta

	Eigen::Vector2f Mouse::positionDelta(bool Reset) {
		Eigen::Vector2f Rval = m_Delta;
		if (Reset) m_Delta = Eigen::Vector2f::Zero();
		return Rval;
	}//positionDelta

	Eigen::Vector2f Mouse::wheel(void)const {
		return m_Wheel;
	}//wheel

	bool Mouse::buttonState(Button Btn)const {
		if (Btn <= BTN_UNKNOWN || Btn >= BTN_COUNT) throw IndexOutOfBoundsExcept("Btn");
		return m_BtnState[Btn];
	}//buttonState

	void Mouse::position(Eigen::Vector2f Pos) {
		m_Delta += Pos - m_Position;
		m_Position = Pos;
	}//position

	void Mouse::positionDelta(Eigen::Vector2f Movement) {
		m_Delta = Movement;
	}//movement

	void Mouse::wheel(Eigen::Vector2f Offset) {
		m_Wheel += Offset;
	}//wheel

	void Mouse::buttonState(Button Btn, bool State) {
		if (Btn <= BTN_UNKNOWN || Btn >= BTN_COUNT) throw IndexOutOfBoundsExcept("Btn");
		m_BtnState[Btn] = State;
	}//buttonState

}//name-space