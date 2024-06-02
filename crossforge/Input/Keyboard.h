/*****************************************************************************\
*                                                                           *
* File(s): Keyboard.h and Keyboard.cpp                                      *
*                                                                           *
* Content: Class for a standard keyboard input device.                      *
*                                                                           *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_KEYBOARD_H__
#define __CFORGE_KEYBOARD_H__

#include "../Core/CForgeObject.h"
#include "../Core/ITCaller.hpp"

namespace CForge {

	struct KeyboardMsg;

	/**
	* \brief This class implements the handling of a default keyboard device. For the time being it is limited to the qwerty (english) layout.
	* \ingroup Input
	* 
	* \todo Try to change default key state to Unknown state, so released can be used properly. Add method to check if released, which is only triggered once when it is released.
	*/
	class CFORGE_API Keyboard: public ITCaller<KeyboardMsg>, public CForgeObject {
	public:

		/**
		* \brief Available keys. Numbers are based off the glfw definitions.
		*/
		enum Key : int16_t {
			KEY_UNKNOWN = -1,	///< Default value.

			KEY_0,// = 48,		///< Numeric key 0.
			KEY_1,// = 49,		///< Numeric key 1.
			KEY_2,// = 50,		///< Numeric key 2.
			KEY_3,// = 51,		///< Numeric key 3.
			KEY_4,// = 52,		///< Numeric key 4.
			KEY_5,// = 53,		///< Numeric key 5.
			KEY_6,// = 54,		///< Numeric key 6.
			KEY_7,// = 55,		///< Numeric key 7.
			KEY_8,// = 56,		///< Numeric key 8.
			KEY_9,// = 57,		///< Numeric key 9.

			KEY_A,// = 97,		///< Letter key a.
			KEY_B,// = 98,		///< Letter key b.
			KEY_C,// = 99,		///< Letter key c.
			KEY_D,// = 100,	///< Letter key d.
			KEY_E,// = 101,	///< Letter key e.
			KEY_F,// = 102,	///< Letter key f.
			KEY_G,// = 103,	///< Letter key g.
			KEY_H,// = 104,	///< Letter key h.
			KEY_I,// = 105,	///< Letter key i.
			KEY_J,// = 106,	///< Letter key j.
			KEY_K,// = 107,	///< Letter key k.
			KEY_L,// = 108,	///< Letter key l.
			KEY_M,// = 109,	///< Letter key m.
			KEY_N,// = 110,	///< Letter key n.
			KEY_O,// = 111,	///< Letter key o.
			KEY_P,// = 112,	///< Letter key p.
			KEY_Q,// = 113,	///< Letter key q.
			KEY_R,// = 114,	///< Letter key r.
			KEY_S,// = 115,	///< Letter key s.
			KEY_T,// = 116,	///< Letter key t.
			KEY_U,// = 117,	///< Letter key u.
			KEY_V,// = 118,	///< Letter key v.
			KEY_W,// = 119,	///< Letter key w.
			KEY_X,// = 120,	///< Letter key x.
			KEY_Y,// = 121,	///< Letter key y.
			KEY_Z,// = 122,	///< Letter key z.
			
			KEY_F1,	///< Function key F1.
			KEY_F2,	///< Function key F2.
			KEY_F3,	///< Function key F3.
			KEY_F4,	///< Function key F4.
			KEY_F5,	///< Function key F5.
			KEY_F6,	///< Function key F6.
			KEY_F7,	///< Function key F7.
			KEY_F8,	///< Function key F8.
			KEY_F9,	///< Function key F9.
			KEY_F10,///< Function key F10.
			KEY_F11,///< Function key F11.
			KEY_F12,///< Function key F12.

			KEY_SPACE,			///< Key space.
			KEY_APOSTROPHE,		///< Key apostrophe.
			KEY_COMMA,			///< Key comma.
			KEY_MINUS,			///< Key minus.
			KEY_PERIOD,			///< Key period.
			KEY_SLASH,			///< Key splash.
			KEY_SEMICOLON,		///< Key semicolon.
			KEY_EQUAL,			///< Key equal.
			KEY_LEFT_BRACKET,	///< Key left bracket.
			KEY_BACKSLASH,		///< Key backslash.
			KEY_RIGHT_BRACKET,	///< Key right bracket.
			KEY_GRAVE_ACCENT,	///< Key grave accent.
			KEY_ESCAPE,			///< Key escape.
			KEY_ENTER,			///< Key enter.
			KEY_TAB,			///< Key tabulator.
			KEY_BACKSPACE,		///< Key backspace.
			KEY_INSERT,			///< Key insert.
			KEY_DELETE,			///< Key delete.
			KEY_RIGHT,			///< Key right arrow.
			KEY_LEFT,			///< Key left arrow.
			KEY_DOWN,			///< Key down arrow.
			KEY_UP,				///< Key up arrow.
			KEY_PAGE_UP,		///< Key page up.
			KEY_PAGE_DOWN,		///< Key page down.
			KEY_HOME,			///< Key home.
			KEY_END,			///< Key end.
			KEY_CAPS_LOCK,		///< Key caps lock.
			KEY_SCROLL_LOCK,	///< Key scroll lock.
			KEY_NUM_LOCK,		///< Key num lock.
			KEY_PRINT_SCREEN,	///< Key print screen.
			KEY_PAUSE,			///< key pause.

			// Keypad
			KEY_KP_0,		///< Keypad key 0.
			KEY_KP_1,		///< Keypad key 1.
			KEY_KP_2,		///< Keypad key 2.
			KEY_KP_3,		///< Keypad key 3.
			KEY_KP_4,		///< Keypad key 4.
			KEY_KP_5,		///< Keypad key 5.
			KEY_KP_6,		///< Keypad Key 6.
			KEY_KP_7,		///< Keypad key 7.
			KEY_KP_8,		///< Keypad key 8.
			KEY_KP_9,		///< Keypad key 9.
			KEY_KP_DECIMAL,	///< Keypad key decimal.
			KEY_KP_DIVIDE,	///< Keypad key divide.
			KEY_KP_MULTIPLY,///< Keypad key multiply.
			KEY_KP_SUBTRACT,///< Keypad key subtract.
			KEY_KP_ADD,		///< Keypad key add.
			KEY_KP_ENTER,	///< Keypad key enter.
			KEY_KP_EQUAL,	///< Keypad key equal.
			
			KEY_LEFT_SHIFT,		///< Keyboard key left shift.
			KEY_LEFT_CONTROL,	///< Keyboard key left control.
			KEY_LEFT_ALT,		///< Keyboard key left alt.
			KEY_RIGHT_SHIFT,	///< Keyboard key right shift.
			KEY_RIGHT_CONTROL,	///< Keyboard key right control.
			KEY_RIGHT_ALT,		///< Keyboard key right alt.

			KEY_MENU,			///< Keyboard key menu.
					
			KEY_COUNT,			///< Number of keys (actually required entries).
		};

		/**
		* \brief Possible state a key can have.
		*/
		enum State: int8_t {
			KEY_RELEASED = 0,	///< Key was released.
			KEY_PRESSED = 1,	///< Key is pressed.
			KEY_REPEATED = 2,	///< Key is repeated (usually when hold for an extended period of time).
		};

		/**
		* \brief Constructor.
		*/
		Keyboard(void);

		/**
		* \brief Destructor
		*/
		~Keyboard(void);

		/**
		* \brief Initialization method. Requires a glfw window from which inputs are received.
		* 
		* \param[in] pWin The glfw window the keyboard is assigned to.
		*/
		void init(struct GLFWwindow *pWin);

		/**
		* \brief Clear method.
		*/
		void clear(void);

		/**
		* \brief Check whether a key is pressed. 
		* 
		* \param[in] K Key to check.
		* \param[in] Reset Whether to reset state to invalid after it was read.
		* \return True if key is pressed, false otherwise.
		*/
		bool keyPressed(Key K, bool Reset);

		/**
		* \brief Check whether a key is pressed.
		* 
		* \param[in] K Key to check.
		* \return True if key is pressed, false otherwise.
		*/
		bool keyPressed(Key K)const;

		/**
		* \brief Check if up to three keys are pressed simultaneously.
		* 
		* \param[in] K1 First key.
		* \param[in] K2 Second key.
		* \param[in] K3 Third key. If KEY_UNKNOWN is specified, the third key will be ignored.
		* \return True if keys K1 through K3 are pressed simultaneously.
		*/
		bool keyPressed(Key K1, Key K2, Key K3 = KEY_UNKNOWN)const;

		/**
		* \brief Key state setter.
		* 
		* \param[in] K Key to set state for.
		* \param[in] S State to set.
		*/
		void keyState(Key K, State S);

		/**
		* \brief Getter for a key state.
		* 
		* \param[in] K Key to check.
		* \return State of the specified key.
		*/
		State keyState(Key K)const;

		/**
		* \brief Allows to recognize and distribute Unicode character input. Specified value will be broadcasted to all listeners.
		* 
		* \param[in] Character Unicode character.
		*/
		void textInput(uint32_t Character);

		
	protected:
		std::vector<State> m_KeyStates;		///< Array that stores state of keys.
		struct GLFWwindow* m_pWin;			///< Associated glfw window.
	};//Keyboard

	typedef std::shared_ptr<Keyboard> KeyboardPtr;

	/**
	* \brief Message that is send by the keyboard object.
	*/
	struct KeyboardMsg {
		Keyboard::Key Key;		///< Key that changed.
		Keyboard::State State;	///< New state of the key.
		uint32_t Unicode;		///< Unicode character sent.
	};

}//name-space

#endif
