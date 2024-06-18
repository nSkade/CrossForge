/*****************************************************************************\
*                                                                           *
* File(s): Mouse.h and Mouse.cpp                                            *
*                                                                           *
* Content: Class dealing with a standard mouse device.                      *
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
#ifndef __CFORGE_MOUSE_H__
#define __CFORGE_MOUSE_H__

#include "../Core/CForgeObject.h"

namespace CForge {

	/**
	* \brief Class that deals with a standard mouse device.
	* \ingroup Input
	* 
	* \todo Add support for messaging system.
	*/
	class CFORGE_API Mouse: public CForgeObject {
	public:
		/**
		* \brief Available buttons.
		*/
		enum Button : int8_t {
			BTN_UNKNOWN = -1,	///< Default value.
			BTN_LEFT = 0,		///< Left mouse button.
			BTN_RIGHT = 1,		///< Right mouse button.
			BTN_MIDDLE = 2,		///< Middle mouse button.
			BTN_4,				///< Mouse button 4.
			BTN_5,				///< Mouse button 5.
			BTN_6,				///< Mouse button 6.
			BTN_7,				///< Mouse button 7.
			BTN_8,				///< Mouse button 8.
			BTN_COUNT,			///< Total number of mouse buttons.
		};

		/**
		* \brief Constructor
		*/
		Mouse(void);

		/**
		* \brief Destructor
		*/
		~Mouse(void);

		/**
		* \brief Initialization method of the mouse. Requires an active window to which the mouse will be bound.
		* 
		* \param[in] pWin Glfw window to which the mouse will be bound and from which it will receive input data.
		*/
		void init(struct GLFWwindow* pWin);

		/**
		* \brief Clear method.
		*/
		void clear(void);

		/**
		* \brief Absolute position data in screen coordinates. Origin in left upper corner.
		* 
		* \return Absolute position data of the mouse.
		*/
		Eigen::Vector2f position(void)const;

		/**
		* \brief Getter for relative mouse movement.
		* 
		* \return Relative mouse movement.
		*/
		Eigen::Vector2f positionDelta()const;

		/**
		* \brief Getter for relative mouse movement.
		* 
		* \param[in] Reset Whether to reset relative movement to (0,0) after read.
		* \return Relative mouse movement.
		*/
		Eigen::Vector2f positionDelta(bool Reset);

		/**
		* \brief Getter for the mouse wheel movement.
		* 
		* \return Mouse wheel movement (x,y).
		*/
		Eigen::Vector2f wheel(void)const;

		/**
		* \brief Getter to check a mouse button's state.
		* 
		* \param[in] Btn Button to check.
		* \return Whether the button is pressed or not.
		*/
		bool buttonState(Button Btn)const;

		/**
		* \brief Setter for the mouse position relative to upper left corner of the window.
		* 
		* \param[in] Pos New mouse position.
		*/
		void position(Eigen::Vector2f Pos);

		/**
		* \brief Setter for the position delta.
		* 
		* \param[in] Delta New mouse delta.
		*/
		void positionDelta(Eigen::Vector2f Delta);

		/**
		* \brief Setter for the wheel status.
		* 
		* \param[in] Offset New (x,y) wheel movement value.
		*/
		void wheel(Eigen::Vector2f Offset);

		/**
		* \brief Setter for the button state.
		* 
		* \param[in] Btn The button that state should be changed.
		* \param[in] State The new state of the button.
		*/
		void buttonState(Button Btn, bool State);

	protected:
		struct GLFWwindow* m_pWin;		///< Glfw window the mouse is associated with.
		Eigen::Vector2f m_Position;		///< Mouse position data. Origin is left upper corner of the window.
		Eigen::Vector2f m_Delta;		///< Position delta.
		Eigen::Vector2f m_Wheel;		///< Wheel movement data.
		std::vector<bool> m_BtnState;	///< Array that tracks state of the mouse buttons.
	};//Mouse

}//name-space

#endif 