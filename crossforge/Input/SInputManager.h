/*****************************************************************************\
*                                                                           *
* File(s): SInputManager.h and SInputManager.cpp                            * 
*                                                                           *
* Content: Singleton class handling initialization and management of        *
*          input devices.                                                   *
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
#ifndef __CFORGE_SINPUTMANAGER_H__
#define __CFORGE_SINPUTMANAGER_H__

#include "../Core/CForgeObject.h"
#include "Keyboard.h"
#include "Mouse.h"

namespace CForge {
	/**
	* \brief Singleton class that handles input devices, i.e. initialization of devices as well as receiving and distribution of system messages related to the I/O devices.
	* \ingroup Input
	* 
	* \todo Write documentation concept page about input devices.
	* \todo Change keyboard and mouse registration to shared pointers as well as glfwWindow pointer.
	* \todo Add glfw joystick support: https://www.glfw.org/docs/3.3/input_guide.html
	*/
	class CFORGE_API SInputManager: public CForgeObject {
	public:
		/**
		* \brief Singleton instantiation method.
		* 
		* \return Pointer to the singleton instance.
		*/
		static SInputManager* instance(void);

		/**
		* \brief Singleton release method.
		*/
		void release(void);

		/**
		* \brief Returns number of active instances.
		*/
		static int32_t instanceCount();

		/**
		* \brief Registers a new keyboard device making it eligible for data handling.
		* 
		* \param[in] pWin Glfw window associated with the keyboard.
		* \param[in] pKeyboard Instance of the keyboard.
		*/
		void registerDevice(struct GLFWwindow *pWin, Keyboard* pKeyboard);

		/**
		* \brief Registers a new mouse device making it eligible for data handling.
		* 
		* \param[in] pWin Glfw window associated with the keyboard.
		* \param[in] pMouse Instance of the mouse.
		*/
		void registerDevice(struct GLFWwindow* pWin, Mouse* pMouse);

		/**
		* \brief Unregisters a keyboard device.
		* 
		* \param[in] pKeyboard Keyboard to remove.
		*/
		void unregisterDevice(Keyboard* pKeyboard);

		/**
		* \brief Unregisters a mouse device.
		* 
		* \param[in] pMouse Mouse device to remove.
		*/
		void unregisterDevice(Mouse* pMouse);

	protected:
		/**
		* \brief Constructor
		*/
		SInputManager(void);

		/**
		* \brief Destructor
		*/
		~SInputManager(void);

		/**
		* \brief Initialization method.
		*/
		void init(void);

		/**
		* \brief Clear method.
		*/
		void clear(void);
	private:
		static SInputManager* m_pInstance;	///< Pointer of the unique instance.
		static uint32_t m_InstanceCount;	///< Number of active instances.

		/**
		* \brief Keyboard callback method for glfw.
		* 
		* \param[in] pWin Associated glfw window, i.e. the window sending the message.
		* \param[in] Key The glfw key.
		* \param[in] Scancode Platform specific key code. Can probably be used to use special keys on certain keyboards.
		* \param[in] Action The action reported for the key (pressed, released, etc.)
		* \param[in] Mods Set of active mods probably. Documentation is not very helpful on that regard.
		* 
		*/
		static void keyboardCallbackFunc(struct GLFWwindow* pWin, int Key, int Scancode, int Action, int Mods);

		/**
		* \brief Character callback method for glfw.
		* 
		* \param[in] pWin Associated glfw window, i.e. the window sending the message.
		* \param[in] Codepoint Character code.
		*/
		static void characterCallbackFunc(struct GLFWwindow* pWin, unsigned int Codepoint);

		/**
		* \brief Mouse position callback method for glfw.
		* 
		* \param[in] pWin Associated glfw window, i.e. window sending the message.
		* \param[in] xPos Position in x direction.
		* \param[in] yPos Position in y direction.
		*/
		static void mousePositionCallbackFunc(struct GLFWwindow* pWin, double xPos, double yPos);

		/**
		* \brief Mouse button callback method for glfw.
		* 
		* \param[in] pWin Associated glfw window, i.e. window sending the message.
		* \param[in] Button The button identifier.
		* \param[in] Action The action performed (press/release).
		* \param[in] Mode Purpose of this parameter is unclear.
		*/
		static void mouseButtonCallbackFunc(struct GLFWwindow* pWin, int Button, int Action, int Mode);

		/**
		* \brief Mouse wheel callback method for glfw.
		* 
		* \param[in] pWin Associated glfw window, i.e. window sending the message.
		* \param[in] xOffset Offset in x direction (primary direction).
		* \param[in] yOffset Offset in y direction.
		*/
		static void mouseWheelCallbackFunc(struct GLFWwindow* pWin, double xOffset, double yOffset);

		/**
		* \brief Structure to store a keyboard entity internally.
		*/
		struct KeyboardEntity {
			Keyboard* pKeyboard;		///< Keyboard instance.
			struct GLFWwindow* pWin;	///< Associated glfw window.

			/**
			* \brief Constructor.
			*/
			KeyboardEntity() {
				pKeyboard = nullptr;
				pWin = nullptr;
			}

			/**
			* \brief Destructor
			*/
			~KeyboardEntity() {
				pKeyboard = nullptr;
				pWin = nullptr;
			}
		};

		/**
		* \brief Structure to store a mouse entity internally.
		*/
		struct MouseEntity {
			Mouse* pMouse;				///< Mouse instance.
			struct GLFWwindow* pWin;	///< Associated window.

			/**
			* \brief Constructor
			*/
			MouseEntity() {
				pMouse = nullptr;
				pWin = nullptr;
			}

			/**
			* \brief Destructor
			*/
			~MouseEntity() {
				pMouse = nullptr;
				pWin = nullptr;
			}
		};
		

		std::vector<std::shared_ptr<KeyboardEntity>> m_RegisteredKeyboards;	///< List of registered keyboards.
		std::vector<std::shared_ptr<MouseEntity>> m_RegisteredMice;			///< List of registered mice.

	};//SInputManager

}//name-space


#endif 
