#pragma once

class BaseWidget;
class FormWidget;
//#include "Widget.h"
#include "WidgetBackground.h"
#include "Font.h"
#include <CForge/Graphics/RenderDevice.h>
#include <CForge/Input/Mouse.h>
#include <CForge/Input/Character.h>
#include <vector>

#include "Callback.h"
#include <CForge/Core/ITListener.hpp>

struct mouseEventInfo {
    Eigen::Vector2f adjustedPosition;   //cursor position adjusted to the position within the clicked widget
};

class CallbackTestClass : public CForge::ITListener<CallbackObject> {
    void listen(const CallbackObject Msg) override;
};

class GUI : public CForge::ITListener<char32_t>, public CForge::ITListener<CForge::KeyboardCallback> {
public:
    GUI(CForge::RenderDevice* renderDevice);
    ~GUI();

    void testInit(CForge::GLWindow* pWin);
    void testRender();

    FormWidget* createOptionsWindow(std::u32string title, int FormID);

    void processEvents();
    void registerMouseDownEvent(BaseWidget* widget);
    void registerMouseDragEvent(BaseWidget* widget);
    void registerKeyPressEvent(BaseWidget* widget);

    void listen(char32_t codepoint) override;
    void listen(CForge::KeyboardCallback kc) override;
    
    CForge::GLShader* BackgroundColoredShader;
    CForge::GLShader* TextShader;
    FontFace* fontFace;
private:
    void processMouseEvents(CForge::Mouse* mouse);
    void processKeyboardEvents(CForge::Keyboard* keyboard);

    std::vector<BaseWidget*> testBG;
    CallbackTestClass callbackTest;

    CForge::GLWindow* m_pWin;
    CForge::RenderDevice* m_renderDevice;
    BaseWidget* focusedWidget = nullptr;
    Eigen::Vector2f focusedClickOffset;
    std::vector<BaseWidget*> m_events_mouseDown;
    std::vector<BaseWidget*> m_events_mouseDrag;
    std::vector<BaseWidget*> m_events_keyPress;
};
