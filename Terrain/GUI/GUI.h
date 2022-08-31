#pragma once

class BaseWidget;
class TextWidget;
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

//for proper wstring to u32string conversion
#include <string>
// #include <locale>   //wstring_convert
// #include <codecvt>  //converter templates used by above function

struct mouseEventInfo {
    Eigen::Vector2f adjustedPosition;   //cursor position adjusted to the position within the clicked widget
    Eigen::Vector2f rawPosition;        //cursor position in window space
};

class CallbackTestClass : public CForge::ITListener<GUICallbackObject> {
    void listen(const GUICallbackObject Msg) override;
};

class GUI : public CForge::ITListener<char32_t>, public CForge::ITListener<CForge::KeyboardCallback> {
public:
    GUI();
    ~GUI();

    void init (CForge::GLWindow* pWin);
    void render(CForge::RenderDevice* renderDevice);

    FormWidget* createOptionsWindow(std::u32string title, int FormID, std::u32string applyName = U"Apply");
    TextWidget* createPlainText();
    TextWidget* createTextWindow(std::u32string title);
    void registerWidgetAsPopup(BaseWidget* widget);
    void unregisterPopup();

    void processEvents();
    void registerMouseDownEvent(BaseWidget* widget);
    void registerMouseDragEvent(BaseWidget* widget);
    void registerKeyPressEvent(BaseWidget* widget);

    void listen(char32_t codepoint) override;
    void listen(CForge::KeyboardCallback kc) override;
    
    //stuff accessed by the widgets
    uint32_t getWindowWidth();
    uint32_t getWindowHeight();
    CForge::GLShader* BackgroundColoredShader;
    CForge::GLShader* TextShader;
    enum FontStyles {       //TODO: remember to keep updated when fonts are added/changed
        DEFAULT_FONT = 0,
        FONT1_REGULAR = 0,
        FONT1_BOLD = 1
    };
    std::vector<FontFace*> fontFaces;
private:
    enum EventType {
        EVENT_CLICK,
        EVENT_DRAG,
        EVENT_KEYPRESS
    };
    void registerEvent(BaseWidget* widget, EventType et);
//     void unregisterEvent(BaseWidget* widget, EventType et);
    void submitTopLevelWidget(BaseWidget* widget);
    void processMouseEvents(CForge::Mouse* mouse);
    void processKeyboardEvents(CForge::Keyboard* keyboard);
    void loadFonts();
    FT_Library library;

    CallbackTestClass callbackTest;

    CForge::GLWindow* m_pWin;
    BaseWidget* focusedWidget = nullptr;
    Eigen::Vector2f focusedClickOffset;

    struct TopLevelWidgetHandler {
        BaseWidget* pWidget;
        std::vector<BaseWidget*> eventsMouseDown;
        std::vector<BaseWidget*> eventsMouseDrag;
        std::vector<BaseWidget*> eventsKeyPress;
        TopLevelWidgetHandler(BaseWidget* w) {
            pWidget = w;
        }
    };
    std::vector<TopLevelWidgetHandler*> m_TopLevelWidgets;
    TopLevelWidgetHandler* m_Popup;
};

std::u32string wstringToU32String(std::wstring ws);
std::wstring u32stringToWString(std::u32string ws);
