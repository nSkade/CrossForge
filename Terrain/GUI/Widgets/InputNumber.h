#pragma once

#include "../Widget.h"
#include "../GUI.h"

struct InputNumberLimits {
    int min;
    int max;
};

class InputNumberWidget;

class InputNumberWidget_DecreaseButton : public TextWidget {
public:
    InputNumberWidget_DecreaseButton(GUI* rootGUIObject, InputNumberWidget* parent);
    void onClick(mouseEventInfo) override;
private:
    InputNumberWidget* m_parentInput;
};

class InputNumberWidget_IncreaseButton : public TextWidget {
public:
    InputNumberWidget_IncreaseButton(GUI* rootGUIObject, InputNumberWidget* parent);
    void onClick(mouseEventInfo) override;
private:
    InputNumberWidget* m_parentInput;
};

class InputNumberWidget : public BaseWidget {
public:
    InputNumberWidget(GUI* rootGUIObject, BaseWidget* parent);
    ~InputNumberWidget();

    void setValue(int value);
    void changeValue(int value);
    int getValue();
    void setLimit(int lower, int higher);
    void focus() override;
    void focusLost() override;
    void onKeyPress(char32_t character) override;

    void changePosition(float dx, float dy) override;
    void updateLayout() override;
    void draw(CForge::RenderDevice* renderDevice) override;
private:
    int m_value;
    InputNumberLimits m_limits;
    InputNumberWidget_DecreaseButton* m_pDec;
    InputNumberWidget_IncreaseButton* m_pInc;
    TextWidget* m_pValue;
    bool m_negativeInput;
};