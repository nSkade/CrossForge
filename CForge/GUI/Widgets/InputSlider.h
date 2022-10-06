/*****************************************************************************\
*                                                                           *
* File(s): InputSlider.h and InputSlider.cpp                                      *
*                                                                           *
* Content:    *
*          .                                         *
*                                                                           *
*                                                                           *
* Author(s): Simon Kretzschmar, Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* FreeBSD License without any warranty or guaranty to work properly.        *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_INPUTSLIDER_H__
#define __CFORGE_INPUTSLIDER_H__

#include "../Widget.h"
#include "../GUI.h"
#include "InputText.h"

namespace CForge {

    class InputSliderWidget;

    class CFORGE_API InputSliderWidget_SliderCursor : public BaseWidget {
    public:
        InputSliderWidget_SliderCursor(GUI* rootGUIObject, BaseWidget* parent);
        ~InputSliderWidget_SliderCursor();
    };

    class CFORGE_API InputSliderWidget_Slider : public BaseWidget {
    public:
        InputSliderWidget_Slider(GUI* rootGUIObject, InputSliderWidget* parent);
        ~InputSliderWidget_Slider();

        void setCursorPosition(float percantage);
        void onClick(mouseEventInfo mouse) override;
        void onDrag(mouseEventInfo mouse) override;

        void changePosition(float dx, float dy) override;
        //     void updateLayout() override;        //we won't change the slider's dimensions
        void draw(CForge::RenderDevice* renderDevice) override;
    private:
        InputSliderWidget* m_slider;
        InputSliderWidget_SliderCursor* m_cursor;
    };

    class CFORGE_API InputSliderWidget_Text : public InputTextWidget {
    public:
        InputSliderWidget_Text(GUI* rootGUIObject, InputSliderWidget* parent);
        ~InputSliderWidget_Text();

        float getFloatValue();
        void setFloatValue(float value);

    private:
        bool validateInput() override;
        float m_floatValue;
        struct {
            float min;
            float max;
        } m_limits;
        InputSliderWidget* m_slider;
    };

    class CFORGE_API InputSliderWidget : public BaseWidget {
    public:
        InputSliderWidget(GUI* rootGUIObject, BaseWidget* parent);
        ~InputSliderWidget();

        float getValue();
        void setValue(float value);
        void setValueByPercentage(float sliderPercantage);
        void setLimit(int lower, int higher);
        void setLimit(float lower, float higher);
        void setStepSize(float stepSize);

        void changePosition(float dx, float dy) override;
        void updateLayout() override;
        void draw(CForge::RenderDevice* renderDevice) override;
    private:
        float m_value;
        struct {
            float min;
            float max;
        } m_limits;
        float m_stepSize;

        InputSliderWidget_Text* m_text;
        InputSliderWidget_Slider* m_slide;
    };

}//name space
#endif