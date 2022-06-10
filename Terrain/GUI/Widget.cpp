#include "Widget.h"

BaseWidget::BaseWidget ( GUI* rootGUIObject, BaseWidget* parent )
{
    //not sure on what needs to be handed over yet
    //The root object is necessary for handling events
    m_root = rootGUIObject;
    //I assume this could be useful for combined Widgets
    m_parent = parent;
    if (parent != nullptr) {
        m_level = parent->getLevel() + 1;
        m_x = parent->getPosition()[0];
        m_y = parent->getPosition()[1];
    }
    else {
        m_level = 0;
        m_x = 0;
        m_y = 0;
    }
    m_width = 20;
    m_height = 20;

    m_background = nullptr;
}
BaseWidget::~BaseWidget()
{
    for (auto x : m_children) {
        delete x;
    }
}

void BaseWidget::setPosition(float x, float y)
{
    //not the most straightforward way to do it, but works better with the derived widgets
    float dx = x - m_x;
    float dy = y - m_y;
    changePosition(dx, dy);
}
void BaseWidget::changePosition(float dx, float dy)
{
    m_x += dx;
    m_y += dy;
    if (m_background != nullptr) m_background->updatePosition();
}
void BaseWidget::updateLayout()
{
    //This method should be called by child widgets if their size or relative position changes
    //(like eg. the number input)
    //since no widget seems to actually use m_children at this point in time,
    //no default default will be provided yet.
    return;
}
void BaseWidget::draw(CForge::RenderDevice* renderDevice)
{
    if (m_background != nullptr) m_background->render(renderDevice);
    for (auto x : m_children) {
        x->draw(renderDevice);
    }
}
int BaseWidget::getLevel()
{
    return m_level;
}
BaseWidget * BaseWidget::getTopWidget()
{
    if (m_parent == nullptr) {
        return this;
    } else {
        return m_parent->getTopWidget();
    }
}
bool BaseWidget::checkHitbox(Eigen::Vector2f pointerPosition)
{
    return m_x <= pointerPosition[0] && m_y <= pointerPosition[1] &&
        m_x+m_width >= pointerPosition[0] && m_y+m_height >= pointerPosition[1];
}

void BaseWidget::focus ( )
{
    return;
}
void BaseWidget::focusLost ( )
{
    return;
}
void BaseWidget::onClick ( mouseEventInfo )
{
    //By default do nothing, have the Widgets overwrite this
    return;
}
void BaseWidget::onDrag ( mouseEventInfo )
{
    return;
}
void BaseWidget::onKeyPress(char32_t)
{
    return;
}

float BaseWidget::getWidth()
{
    return m_width;
}
float BaseWidget::getHeight()
{
    return m_height;
}
Eigen::Vector2f BaseWidget::getPosition()
{
    return Eigen::Vector2f(m_x, m_y);
}
Eigen::Vector2f BaseWidget::getDimension()
{
    return Eigen::Vector2f(m_width, m_height);
}





TextWidget::TextWidget(GUI* rootGUIObject, BaseWidget* parent) : BaseWidget(rootGUIObject, parent)
{
    m_pText = new TextLine;
    m_pText->init(m_root->fontFaces[0], m_root->TextShader);
    WidgetStyle defaults;
    m_padding = defaults.TextPadding;
    m_pText->setPosition(m_padding, m_padding);
    m_height = m_pText->getTextSize() + 2*m_padding;
}
TextWidget::~TextWidget()
{
    delete m_pText;
}
void TextWidget::draw(CForge::RenderDevice* renderDevice)
{
    if (m_background != nullptr) m_background->render(renderDevice);
    m_pText->render(renderDevice);
}
void TextWidget::changeFont(FontFace* newFont)
{
    m_pText->changeFont(newFont);
}
void TextWidget::changeFont(GUI::FontStyles style)
{
    m_pText->changeFont(m_root->fontFaces[style]);
}
void TextWidget::setText(std::u32string textString)
{
    m_text = textString;

    //calculate the width of the string to be rendered
    //this could be done during rendering and passing an information object
    //or something like that, saving one iteration over the string.
    //However, having a seperate function for it has some benefits
    //(eg. it could be expanded to cut-off/break up long strings)
    m_width = m_pText->computeStringWidth(textString) + 2*m_padding;
    if (m_background != nullptr) m_background->updateSize();

    m_pText->setText(textString);
}
std::u32string TextWidget::getText()
{
    return m_text;
}
void TextWidget::changeText(char32_t character)
{
    //can be expanded in the future to support a text cursor
    //once it's clear how the key code to ascii/unicode mapping will happen
    //TODO: for now only takes unicode as input for testing
    switch (character) {
        case 8:     //Backspace: delete last chacter from string
            if (m_text.length() > 0) m_text.pop_back();
            break;
        default:     //treat as normal character
            m_text.push_back(character);
            break;
    }
    setText(m_text);
}
void TextWidget::setPosition(float x, float y)
{
    m_x = x;
    m_y = y;
    m_pText->setPosition(x + m_padding, y + m_padding);
    if (m_background != nullptr) m_background->updatePosition();
}

void TextWidget::changePosition(float dx, float dy)
{
    m_x += dx;
    m_y += dy;
    m_pText->setPosition(m_x + m_padding, m_y + m_padding);
    if (m_background != nullptr) m_background->updatePosition();
}
void TextWidget::setColor(float r, float g, float b)
{
    m_pText->setColor(r, g, b);
}
void TextWidget::setColor(float color[3])
{
    m_pText->setColor(color[0], color[1], color[2]);
}

