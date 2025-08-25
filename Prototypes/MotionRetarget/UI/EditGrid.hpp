#pragma once

#include <crossforge/Graphics/GLBuffer.h>
#include <crossforge/Graphics/GLVertexArray.h>

//TODOfff(skade) LODActor
namespace CForge {
using namespace Eigen;
class Box;
class RenderDevice;
class GLShader;

/**
 * @brief Render an Editor Grid to easier identify Camera Orentation and Axis directions.
 * based on OGLDEV's The Endless Grid tutorial Video: https://www.youtube.com/watch?v=mZorEowBauw
*/
class EditGrid {
public:
	void init();
	void render(RenderDevice* pRDev,float fadeOutDist);
	Vector4f m_colorThick = Vector4f(0.,0.,0.,1.);
	Vector4f m_colorThin = Vector4f(0.075,0.075,0.075,1.);
private:
	GLVertexArray m_vertArray;
	GLBuffer m_vertBuffer;
	GLBuffer m_indexBuffer;
	GLShader* m_shader;
};

}//CForge

