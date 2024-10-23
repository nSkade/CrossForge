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
private:
	GLVertexArray m_vertArray;
	GLBuffer m_vertBuffer;
	GLBuffer m_indexBuffer;
	GLShader* m_shader;
};

}//CForge

