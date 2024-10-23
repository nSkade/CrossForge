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
 * @brief Render AABB / Box with gl lines.
*/
class LineBox {
public:
	void init();
	void render(RenderDevice* pRDev, const Box& b, Matrix4f sgnT);
	Vector4f color = Vector4f(227./255,142./255,48./255,.75);
private:
	void update(const Box& aabb);
	GLVertexArray m_AABBvertArray;
	GLBuffer m_AABBvertBuffer;
	GLBuffer m_AABBindexBuffer;
	GLShader* m_AABBshader;
};

}//CForge
