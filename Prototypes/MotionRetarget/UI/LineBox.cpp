#include "LineBox.hpp"

#include <crossforge/Graphics/Shader/SShaderManager.h>
#include <crossforge/Math/Box.hpp>
#include <crossforge/Graphics/OpenGLHeader.h>
#include <crossforge/Graphics/RenderDevice.h>
#include <crossforge/Graphics/Shader/GLShader.h>

namespace CForge {

void LineBox::init() {
	// create Shader
	const char* vertexShaderSource = "#version 420 core\n"
		"layout(std140) uniform CameraData {\n"
		"	mat4 ViewMatrix;\n"
		"	mat4 ProjectionMatrix;\n"
		"	vec4 Position;\n"
		"} Camera;\n"
		"layout(std140) uniform ModelData {\n"
		"	mat4 ModelMatrix;\n"
		"};\n"
		"in vec3 VertPosition;\n"
		"void main()\n"
		"{\n"
		"   gl_Position = Camera.ProjectionMatrix * Camera.ViewMatrix * ModelMatrix * vec4(VertPosition.x, VertPosition.y, VertPosition.z, 1.0);\n"
		"}\0";
	const char* fragmentShaderSource = "#version 420 core\n"
		"layout(early_fragment_tests) in;\n"
		"uniform vec4 u_color;\n"
		"out vec4 FragColor;\n"
		"void main()\n"
		"{\n"
		"   FragColor = u_color;\n"
		"}\n\0";

	SShaderManager* shaderManager = SShaderManager::instance();
	uint8_t ConfigOptions = 0;
	ConfigOptions |= ShaderCode::CONF_VERTEXCOLORS;

	ShaderCode* pVertC = shaderManager->createShaderCode(vertexShaderSource, "420", ConfigOptions, "highp");
	ShaderCode* pFragC = shaderManager->createShaderCode(fragmentShaderSource, "420", ConfigOptions, "highp");
	std::vector<ShaderCode*> VSSources;
	std::vector<ShaderCode*> FSSources;
	VSSources.push_back(pVertC);
	FSSources.push_back(pFragC);
	std::string buildShaderError;

	m_AABBshader = shaderManager->buildShader(&VSSources, &FSSources, &buildShaderError);
	//std::cout << buildShaderError << "\n";
	shaderManager->release();
	
	// bind AABB to buffer
	update(Box());
}
void LineBox::update(const Box& aabb) {
	GLfloat vertices[] = { //Triangle Strip Cube
		aabb.min().x(), aabb.min().y(), aabb.min().z(),
		aabb.max().x(), aabb.min().y(), aabb.min().z(),
		aabb.min().x(), aabb.max().y(), aabb.min().z(),
		aabb.max().x(), aabb.max().y(), aabb.min().z(),
		aabb.min().x(), aabb.min().y(), aabb.max().z(),
		aabb.max().x(), aabb.min().y(), aabb.max().z(),
		aabb.max().x(), aabb.max().y(), aabb.max().z(),
		aabb.min().x(), aabb.max().y(), aabb.max().z()
	};
	GLuint elements[] = {
		3,2,6,
		7,4,2,
		0,3,1,
		6,5,4,
		1,0
	};

	m_AABBvertArray.clear();
	m_AABBvertArray.init();
	m_AABBvertArray.bind();

	m_AABBvertBuffer.init(GLBuffer::BTYPE_VERTEX, GLBuffer::BUSAGE_STATIC_DRAW, vertices, 24 * sizeof(GLfloat));
	m_AABBindexBuffer.init(GLBuffer::BTYPE_INDEX, GLBuffer::BUSAGE_STATIC_DRAW, elements, 14 * sizeof(GLuint));
	m_AABBvertBuffer.bind();
	m_AABBindexBuffer.bind();

	glEnableVertexAttribArray(GLShader::attribArrayIndex(GLShader::ATTRIB_POSITION));
	glVertexAttribPointer(GLShader::attribArrayIndex(GLShader::ATTRIB_POSITION), 3, GL_FLOAT, GL_FALSE, 0, nullptr);

	m_AABBvertArray.unbind();
}
void LineBox::render(RenderDevice* pRDev, const Box& b, Matrix4f sgnT) {
	update(b);
	
	// set ogl line rendering mode
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_CULL_FACE);
	glLineWidth(1);

	pRDev->activeShader(m_AABBshader);
	m_AABBvertArray.bind();
	pRDev->modelUBO()->modelMatrix(sgnT);
	glUniform4f(m_AABBshader->uniformLocation("u_color"),color[0],color[1],color[2],color[3]);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDrawElements(GL_TRIANGLE_STRIP, 14, GL_UNSIGNED_INT, nullptr);
	glDisable(GL_BLEND);
	m_AABBvertArray.unbind();

	// disable ogl line rendering
	glEnable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

}//CForge
