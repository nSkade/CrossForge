#include "EditGrid.hpp"

#include <crossforge/Graphics/Shader/SShaderManager.h>
#include <crossforge/Math/Box.hpp>
#include <crossforge/Graphics/OpenGLHeader.h>
#include <crossforge/Graphics/RenderDevice.h>
#include <crossforge/Graphics/Shader/GLShader.h>

namespace CForge {

void EditGrid::init() {
	// create Shader
	//TODOff(skade) move into file
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
		"out vec3 WorldPos;\n"
		"void main()\n"
		"{\n"
		"	vec4 Pos = vec4(VertPosition,1.);\n"
		"	Pos = ModelMatrix * Pos;\n"
		"	Pos.xz += Camera.Position.xz;\n"
		"	gl_Position = Camera.ProjectionMatrix * Camera.ViewMatrix * Pos;\n"
		"	WorldPos = Pos.xyz;\n"
		"}\0";

	//TODOff(skade) move into file
	const char* fragmentShaderSource = "#version 420 core\n"
		"layout(early_fragment_tests) in;\n"
		"layout(std140) uniform CameraData {\n"
		"	mat4 ViewMatrix;\n"
		"	mat4 ProjectionMatrix;\n"
		"	vec4 Position;\n"
		"} Camera;\n"
		"in vec3 WorldPos;\n"
		"out vec4 FragColor;\n"

		"uniform float u_fadeOutDist;\n"
	
		// log10 func
		"float log10(float x) {\n"
		"	return log(x) / log(10.0);\n"
		"}\n"

		"void main()\n"
		"{\n"
		"	float gridCellSize = .1;\n"
		"	float lx = length(vec2(dFdx(WorldPos.x),dFdy(WorldPos.x)));\n"
		"	float ly = length(vec2(dFdx(WorldPos.z),dFdy(WorldPos.z)));\n"
		"	vec2 dudv = vec2(lx,ly)*2.;\n"

		"	float minPixelsBetweenCells = 10.;\n"
		"	float LOD = max(0.,log10(length(dudv)*minPixelsBetweenCells/gridCellSize));\n"
		"	float gridCellSizeLOD = gridCellSize * pow(10.,floor(LOD));\n"

		"	vec2 mod_div_dudv = mod(WorldPos.xz,gridCellSizeLOD)/dudv;\n"
		"	vec2 Lod0a2 = vec2(1.)-abs(mod_div_dudv*2.-1.);\n"
		"	float Lod0a = max(Lod0a2.x,Lod0a2.y);\n"

		"	gridCellSizeLOD *= 10.;\n"
		"	mod_div_dudv = mod(WorldPos.xz,gridCellSizeLOD)/dudv;\n"
		"	vec2 Lod1a2 = vec2(1.)-abs(mod_div_dudv*2.-1.);\n"
		"	float Lod1a = max(Lod1a2.x,Lod1a2.y);\n"

		"	gridCellSizeLOD *= 10.;\n"
		"	mod_div_dudv = mod(WorldPos.xz,gridCellSizeLOD)/dudv;\n"
		"	vec2 Lod2a2 = vec2(1.)-abs(mod_div_dudv*2.-1.);\n"
		"	float Lod2a = max(Lod2a2.x,Lod2a2.y);\n"

		"	vec4 gridColorThick = vec4(vec3(0.),1.);\n"
		"	vec4 gridColorThin = vec4(vec3(0.075),1.);\n"

		// tint basis axis
		"	if (abs(WorldPos.x) < gridCellSize*lx*20.)\n" // z axis
		"		gridColorThick = WorldPos.z < 0. ? vec4(0.,0.,.25,1.) : vec4(0.,0.,.75,1.);\n"
		"	if (abs(WorldPos.z) < gridCellSize*ly*20.)\n" // x axis
		"		gridColorThick = WorldPos.x < 0. ? vec4(.25,0.,0.,1.) : vec4(.75,0.,0.,1.);\n"

		"	vec4 Color;\n"
		"	if (Lod2a > 0.0) {\n"
		"		Color = gridColorThick;\n"
		"		Color.a *= Lod2a;\n"
		"	} else {\n"
		"		if (Lod1a > 0.) {\n"
		"			Color = mix(gridColorThick,gridColorThin,fract(LOD));\n"
		"			Color.a *= Lod1a;\n"
		"		} else {\n"
		"			Color = gridColorThin;\n"
		"			Color.a *= Lod0a * (1.-fract(LOD));\n"
		"		}\n"
		"	}\n"
		"	\n"

		"	float fadeOutScale = 1.;\n"
		"	float dist = length(Camera.Position.xyz-WorldPos);\n"
		"	if (dist > u_fadeOutDist)\n"
		"		Color.a = min(Color.a,1./(1.+(dist-u_fadeOutDist)*fadeOutScale));\n"
		"	FragColor = vec4(Color);\n"
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

	m_shader = shaderManager->buildShader(&VSSources, &FSSources, &buildShaderError);
	shaderManager->release();
	
	// bind AABB to buffer
	GLfloat vertices[] = { //Triangle Strip Cube
		-1,0,-1,
		 1,0,-1,
		 1,0, 1,
		-1,0, 1,
	};
	GLuint elements[] = {
		0,1,2,
		0,2,3,
	};

	m_vertArray.clear();
	m_vertArray.init();
	m_vertArray.bind();

	m_vertBuffer.init(GLBuffer::BTYPE_VERTEX, GLBuffer::BUSAGE_STATIC_DRAW, vertices, 12 * sizeof(GLfloat));
	m_indexBuffer.init(GLBuffer::BTYPE_INDEX, GLBuffer::BUSAGE_STATIC_DRAW, elements, 6 * sizeof(GLuint));
	m_vertBuffer.bind();
	m_indexBuffer.bind();

	glEnableVertexAttribArray(GLShader::attribArrayIndex(GLShader::ATTRIB_POSITION));
	glVertexAttribPointer(GLShader::attribArrayIndex(GLShader::ATTRIB_POSITION), 3, GL_FLOAT, GL_FALSE, 0, nullptr);

	m_vertArray.unbind();
}
void EditGrid::render(RenderDevice* pRDev,float fadeOutDist) {
	
	glDisable(GL_CULL_FACE);

	pRDev->activeShader(m_shader);
	m_vertArray.bind();

	Matrix4f sgnT = Matrix4f::Identity();
	sgnT(0,0) = fadeOutDist*20.*2.;
	sgnT(2,2) = fadeOutDist*20.*2.;
	//sgnT[0] = 100.;
	//sgnT[10] = 100.;
	
	pRDev->modelUBO()->modelMatrix(sgnT);
	glUniform1f(m_shader->uniformLocation("u_fadeOutDist"),fadeOutDist*20.);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
	glDisable(GL_BLEND);
	m_vertArray.unbind();
	glEnable(GL_CULL_FACE);
}

}//CForge
