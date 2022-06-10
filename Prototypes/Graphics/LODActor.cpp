﻿#include <glad/glad.h>
#include "LODActor.h"
#include "../../CForge/Graphics/RenderDevice.h"
#include "../../CForge/Graphics/GraphicsUtility.h"
#include "../../CForge/Core/SLogger.h"
#include "../MeshDecimate.h"
#include "../../CForge/Graphics/Shader/SShaderManager.h"

//#define SKIP_INSTANCED_QUERIES // TODO determine skipping by time query

namespace CForge {
	LODActor::LODActor(void) : IRenderableActor("LODActor", ATYPE_STATIC) {
		m_TypeID = ATYPE_STATIC;
		m_TypeName = "LODActor";
		m_pSLOD = SLOD::instance();
	}//Constructor

	LODActor::~LODActor(void) {
		clear();
		m_pSLOD->release();
		for (uint32_t i = 0; i < m_instancedMatRef.size(); i++) {
			delete m_instancedMatRef[i];
		}
	}//Destructor
	
	void LODActor::init(T3DMesh<float>* pMesh, bool isInstanced, bool manualyInstanced) {
		if (nullptr == pMesh) throw NullpointerExcept("pMesh");
		if (pMesh->vertexCount() == 0) throw CForgeExcept("Mesh contains no vertex data");
		
		m_VertexArray.init();
		//m_translucent = isTranslucent;
		m_isInstanced = isInstanced || manualyInstanced;
		m_isManualInstaned = manualyInstanced;
		
		// fetch LOD levels from SLOD if not given
		if (m_LODStages.empty())
			m_LODStages = std::vector<float>(*m_pSLOD->getLevels());

		m_LODMeshes.push_back(pMesh);
		uint16_t VertexProperties = 0;
		if (pMesh->vertexCount() > 0) VertexProperties |= VertexUtility::VPROP_POSITION;
		if (pMesh->normalCount() > 0) VertexProperties |= VertexUtility::VPROP_NORMAL;
		if (pMesh->tangentCount() > 0) VertexProperties |= VertexUtility::VPROP_TANGENT;
		if (pMesh->textureCoordinatesCount() > 0) VertexProperties |= VertexUtility::VPROP_UVW;
		if (pMesh->colorCount() > 0) VertexProperties |= VertexUtility::VPROP_COLOR;
		m_VertexProperties.push_back(VertexProperties);
		m_VertexUtility.init(VertexProperties);
		initiateBuffers(0);
		bindLODLevel(0);
		
		std::vector<Eigen::Matrix4f>* m_instLODMats = new std::vector<Eigen::Matrix4f>();
		m_instancedMatRef.push_back(m_instLODMats);
		
		// TODO parallelise generation
		for (uint32_t i = 1; i < m_LODStages.size(); i++) {
			T3DMesh<float>* pLODMesh = new T3DMesh<float>();
			float amount = float(m_LODStages[i]) / m_LODStages[i-1];
			if (amount > 1.0)
				throw CForgeExcept("decimation stages are in wrong order");
			
			MeshDecimator::decimateMesh(m_LODMeshes[i-1], pLODMesh, amount);
			m_LODMeshes.push_back(pLODMesh);
			initiateBuffers(i);
			
			m_instLODMats = new std::vector<Eigen::Matrix4f>();
			m_instancedMatRef.push_back(m_instLODMats);
		}
		
		m_LODMeshes[0]->computeAxisAlignedBoundingBox();
		m_aabb = m_LODMeshes[0]->aabb();
		initAABB();
	}//initialize

	void LODActor::init(T3DMesh<float>* pMesh, const std::vector<float>& LODStages, bool isInstanced, bool manualyInstanced) {
		m_LODStages = LODStages;
		init(pMesh, isInstanced, manualyInstanced);
	}//initialize

	void LODActor::initiateBuffers(uint32_t level) {
		auto pMesh = m_LODMeshes[level];

		try {
			uint8_t* pBuffer = nullptr;
			uint32_t BufferSize = 0;
			
			m_VertexUtility.buildBuffer(pMesh->vertexCount(), (void**)&pBuffer, &BufferSize, pMesh);
						
			m_VertexBuffers.push_back(pBuffer);
			m_VertexBufferSizes.push_back(BufferSize);
			}
		catch (CrossForgeException& e) {
			SLogger::logException(e);
			return;
		}
		catch (...) {
			SLogger::log("Unknown exception occurred during vertex buffer creation!");
			return;
		}

		// build render groups and element array
		try {
			uint8_t* pBuffer = nullptr;
			uint32_t BufferSize = 0;
			RenderGroupUtility* RGU = new RenderGroupUtility();
			RGU->init(pMesh, (void**)&pBuffer, &BufferSize);
			m_RenderGroupUtilities.push_back(RGU);
			
			m_ElementBuffers.push_back(pBuffer);
			m_ElementBufferSizes.push_back(BufferSize);
		}
		catch (CrossForgeException& e) {
			SLogger::logException(e);
			return;
		}
		catch (...) {
			SLogger::log("Unknown exception occurred during building of index buffer!");
			return;
		}
	}

	void LODActor::bindLODLevel(uint32_t level) {

		m_VertexArray.unbind();
		try {
			m_VertexBuffer.init(GLBuffer::BTYPE_VERTEX, GLBuffer::BUSAGE_STATIC_DRAW, m_VertexBuffers[level], m_VertexBufferSizes[level]);
		}
		catch (CrossForgeException& e) {
			SLogger::logException(e);
			return;
		}
		catch (...) {
			SLogger::log("Unknown exception occurred during vertex buffer creation!");
			return;
		}

		// build render groups and element array
		try {
			m_ElementBuffer.init(GLBuffer::BTYPE_INDEX, GLBuffer::BUSAGE_STATIC_DRAW, m_ElementBuffers[level], m_ElementBufferSizes[level]);
		}
		catch (CrossForgeException& e) {
			SLogger::logException(e);
			return;
		}
		catch (...) {
			SLogger::log("Unknown exception occurred during building of index buffer!");
			return;
		}
		
		//m_VertexArray.clear();
		m_VertexArray.bind();
		setBufferData();
		m_VertexArray.unbind();
		
		m_LODLevel = level;
	}

	uint32_t LODActor::getLODLevel() {
		return m_LODLevel;
	}
	
	void LODActor::clear(void) {
		m_VertexBuffer.clear();
		m_ElementBuffer.clear();
		m_VertexArray.clear();

		m_VertexUtility.clear();
		m_RenderGroupUtility.clear();

		for (uint32_t i = 0; i < m_LODMeshes.size(); i++) {
			if (i > 0) // do not delete the first mesh, we do not own its reference
				delete m_LODMeshes[i];

			delete m_RenderGroupUtilities[i];
			if (nullptr != m_VertexBuffers[i])
				delete[] m_VertexBuffers[i];
			m_VertexBuffers[i] = nullptr;
			if (nullptr != m_ElementBuffers[i])
				delete[] m_ElementBuffers[i];
			m_ElementBuffers[i] = nullptr;	
		}
	}//Clear

	void LODActor::release(void) {
		delete this;
	}//release

	uint32_t LODActor::materialCount(void) const {
		return m_RenderGroupUtilities[0]->renderGroupCount();
	}//materialCount

	RenderMaterial* LODActor::material(uint32_t Index) {
		if (Index >= m_RenderGroupUtilities[0]->renderGroupCount()) throw IndexOutOfBoundsExcept("Index");
		return &(m_RenderGroupUtilities[0]->renderGroups()[Index]->Material);
	}
	
	void LODActor::render(RenderDevice* pRDev) {
		if (nullptr == pRDev) throw NullpointerExcept("pRDev");
		
		if (m_isInstanced) {
			for (uint32_t j = 0; j < m_instancedMatRef.size(); j++) {
				if (m_instancedMatRef[j]->size() > 0) {
					
					bindLODLevel(j);
					m_VertexArray.bind();
					uint32_t maxInstances = pRDev->getInstancedUBO()->getMaxInstanceCount();
					
					for (uint32_t k = 0; k < m_instancedMatRef[j]->size(); k += maxInstances) {
						
						uint32_t range = std::min((k + maxInstances), (uint32_t) m_instancedMatRef[j]->size());
						pRDev->getInstancedUBO()->setInstances(m_instancedMatRef[j], Eigen::Vector2i(k, range));
						
						for (auto i : m_RenderGroupUtilities[m_LODLevel]->renderGroups()) {
							if (i->pShader == nullptr) continue;
							if (pRDev->activePass() == RenderDevice::RENDERPASS_SHADOW) {
								pRDev->activeShader(pRDev->shadowPassShader());
							}
							else {
								pRDev->activeShader(i->pShader);
								pRDev->activeMaterial(&i->Material);
							}
							glDrawElementsInstanced(GL_TRIANGLES, i->Range.y() - i->Range.x(), GL_UNSIGNED_INT, (const void*)(i->Range.x() * sizeof(unsigned int)), range-k);
						} //for [render groups]
					} //for [maxInstances]
				} //if [instances exist]
				m_instancedMatRef[j]->clear();
			}
			if (!m_isManualInstaned)
				m_instancedMatrices.clear();
		} else {
			m_VertexArray.bind();
			for (auto i : m_RenderGroupUtilities[m_LODLevel]->renderGroups()) {
				if (i->pShader == nullptr) continue;
				if (pRDev->activePass() == RenderDevice::RENDERPASS_SHADOW) {
					pRDev->activeShader(pRDev->shadowPassShader());
				}
				else {
					pRDev->activeShader(i->pShader);
					pRDev->activeMaterial(&i->Material);
				}
				glDrawRangeElements(GL_TRIANGLES, 0, m_ElementBuffer.size() / sizeof(unsigned int), i->Range.y() - i->Range.x(), GL_UNSIGNED_INT, (const void*)(i->Range.x() * sizeof(unsigned int)));
			}//for[all render groups]
		}
	}//render
	
	void LODActor::initAABB() {
		
		// create Shader
		const char* vertexShaderSource = "#version 330 core\n"
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
		const char* fragmentShaderSource = "#version 330 core\n"
			"layout(early_fragment_tests) in;\n"
			"out vec4 FragColor;\n"
			"void main()\n"
			"{\n"
			"   FragColor = vec4(1.0);\n"
			"}\n\0";

		SShaderManager* shaderManager = SShaderManager::instance();
		uint8_t ConfigOptions = 0;
		ConfigOptions |= ShaderCode::CONF_VERTEXCOLORS;

		ShaderCode* pVertC = shaderManager->createShaderCode(vertexShaderSource, "330", ConfigOptions, "highp", "highp");
		ShaderCode* pFragC = shaderManager->createShaderCode(fragmentShaderSource, "330", ConfigOptions, "highp", "highp");
		std::vector<ShaderCode*> VSSources;
		std::vector<ShaderCode*> FSSources;
		VSSources.push_back(pVertC);
		FSSources.push_back(pFragC);
		std::string buildShaderError;

		m_AABBshader = shaderManager->buildShader(&VSSources, &FSSources, &buildShaderError);
		//std::cout << buildShaderError << "\n";
		shaderManager->release();
		
		// bind AABB to buffer
		updateAABB();
	}
	
	void LODActor::testAABBvis(class RenderDevice* pRDev, Eigen::Matrix4f sgMat) {
		
		if (m_isManualInstaned) { // all instanced at once
			for (uint32_t i = 0; i < m_instancedMatrices.size(); i++) {

#ifndef SKIP_INSTANCED_QUERIES
				if (!fovCulling(pRDev, &m_instancedMatrices[i]))
					continue;
				queryAABB(pRDev, m_instancedMatrices[i]);
#else
				m_instancedMatRef[0]->push_back(m_instancedMatrices[i]);
				if (!this->isInLODSG()) {
					pRDev->LODSGPushBack(this, Eigen::Matrix4f::Identity());
					this->setLODSG(true);
				}
#endif
			}
		}
		else if (m_isInstanced) { // single instance, other instances get added over SG

#ifndef SKIP_INSTANCED_QUERIES
			if (!fovCulling(pRDev, &sgMat))
				return;
			queryAABB(pRDev, sgMat);
#else
			m_instancedMatrices.push_back(sgMat);
			m_instancedMatRef[0]->push_back(m_instancedMatrices.at(m_instancedMatrices.size()-1));
			if (!this->isInLODSG()) {
				pRDev->LODSGPushBack(this, Eigen::Matrix4f::Identity());
				this->setLODSG(true);
			}
#endif
		}
		else {
			queryAABB(pRDev, sgMat);
		}
	}
	
	void LODActor::queryAABB(RenderDevice* pRDev, Eigen::Matrix4f transform) {
		GLuint queryID;
		glGenQueries(1, &queryID);
		glBeginQuery(GL_SAMPLES_PASSED, queryID);

		if (!glIsQuery(queryID)) {
			// fetch current queries if no more are available
			pRDev->fetchQueryResults();
			glGenQueries(1, &queryID);
			glBeginQuery(GL_SAMPLES_PASSED, queryID);
		}

		//if (!glIsQuery(queryID))
		//	CForgeExcept("could not generate gl query");
		
		pRDev->LODQueryContainerPushBack(queryID, this, transform);
	
		renderAABB(pRDev);
		glEndQuery(GL_SAMPLES_PASSED);
	}
	
	void LODActor::evaluateQueryResult(Eigen::Matrix4f mat, uint32_t pixelCount) {
		int32_t level = 0;
		
		if (pixelCount == 0)
			return;
		
		// set LOD level based on screen coverage
		float screenCov = float(pixelCount) / m_pSLOD->getResPixAmount();
		
		while (level < m_LODStages.size()-1) {
			if (screenCov > m_pSLOD->getLODPercentages()[level])
				break;
			level++;
		}
		
		if (m_isManualInstaned) {
			m_instancedMatRef[level]->push_back(mat);
		}
		else if (m_isInstanced) {
			m_instancedMatrices.push_back(mat);
			m_instancedMatRef[level]->push_back(m_instancedMatrices.at(m_instancedMatrices.size() - 1));
		}
		else { // not instanced bind lod immediately
			if (level != m_LODLevel)
				bindLODLevel(level);
		}
	}
	
	
	void LODActor::renderAABB(class RenderDevice* pRDev) {
		pRDev->activeShader(m_AABBshader);
		m_AABBvertArray.bind();
		glDrawElements(GL_TRIANGLE_STRIP, 14, GL_UNSIGNED_INT, nullptr);
		m_AABBvertArray.unbind();
	}
	
	void LODActor::updateAABB() {
		
		//GLfloat vertices[] = { // Triangle Cube
		//	aabb.Min.x(), aabb.Min.y(), aabb.Max.z(),
		//	aabb.Min.x(), aabb.Max.y(), aabb.Max.z(),
		//	aabb.Min.x(), aabb.Min.y(), aabb.Min.z(),
		//	aabb.Min.x(), aabb.Max.y(), aabb.Min.z(),
		//	aabb.Max.x(), aabb.Min.y(), aabb.Max.z(),
		//	aabb.Max.x(), aabb.Max.y(), aabb.Max.z(),
		//	aabb.Max.x(), aabb.Min.y(), aabb.Min.z(),
		//	aabb.Max.x(), aabb.Max.y(), aabb.Min.z()
		//};
		//GLuint elements[] = {
		//1, 2, 0,
		//3, 6, 2,
		//7, 4, 6,
		//5, 0, 4,
		//6, 0, 2,
		//3, 5, 7,
		//1, 3, 2,
		//3, 7, 6,
		//7, 5, 4,
		//5, 1, 0,
		//6, 4, 0,
		//3, 1, 5
		//};
		
		T3DMesh<float>::AABB aabb = m_aabb;
		GLfloat vertices[] = { //Triangle Strip Cube
			aabb.Min.x(), aabb.Min.y(), aabb.Min.z(),
			aabb.Max.x(), aabb.Min.y(), aabb.Min.z(),
			aabb.Min.x(), aabb.Max.y(), aabb.Min.z(),
			aabb.Max.x(), aabb.Max.y(), aabb.Min.z(),
			aabb.Min.x(), aabb.Min.y(), aabb.Max.z(),
			aabb.Max.x(), aabb.Min.y(), aabb.Max.z(),
			aabb.Max.x(), aabb.Max.y(), aabb.Max.z(),
			aabb.Min.x(), aabb.Max.y(), aabb.Max.z()
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
	
	bool LODActor::isTranslucent() {
		return m_translucent;
	}

	T3DMesh<float>::AABB LODActor::getAABB() {
		return m_aabb;
	}

	void LODActor::initInstancing(bool manualInstanced) {
		m_isManualInstaned = manualInstanced;
	}
	
	void LODActor::addInstance(Eigen::Matrix4f matrix) {
		m_instancedMatrices.push_back(matrix);
	}

	void LODActor::clearInstances() {
		m_instancedMatrices.clear();
	}

	void LODActor::changeInstance(uint32_t index, Eigen::Matrix4f mat) {
		m_instancedMatrices[index] = mat;
	}
	
	void LODActor::setInstanceMatrices(std::vector<Eigen::Matrix4f> matrices) {
		m_instancedMatrices = matrices;
	}

	const std::vector<Eigen::Matrix4f>* LODActor::getInstanceMatrices(uint32_t level) {
		return m_instancedMatRef[level];
	}
	
	std::vector<float> LODActor::getLODStages() {
		return m_LODStages;
	}
	
	bool LODActor::fovCulling(RenderDevice* pRDev, Eigen::Matrix4f* mat) {
		Eigen::Vector3f Translation = Eigen::Vector3f(mat->data()[12], mat->data()[13], mat->data()[14]);
		float aabbRadius = std::max(std::abs(getAABB().Max.norm()), std::abs(getAABB().Min.norm()));
		float distance = (Translation - pRDev->activeCamera()->position()).norm() - aabbRadius;
		if (distance < 0.0)
			return true;
		
		Eigen::Vector3f center = getAABB().Min*0.5+getAABB().Max*0.5;
		Eigen::Vector3f camPosToObj = Translation+center-pRDev->activeCamera()->position();
		float offset = getAABB().diagonal().norm()*0.5 * 0.1;

		float fov = pRDev->activeCamera()->getFOV();
		if (camPosToObj.normalized().dot(pRDev->activeCamera()->dir())+(1.0/distance)*offset < std::cos(fov))
			return false;
		return true;
	}
}