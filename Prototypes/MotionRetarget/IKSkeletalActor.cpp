#include <crossforge/Graphics/RenderDevice.h>
#include <crossforge/Graphics/OpenGLHeader.h>
#include "IKSkeletalActor.h"


namespace CForge {
	IKSkeletalActor::IKSkeletalActor(void) : SkeletalActor() {
		m_pAnimationController = nullptr;
	}//Constructor

	IKSkeletalActor::~IKSkeletalActor(void) {
		clear();
	}//Destructor

	void IKSkeletalActor::init(T3DMesh<float>* pMesh, IKController *pController) {
		clear();
		initBuffer(pMesh,false);

		m_pAnimationController = pController;
		m_BV.init(*pMesh, BoundingVolume::TYPE_AABB);
	}//initialize

	void IKSkeletalActor::clear(void) {
		m_pAnimationController = nullptr;
	}//clear

	void IKSkeletalActor::release(void) {
		delete this;
	}//release

	void IKSkeletalActor::render(RenderDevice* pRDev, Eigen::Quaternionf Rotation, Eigen::Vector3f Translation, Eigen::Vector3f Scale) {
		if (nullptr == pRDev) throw NullpointerExcept("pRDev");
		
		m_pAnimationController->applyAnimation(m_pActiveAnimation,true);
		
		for (auto i : m_RenderGroupUtility.renderGroups()) {

			switch (pRDev->activePass()) {
			case RenderDevice::RENDERPASS_SHADOW: {
				if (nullptr == i->pShaderShadowPass) continue;
				pRDev->activeShader(m_pAnimationController->shadowPassShader());
				uint32_t BindingPoint = pRDev->activeShader()->uboBindingPoint(GLShader::DEFAULTUBO_BONEDATA);
				if (BindingPoint != GL_INVALID_INDEX) m_pAnimationController->ubo()->bind(BindingPoint);
			}
				break;
			case RenderDevice::RENDERPASS_GEOMETRY: {
				if (nullptr == i->pShaderGeometryPass) continue;

				pRDev->activeShader(i->pShaderGeometryPass);
				uint32_t BindingPoint = pRDev->activeShader()->uboBindingPoint(GLShader::DEFAULTUBO_BONEDATA);
				if (BindingPoint != GL_INVALID_INDEX) m_pAnimationController->ubo()->bind(BindingPoint);

				pRDev->activeMaterial(&i->Material);
			}
				break;
			case RenderDevice::RENDERPASS_FORWARD: {
				if (nullptr == i->pShaderForwardPass) continue;

				pRDev->activeShader(i->pShaderForwardPass);
				uint32_t BindingPoint = pRDev->activeShader()->uboBindingPoint(GLShader::DEFAULTUBO_BONEDATA);
				if (BindingPoint != GL_INVALID_INDEX) m_pAnimationController->ubo()->bind(BindingPoint);

				pRDev->activeMaterial(&i->Material);
			}
				break;
			default:
				break;
			}
			m_VertexArray.bind();
			glDrawRangeElements(GL_TRIANGLES, 0, m_ElementBuffer.size() / sizeof(unsigned int), i->Range.y() - i->Range.x(), GL_UNSIGNED_INT, (const void*)(i->Range.x() * sizeof(unsigned int)));
			m_VertexArray.unbind();
		}//for[all render groups]
	}//render

	IKController* IKSkeletalActor::getController() {
		return m_pAnimationController;
	}
}//CForge
