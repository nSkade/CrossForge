/*****************************************************************************\
*                                                                           *
* File(s): RenderDevice.h and RenderDevice.cpp                              *
*                                                                           *
* Content:    *
*          .                                         *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_RENDERDEVICE_H__
#define __CFORGE_RENDERDEVICE_H__


#include "../Core//CForgeObject.h"
#include "../Core/ITListener.hpp"

#include "GLWindow.h"
#include "GBuffer.h"
#include "Camera/VirtualCamera.h"
#include "RenderMaterial.h"

#include "Actors/ScreenQuad.h"
#include "Lights/ILight.h"
#include "UniformBufferObjects/UBOCameraData.h"
#include "UniformBufferObjects/UBOLightData.h"
#include "UniformBufferObjects/UBOMaterialData.h"
#include "UniformBufferObjects/UBOModelData.h"
#include "Shader/GLShader.h"


namespace CForge {
	/**
	* \brief Render device that handles the coordination of the render requests and communication with shaders and buffers. Sets pipeline parameters.
	*
	*
	*
	* \todo Write appropriate clear() function
	* \todo Full documentation
	*/
	class CFORGE_API RenderDevice: public CForgeObject, public ITListener<VirtualCameraMsg>, public ITListener<LightMsg> {
	public:
		enum RenderPass: int8_t {
			RENDERPASS_UNKNOWN = -1,
			RENDERPASS_SHADOW = 0,  ///< Shadow pass
			RENDERPASS_GEOMETRY,	///< deferred shading geometry pass
			RENDERPASS_LIGHTING,	///< deferred shading lighting pass
			RENDERPASS_FORWARD,		///< forward rendering draw pass
			RENDERPASS_COUNT,
		};

		struct CFORGE_API RenderDeviceConfig {
			uint32_t DirectionalLightsCount;
			uint32_t PointLightsCount;
			uint32_t SpotLightsCount;
			
			GLWindow* pAttachedWindow;

			bool UseGBuffer;
			bool ExecuteLightingPass;
			bool PhysicallyBasedShading;
			bool MatchGBufferAndWindow;
			uint32_t GBufferWidth;
			uint32_t GBufferHeight;

			int32_t ForwardBufferWidth;
			int32_t ForwardBufferHeight;

			RenderDeviceConfig(void);
			~RenderDeviceConfig(void);
			void init(void);
		};

		struct Viewport {
			Eigen::Vector2i Position;
			Eigen::Vector2i Size;
		};

		RenderDevice(void);
		~RenderDevice(void);

		void init(RenderDeviceConfig *pConfig);
		void clear(void);

		void requestRendering(IRenderableActor* pActor, Eigen::Quaternionf Rotation, Eigen::Vector3f Translation, Eigen::Vector3f Scale);

		void activeShader(GLShader* pShader);
		void activeMaterial(RenderMaterial* pMaterial);
		void activeCamera(VirtualCamera* pCamera);
		void activePass(RenderPass Pass, ILight* pActiveLight = nullptr, bool ClearBuffer = true);
		RenderDevice::RenderPass activePass(void)const;

		GLShader* activeShader(void)const;
		RenderMaterial* activeMaterial(void)const;
		VirtualCamera* activeCamera(void)const;

		UBOCameraData* cameraUBO(void);
		UBOModelData* modelUBO(void);
		UBOLightData* lightsUBO(void);

		void addLight(ILight* pLight);
		void removeLight(ILight* pLight);
		uint32_t activeLightsCount(ILight::LightType Type)const;

		void listen(const VirtualCameraMsg Msg);
		void listen(const LightMsg Msg);

		GBuffer* gBuffer(void);

		GLShader* shadowPassShader(void);

		void viewport(RenderPass Pass, Viewport VP);
		Viewport viewport(RenderPass Pass)const;

		//TODO(skade)
		float m_clearColor[4] = {.0,.0,.0,.0}; // clear color used for lighting and forward pass
	protected:
		struct ActiveLight {
			ILight* pLight;
			int32_t ShadowIndex; // for UBO
			uint32_t UBOIndex;
			GLShader::DefaultTex DefaultTexture;
		};

		void updateMaterial(void);
		void addLight(ILight *pLight, std::vector<ActiveLight*>* pLights);

		// settings for current rendering
		GLShader* m_pActiveShader;
		RenderMaterial* m_pActiveMaterial;
		VirtualCamera* m_pActiveCamera;
		RenderPass m_ActiveRenderPass;

		// UBO stuff
		UBOCameraData m_CameraUBO;
		UBOModelData m_ModelUBO;
		UBOMaterialData m_MaterialUBO;
		UBOLightData m_LightsUBO;

		// lights
		std::vector<ActiveLight*> m_ActiveDirLights;
		std::vector<ActiveLight*> m_ActivePointLights;
		std::vector<ActiveLight*> m_ActiveSpotLights;
		std::vector<ActiveLight*> m_ShadowCastingLights; // array that holds shadow casting active lighty only

		RenderDeviceConfig m_Config;

		GBuffer m_GBuffer;
		ScreenQuad m_ScreenQuad;
		GLShader* m_pDeferredLightingPassShader;
		GLShader* m_pShadowPassShader;

		Viewport m_Viewport[RENDERPASS_COUNT];

		ActiveLight* m_pActiveShadowLight;
	private:

	};//RenderDevice
}//name space

#endif 