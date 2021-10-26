/*****************************************************************************\
*                                                                           *
* File(s): ScreenQuad.h and ScreenQuad.cpp                                *
*                                                                           *
* Content:    *
*          .                                         *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* FreeBSD License without any warranty or guaranty to work properly.        *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#ifndef __CFORGE_SCREENQUAD_H__
#define __CFORGE_SCREENQUAD_H__

#include "IRenderableActor.h"
#include "RenderGroupUtility.h"
#include "VertexUtility.h"
#include "../GLVertexArray.h"
#include "../GLBuffer.h"

namespace CForge {
	/**
	* \brief A simple quad that covers a 2D area and can be rendered. Used for post processing, GBuffer lighting stage, and on screen drawing
	*
	* \todo Do full documentation
	*/
	class CFORGE_IXPORT ScreenQuad : public IRenderableActor {
	public:
		ScreenQuad(void);
		~ScreenQuad(void);

		void init(float left, float top, float right, float bottom, GLShader *pShader = nullptr);
		void clear(void);
		void release(void);

		void render(class RenderDevice* pRDev);

	protected:
		void setBufferData(void);

		/*GLVertexArray m_VertexArray;
		GLBuffer m_VertexBuffer;
		GLBuffer m_ElementBuffer;*/
		GLShader *m_pShader;

	};//ScreenQuad
}//name space



#endif