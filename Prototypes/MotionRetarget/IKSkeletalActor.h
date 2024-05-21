#pragma once

#include <crossforge/Graphics/Actors/SkeletalActor.h>

#include "IKController.h"

namespace CForge {
	class IKSkeletalActor : public SkeletalActor {
	public:
		IKSkeletalActor(void);
		~IKSkeletalActor(void);

		void init(T3DMesh<float>* pMesh, IKController* pController);
		void clear(void);
		void release(void);

		void render(class RenderDevice* pRDev, Eigen::Quaternionf Rotation, Eigen::Vector3f Translation, Eigen::Vector3f Scale);

		IKController* getController();
	protected:
		IKController* m_pAnimationController;
	};//SkeletalActor

}//CForge
