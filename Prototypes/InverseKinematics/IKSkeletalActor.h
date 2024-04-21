#pragma once

#include <crossforge/Graphics/Actors/SkeletalActor.h>

#include "InverseKinematicsController.h"

namespace CForge {
	class IKSkeletalActor : public SkeletalActor {
	public:
		IKSkeletalActor(void);
		~IKSkeletalActor(void);

		void init(T3DMesh<float>* pMesh, InverseKinematicsController* pController);
		void clear(void);
		void release(void);

		void render(class RenderDevice* pRDev, Eigen::Quaternionf Rotation, Eigen::Vector3f Translation, Eigen::Vector3f Scale);

		InverseKinematicsController* getController();
	protected:
		InverseKinematicsController* m_pAnimationController;
	};//SkeletalActor

}//CForge
