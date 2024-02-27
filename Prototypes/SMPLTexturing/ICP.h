#pragma once 

#include <crossforge/Core/CForgeObject.h>
#include "SpacePartition.h"

namespace CForge
{
    struct TotalRT {
		Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
		Eigen::Vector3f T = Eigen::Vector3f::Zero();
	};

    class ICP{
    public:
        ICP(void);
        ~ICP(void);

        static TotalRT icp(std::vector<Eigen::Vector3f>& Source, const std::vector<Eigen::Vector3f> Target, int maxItterations = 100);
        static void computeAlignment(const std::vector<Eigen::Vector3f> X, const std::vector<Eigen::Vector3f> P, Eigen::Matrix3f &R, Eigen::Vector3f &T);
		static float computeResidualError(const std::vector<Eigen::Vector3f> X, const std::vector<Eigen::Vector3f> P, Eigen::Matrix3f R, Eigen::Vector3f T);
		static Eigen::Vector3f computeCenterOfMass(const std::vector<Eigen::Vector3f> X);
        static int32_t findClosestPoint(const Eigen::Vector3f P, SpacePartition::OctreeNode* pRoot, const std::vector<Eigen::Vector3f> *pPointCloud);

    }; //ICP   

} // namespace CForge
