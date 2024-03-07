#pragma once 

#include <crossforge/Core/CForgeObject.h>
#include "SpacePartition.h"
#include <nanoflann.hpp>
#include <vector>

namespace CForge
{


    struct PointLenght{
        size_t source;
        size_t target;
        float lenght;
    }; //PointLenght

    class ICP{
    public:
        ICP(void);
        ~ICP(void);

        static void icp(std::vector<Eigen::Vector3f>& Source, const std::vector<Eigen::Vector3f> Target, int maxItterations = 100);
        static void computeAlignment(const std::vector<Eigen::Vector3f> X, const std::vector<Eigen::Vector3f> P, Eigen::Matrix3f &R, Eigen::Vector3f &T);
		static float computeResidualError(const std::vector<Eigen::Vector3f> X, const std::vector<Eigen::Vector3f> P, Eigen::Matrix3f R, Eigen::Vector3f T);
		static Eigen::Vector3f computeCenterOfMass(const std::vector<Eigen::Vector3f> X);
        static int32_t findClosestPoint(const Eigen::Vector3f P, SpacePartition::OctreeNode* pRoot, const std::vector<Eigen::Vector3f> *pPointCloud, float &closestDistance);
        static void findClosestPointKDTree(const std::vector<Eigen::Vector3f> *A, const std::vector<Eigen::Vector3f> *B, std::vector<PointLenght> &closestPoints); 
    }; //ICP   

} // namespace CForge
