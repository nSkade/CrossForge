
#pragma once

#include <crossforge/Core/CForgeObject.h>

namespace CForge
{
    class SpacePartition:public CForgeObject
    {
    public:

        /**
		* \brief Structure of an axis aligned bounding box (AABB)
		*/
		struct AABB {
			Eigen::Vector3f Max;	///< Maximum coordinate values.
			Eigen::Vector3f Min;	///< Minimum coordinate values.

			/**
			* \brief Constructor
			*/
			AABB(void) {
				Max = Eigen::Vector3f::Zero();
				Min = Eigen::Vector3f::Zero();
			}//Constructor

			/**
			* \brief Destructor
			*/
			~AABB(void) {
				Max = Eigen::Vector3f::Zero();
				Min = Eigen::Vector3f::Zero();
			}//AABB

			/**
			* \brief Initialization method.
			* \param[in] Min Minimal point of the box.
			* \param[in] Max Maximum point of the box.
			*/
			void init(const Eigen::Vector3f Min, const Eigen::Vector3f Max) {
				this->Min = Min;
				this->Max = Max;
			}//initialize

			/**
			* \brief Computes the diagonal vector.
			* \return Vector of the box's diagonal.
			*/
			Eigen::Vector3f diagonal(void)const {
				return Max - Min;
			}//diagonal

			/**
			* \brief Computes center of the box.
			* \return Geometric center of the box.
			*/
			Eigen::Vector3f center(void) const {
				return Min + 0.5f * (Max - Min);
			}//midpoint

			/**
			* \brief Compute diameter.
			* \return Diameter of the box.
			*/
			float diameter(void) const {
				return diagonal().norm();
			}//diameter

			/**
			* \brief Computes radius.
			* \return Radius.
			*/
			float radius(void) const {
				return 0.5f * diameter();
			}//radius
		};//AABB

        struct OctreeNode {
			std::vector<int32_t> VertexIDs;		///< Vertices (indices) that are within the node.
			AABB BoundingBox;					///< Axis Aligned Bounding Box of this node.
			OctreeNode* pParent;				///< Parent node.
			int32_t Depth;						///< Depth within the octree. Root node has depth 0.
			OctreeNode* Children[8];			///< The 8 child nodes. 

			/**
			* \brief Constructor
			*/
			OctreeNode(void) {
				VertexIDs.clear();
				pParent = nullptr;
				Depth = -1;
				for (uint8_t i = 0; i < 8; ++i) Children[i] = nullptr;
			}//Constructor

			/**
			* \brief Destructor
			*/
			~OctreeNode(void) {
				VertexIDs.clear();
				pParent = nullptr;
				Depth = -1;
				for (uint8_t i = 0; i < 8; ++i) Children[i] = nullptr;
			}//Destructor

			/**
			* \brief Returns whether the node is a leaf or not.
			* \return True if leaf, false otherwise.
			*/
			bool isLeaf(void) const {
				bool Rval = true;
				for (uint8_t i = 0; i < 8; ++i) {
					if (Children[i] != nullptr) Rval = false;
				}
				return Rval;
			}//isLeaf
		};//OctreeNode

		struct BoundingSphere {
			Eigen::Vector3f Center;	///< Center point of the sphere.
			float Radius;			///< The sphere's radisu.

			/**
			* \brief Constructor
			*/
			BoundingSphere(void) {
				Center = Eigen::Vector3f::Zero();
				Radius = 0.0f;
			}//Constructor

			/**
			* \brief Destructor
			*/
			~BoundingSphere(void) {
				Center = Eigen::Vector3f::Zero();
				Radius = 0.0f;
			}//Destructor

		};//BoundingSphere

		static AABB createAABB(const std::vector<Eigen::Vector3f> Points);
		static void createOctree(OctreeNode* pNode, const std::vector<Eigen::Vector3f> Vertices);
        static int32_t findAcuratlyClosestPoint(const Eigen::Vector3f P, SpacePartition::OctreeNode* pNode, const std::vector<Eigen::Vector3f> *pPointCloud, float &closestDistance);
		static int32_t findClosestPoint(const Eigen::Vector3f P, SpacePartition::OctreeNode* pNode, const std::vector<Eigen::Vector3f> *pPointCloud);
		static void buildOctree(SpacePartition::OctreeNode* pRoot, const std::vector<Eigen::Vector3f> PointCloud);
	
		static bool insideAABB(AABB BoundingBox, Eigen::Vector3f Vertex); 
        static const int32_t m_MaxOctreeDepth = 8;
		static const int32_t m_MaxLeafVertexCount = 50; 

		static bool insideSphere(BoundingSphere Sphere, Eigen::Vector3f Vertex);
		static bool intersection(BoundingSphere B1, BoundingSphere B2); 
		static bool intersection(BoundingSphere BS, AABB Box);
		static float AABBsdf(const SpacePartition::AABB &AABB, Eigen::Vector3f P);
    };

    
} // namespace CForge


