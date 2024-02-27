#include "SpacePartition.h"


namespace CForge
{
    using namespace Eigen; 
    SpacePartition::AABB SpacePartition::createAABB(const std::vector<Eigen::Vector3f> Points) {
		AABB Rval;

		//float minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
		//float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f, minZ = 0.0f, maxZ = 0.0f;
		float minX = std::numeric_limits<float>::infinity(), maxX = -std::numeric_limits<float>::infinity();
		float minY = std::numeric_limits<float>::infinity(), maxY = -std::numeric_limits<float>::infinity();
		float minZ = std::numeric_limits<float>::infinity(), maxZ = -std::numeric_limits<float>::infinity();
		for (auto i : Points) {
			//x
			if (i.x() < minX) minX = i.x();
			if (i.x() > maxX) maxX = i.x();

			//y 
			if (i.y() < minY) minY = i.y();
			if (i.y() > maxY) maxY = i.y();

			//z
			if (i.z() < minZ) minZ = i.z();
			if (i.z() > maxZ) maxZ = i.z();
		}

		Rval.Min.x() = minX;
		Rval.Min.y() = minY;
		Rval.Min.z() = minZ;

		Rval.Max.x() = maxX;
		Rval.Max.y() = maxY;
		Rval.Max.z() = maxZ;

		return Rval;
	}//createAABB

	void SpacePartition::buildOctree(SpacePartition::OctreeNode* pRoot, const std::vector<Eigen::Vector3f> PointCloud) {
		if (nullptr == pRoot) throw NullpointerExcept("pRoot");
		SpacePartition::AABB BBox;

		for (uint32_t i = 0; i < PointCloud.size(); ++i) pRoot->VertexIDs.push_back(i);
		pRoot->Depth = 0;
		// if (m_UseSpacePartitionSolution) {
		// 	ExerciseSpacePartitionSolution* pTmp = (ExerciseSpacePartitionSolution*)m_pExSpacePartition;
		// 	BBox = pTmp->createAABB(PointCloud);
		// 	pRoot->BoundingBox = BBox;
		// 	pTmp->createOctree(pRoot, PointCloud);

		// }
		SpacePartition* pTmp;
		BBox = pTmp->createAABB(PointCloud);
		pRoot->BoundingBox = BBox;
		pTmp->createOctree(pRoot, PointCloud);

	}//buildOctree

	void SpacePartition::createOctree(OctreeNode* pNode, const std::vector<Eigen::Vector3f> Vertices) {
		// Proposed steps for creating the octree.
		// Step 1: Create all 8 AABBs of the respective octants
		Vector3f Maxs[8];
		Vector3f Mins[8];

		// some helper values that may be useful
		Vector3f Diag = pNode->BoundingBox.diagonal();
		Vector3f C = pNode->BoundingBox.Min + 0.5f * Diag;
		
		// unten
		// vorn links
		// 6 7
		// 4 5
		// 2 3
		// 0 1 

		Maxs[0] = C; // einfach min und max der einzelnen Boxen eintragen
		Mins[0] = pNode->BoundingBox.Min;

		Maxs[1] = Vector3f(pNode->BoundingBox.Max.x(), C.y(), C.z());
		Mins[1] = Vector3f(C.x(), pNode->BoundingBox.Min.y(), pNode->BoundingBox.Min.z());

		Maxs[2] = Vector3f(C.x(), pNode->BoundingBox.Max.y(), C.z());
		Mins[2] = Vector3f(pNode->BoundingBox.Min.x(), C.y(), pNode->BoundingBox.Min.z());

		Maxs[3] = Vector3f(pNode->BoundingBox.Max.x(), pNode->BoundingBox.Max.y(), C.z());
		Mins[3] = Vector3f(C.x(), C.y(), pNode->BoundingBox.Min.z());

		//oben
		Maxs[4] = Vector3f(C.x(), C.y(), pNode->BoundingBox.Max.z());
		Mins[4] = Vector3f(pNode->BoundingBox.Min.x(), pNode->BoundingBox.Min.y(), C.z());
		
		Maxs[5] = Vector3f(pNode->BoundingBox.Max.x(), C.y(), pNode->BoundingBox.Max.z());
		Mins[5] = Vector3f(C.x(), pNode->BoundingBox.Min.y(), C.z());
		
		Maxs[6] = Vector3f(C.x(), pNode->BoundingBox.Max.y(), pNode->BoundingBox.Max.z());
		Mins[6] = Vector3f(pNode->BoundingBox.Min.x(), C.y(), C.z());
		//hinten rechst
		Maxs[7] = pNode->BoundingBox.Max;
		Mins[7] = C;

		// Step 2: create and initialize the 8 child nodes
		for (uint8_t i = 0; i < 8; ++i) {
			//pNode->Children[i] = new OctreeNode();
			pNode->Children[i] = new OctreeNode(); // ensprechende Ausmasse
			pNode->Children[i]->Depth = pNode->Depth + 1; //pNode ist parent

			pNode->Children[i]->BoundingBox.Max = Maxs[i];
			pNode->Children[i]->BoundingBox.Min = Mins[i];

		}//for[each octant]

		// Step 3: Iterate other all the node's vertex IDs and sort them into the child nodes
		// Do not forget, that we have the insideAABB utility method
		for (auto id : pNode->VertexIDs) {
			// sort vertices into child nodes
			for (uint8_t i = 0; i < 8; ++i) {
				if (insideAABB(pNode->Children[i]->BoundingBox, Vertices[id])) {
					pNode->Children[i]->VertexIDs.push_back(id);
					break;
				}
			}
		}//for[all vertexes]


		// Step4: Recursion
		for (uint8_t i = 0; i < 8; ++i) {
			// kill empty nodes
			if (pNode->Children[i]->VertexIDs.size() == 0) {
				delete pNode->Children[i];
				pNode->Children[i] = nullptr;
			}
			// call method again, if conditions for further subdivision are met
			else if (pNode->Depth <= m_MaxOctreeDepth && pNode->Children[i]->VertexIDs.size() >= m_MaxLeafVertexCount) {
				createOctree(pNode->Children[i], Vertices);
			}
		}//for[octants]

	}//createOctree

	bool SpacePartition::insideAABB(AABB BoundingBox, Eigen::Vector3f Vertex) {
		bool Rval = false;
		AABB BB = BoundingBox;

		if (BB.Min.x() < Vertex.x() && BB.Min.y() < Vertex.y() && BB.Min.z() < Vertex.z() &&
			Vertex.x() < BB.Max.x() && Vertex.y() < BB.Max.y() && Vertex.z() < BB.Max.z()) Rval = true;

		return Rval;
	}//insideAABB

	bool SpacePartition::insideSphere(BoundingSphere Sphere, Eigen::Vector3f Vertex) {
		
		float tmp = Sphere.Center.dot(Vertex);
		if (sqrt(tmp * tmp) < Sphere.Radius) return true;
		return false;
	
	}//insideSphere

	bool SpacePartition::intersection(BoundingSphere B1, BoundingSphere B2) {
		float tmp = (B1.Center.dot(B2.Center)); 
		if (sqrt(tmp * tmp) < B1.Radius + B2.Radius) return true;
		return false;
	}//intersection

	bool SpacePartition::intersection(BoundingSphere BS, AABB Box) {
		BoundingSphere B2;
		B2.Center = Box.center();
		B2.Radius = Box.radius();
		return intersection(BS, B2);
	}//intersection

	int32_t SpacePartition::findClosetsPoint(const Eigen::Vector3f P, SpacePartition::OctreeNode* pNode, const std::vector<Eigen::Vector3f> *pPointCloud) {
		int32_t Rval = -1;
		if (nullptr == pNode) return -1;

		// search if leaf node
		if (pNode->isLeaf() && pNode->VertexIDs.size() > 0) {
			float MinDist = std::numeric_limits<float>::max();
			for (auto i : pNode->VertexIDs) {
				float Dist = (pPointCloud->at(i) - P).norm();
				if (Dist < MinDist) Rval = i;
			}//for[all vertex IDs of node]
		}
		else {
			// descent
			for (uint8_t i = 0; i < 8; ++i) {
				if (nullptr != pNode->Children[i] && insideAABB(pNode->Children[i]->BoundingBox, P)) {
					Rval = findClosetsPoint(P, pNode->Children[i], pPointCloud);
				}
			}//for[8 children]

			// point to search for is outside of the octree
			// descent to octant which is closest to P
			if (Rval == -1) {
				int32_t BestOctant = -1;
				float MinDist = std::numeric_limits<float>::max();
				for (uint8_t i = 0; i < 8; ++i) {
					if (pNode->Children[i] == nullptr) continue;
					float Dist = (P - pNode->Children[i]->BoundingBox.center()).norm();
					if (Dist < MinDist) {
						BestOctant = i;
						MinDist = Dist;
					}
				}//for[octants]
				if(BestOctant != -1) Rval = findClosetsPoint(P, pNode->Children[BestOctant], pPointCloud);
			}
		}

		return Rval;
	}///findClosetsPoint

} // namespace 
