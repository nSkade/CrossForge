#include "ICP.h"
#include "SpacePartition.h"
#include <crossforge/Utility/CForgeUtility.h>
#include <nanoflann.hpp>
#include "KDTreeVectorOfVectorsAdaptor.h"


using namespace Eigen;
using namespace CForge;
using namespace std;

namespace CForge {

	void ICP::icp(std::vector<Eigen::Vector3f>& Source, const std::vector<Eigen::Vector3f> Target, int maxItterations) {

		uint64_t Start = CForgeUtility::timestamp();
        
		// sets of points  which we will actually use to compute the transformation from source to target
		std::vector<Eigen::Vector3f> X;
		std::vector<Eigen::Vector3f> P;

		// let's build an octree over the target point set so we can make fast closest point queries
		SpacePartition::OctreeNode Root;
		SpacePartition::buildOctree(&Root, Target);

		int32_t targetIndex = 0; 
		for (int32_t i = 0; i < Source.size(); i++) {
			float maxDistance = std::numeric_limits<float>::max();
			targetIndex = findClosestPoint(Source[i], &Root, &Target, maxDistance);
			X.push_back(Target[targetIndex]);
			P.push_back(Source[i]);
		}

		// // for every point in the source point set we will find the closest point in the target point set
		// std::vector<PointLenght> pointLenght;
		// findClosestPointKDTree(&Source, &Target, pointLenght);
		// for(size_t i = 0; i < pointLenght.size(); i++){
		// 	X.push_back(Target[pointLenght[i].index2]);
		// 	P.push_back(Source[pointLenght[i].index1]);
		// }
		
		Matrix3f R = Matrix3f::Identity(); // rotation we will obtain from ICP
		Vector3f T = Vector3f::Zero(); // translation we will obtain from ICP
		float E = computeResidualError(X, P, R, T);
		float ErrorDelta = -std::numeric_limits<float>::max();

		// // if correspondences are already given, we will use them directly
		// // need to change the function for this!
		// if (Correspondences.size() > 0) {
		// 	// build point set X and P
		// 	for (auto i : Correspondences) {
		// 		P.push_back(Source[i.first]);
		// 		X.push_back(Target[i.second]);
		// 	}//for[correspondences]

		// 	// compute alignment
		// 	computeAlignment(X, P, R, T);

		// 	// compute predicted residual error
		// 	float ENew = computeResidualError(Target, Source, R, T);
		// 	float ErrorDelta = ENew - E;
		// 	printf("Single step ICP with correspondences. Residual error: %.2f | ErrorDelta: %.2f\n", ENew, ErrorDelta);

		// 	// apply transformation (R,T) to Source point set
		// 	// ...
		// }
		
			// if no correspondences given we will estimate them by closest point to target point set
			// we are going to iterate a certain number of times or if the error change (ErrorDelta) if above a certain threshold
			// m_MaxICPIterations
			for (uint32_t k = 0; k < maxItterations; ++k) {
				P.clear();
				X.clear();

				// take all points of current Source point set and find corresponding points in target set	
				// for a 3D point P you can find the index of the closets point in the target set with findClosestPoint(P, &Root, &Target)
				// build sets X and P which will be used for ICP

				int32_t targetIndex = 0; 
				for (int32_t i = 0; i < Source.size(); i++) {
					float maxDistance = std::numeric_limits<float>::max();
					targetIndex = findClosestPoint(Source[i], &Root, &Target, maxDistance); // for one point --> noch eintragen
					X.push_back(Target[targetIndex]);
					P.push_back(Source[i]);
				}

				// compute alignment
				computeAlignment(X, P, R, T);

				// compute residual error
				//float ENew = computeResidualError(Target, Source, R, T);
				float ENew = computeResidualError(X, P, R, T);
				ErrorDelta = ENew - E;
				// if error change is below threshold we ran into local, or ideally global minimum
				// if ErrorDelta is greater than 0 we are transforming away from optimal alignment

				// set the ErrorDelta a bit higher to stop the ICP earlier # before: -0.01f
				if (ErrorDelta > -0.001f) {
					printf("Error Delta is %.2f. Aborting ICP\n", ErrorDelta);
					break;
				}
				E = ENew;
				printf("Iteration %d - Residual Error: %.4f | ErrorDelta: %.2f\n", k, ENew, ErrorDelta);

				// apply transformation to Source point set
				for (int32_t i = 0; i < Source.size(); i++) {
					Source[i] = R * Source[i] + T;
				}

			}//for[iterations]
		

		printf("Alignment finished. Took %d ms\n", uint32_t(CForgeUtility::timestamp() - Start));

	}//align

	void ICP::computeAlignment(const std::vector<Eigen::Vector3f> X, const std::vector<Eigen::Vector3f> P, Eigen::Matrix3f& R, Eigen::Vector3f& T) {
		if (X.size() != P.size()) throw CForgeExcept("Sizes of X and P do not match!");

		// compute centers of mass
		Vector3f centerMassX = computeCenterOfMass(X);
		Vector3f centerMassP = computeCenterOfMass(P);

		// compute matrix W
		Matrix3f W = Matrix3f::Zero();
		for (int32_t i = 0; i < P.size(); i++) {
			W += (X[i] - centerMassX) * ((P[i] - centerMassP).transpose());
		}
		
		// compute singular value decomposition (Eigenwertzerlegung)
		// see for more information: https://eigen.tuxfamily.org/dox/group__SVD__Module.html
		JacobiSVD<Matrix3f> SVD(W, ComputeFullU | ComputeFullV);
		Matrix3f U = SVD.matrixU();
		Matrix3f V = SVD.matrixV();
		//Vector3f Sigma = SVD.singularValues(); // we do not actually use this one, but nice to know how to obtain it

		// compute Rotation (R) and translation (T)
		R = U * V.transpose();
		T = centerMassX - R * centerMassP;
		
	}//icp

	float ICP::computeResidualError(const std::vector<Eigen::Vector3f> X, const std::vector<Eigen::Vector3f> P, Eigen::Matrix3f R, Eigen::Vector3f T) {
		if (X.size() != P.size()) throw CForgeExcept("Point set X and P are of different size!");
		float Rval = 0.0f;
		// compute residual error

		for (int32_t i = 0; i < P.size(); i++) {
			Rval += ((X[i] - (R * P[i] + T)).norm()) * ((X[i] - (R * P[i] + T)).norm());
		}
		Rval /= P.size();
		
		return Rval;
	}//computeResidualError

	Eigen::Vector3f ICP::computeCenterOfMass(const std::vector<Eigen::Vector3f> X) {
		Vector3f Rval = Eigen::Vector3f::Zero();
		// compute center of mass

		for (int32_t i = 0; i < X.size(); i++) {
			Rval += X[i];
		}
		Rval /= X.size();
		
		return Rval;
	}//computeCentroid

	
	// for every point in A, a closest point in B is found (A -> B)
	void ICP::findClosestPointsKDTree(const std::vector<Eigen::Vector3f> *A, const std::vector<Eigen::Vector3f> *B, std::vector<PointLenght> &pointLenght){

		typedef std::vector<std::vector<float>> PointCloud;
		const float max_range = 20;

		// convert B to pointCloud
		PointCloud cloudB;
		for (size_t i = 0; i < B->size(); i++) {
			std::vector<float> point;
			point.push_back(B->at(i).x());
			point.push_back(B->at(i).y());
			point.push_back(B->at(i).z());
			cloudB.push_back(point);
		}

		// create the KDTree
		typedef KDTreeVectorOfVectorsAdaptor<PointCloud, float> my_kd_tree_t;
		my_kd_tree_t  index(3, cloudB, 10 /* max leaf */);

		// do a knn search
    	const size_t        num_results = 3;
    	std::vector<size_t> ret_indexes(num_results);
    	std::vector<float> out_dists_sqr(num_results);

		for (size_t i = 0; i < A->size(); i++) {
			nanoflann::KNNResultSet<float> resultSet(num_results);
			resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

			float query[3];
			query[0] = A->at(i).x();
			query[1] = A->at(i).y();
			query[2] = A->at(i).z();
			index.index->findNeighbors(resultSet, &query[0]); 

			PointLenght pt{i, ret_indexes[0], (A->at(i) - B->at(ret_indexes[0])).norm()};
			pointLenght.push_back(pt);
		}
	} //findClosestPointKDTree
	
	int32_t ICP::findClosestPoint(const Eigen::Vector3f P, SpacePartition::OctreeNode* pRoot, const std::vector<Eigen::Vector3f> *pPointCloud, float &closestDistance) {
		int32_t ClosestPoint = -1;

		ClosestPoint = SpacePartition::findClosestPoint(P, pRoot, pPointCloud);

		// if it did not work we need to use brute force
		if (ClosestPoint == -1) {
			float MinDist = std::numeric_limits<float>::max();
			for (uint32_t i = 0; i < pPointCloud->size(); ++i) {
				float D = (P - pPointCloud->at(i)).norm();
				if (D < MinDist) {
					ClosestPoint = i;
					MinDist = D;
				}
			}//for[all points]
		}

		return ClosestPoint;
	}//findClosestPoint

}//name space