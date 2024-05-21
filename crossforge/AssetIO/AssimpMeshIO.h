/*****************************************************************************\
*                                                                           *
* File(s): AssimpMeshIO.h and AssimpMeshIO.cpp                              *
*                                                                           *
* Content: Mesh import/export class using AssImp.                           *
*                                                                           *
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
#ifndef __CFORGE_ASSIMPMESHIO_H__
#define __CFORGE_ASSIMPMESHIO_H__

#include "I3DMeshIO.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>

namespace CForge {
	/***
	* \brief 3D mesh import/export plugin that employs AssImp library.
	* \ingroup AssetIO
	*
	* \todo Implement export of models
	* \todo Check export of animated objects.
	* \todo Change pass by pointer to pass by reference, where applicable.
	* \todo Change internal handing of data to smart pointers.
	*/
	class AssimpMeshIO : public I3DMeshIO {
	public:
		/**
		* \brief Constructor.
		*/
		AssimpMeshIO(void);

		/**
		* \brief Destructor
		*/
		~AssimpMeshIO(void);

		/**
		* \brief Initialization method.
		*/
		void init(void);

		/**
		* \brief Clear method.
		*/
		void clear(void);

		/**
		* \brief Release method.
		*/
		void release(void) override;

		/**
		* \brief Returns whether the plugin accepts a file for a certain operation.
		* 
		* \param[in] Filepath URI to the resource.
		* \param[in] Op Operation to check.
		* \return Whether the plugin can process the specified file.
		*/
		bool accepted(const std::string Filepath, Operation Op) override;

		/**
		* \brief Parse the file and store in the T3DMesh structure.
		*
		* \param[in] Filepath URI to the resource.
		* \param[out] pMesh Data structure where the data will be stored to.
		*/
		void load(const std::string Filepath, T3DMesh<float> *pMesh) override;

		/**
		* \brief Stores the specified data structure at the specified URI.
		*
		* \param[in] Filepath URI where the data will be located.
		* \param[in] pMesh 3D mesh data that will be stored.
		*/
		void store(const std::string Filepath, const T3DMesh<float>* pMesh) override;
	
	private:
		/**
		* \brief Converts the AssImp scene to the 3D mesh data structure.
		* 
		* \param[in] pScene Scene to convert.
		* \param[out] pMesh Data structure which will contain the converted data.
		* \param[in] Directory The path to the directory where the file is located.
		*/
		void aiSceneTo3DMesh(const aiScene* pScene, T3DMesh<float>* pMesh, const std::string Directory);

		/**
		* \brief Converts a T3DMesh structure to an aiScene.
		* 
		* \param[in] pMesh Mesh data structure.
		* \param[out] pScene The AssImp mesh scene.
		*/
		void T3DMeshToAiScene(const T3DMesh<float>* pMesh, aiScene* pScene);

		/**
		* \brief Utility method to convert an aiVector3D to an Eigen Vector3f.
		* 
		* \param[in] aiVector3D Input vector.
		* \return Vector as Eigen class.
		*/
		inline Eigen::Vector3f toEigenVec(const aiVector3D Vec)const;

		/**
		* \brief Utility method to convert AssImp matrix to Eigen matrix.
		* 
		* \param[in] Mat Input matrix.
		* \return Matrix as Eigen class.
		*/
		inline Eigen::Matrix4f toEigenMat(const aiMatrix4x4 Mat)const;

		/**
		* \brief Utility method to convert AssImp quaternion to Eigen quaternion.
		* 
		* \param[in] Q Input quaternion.
		* \return Quaternion as Eigen class.
		*/
		inline Eigen::Quaternionf toEigenQuat(const aiQuaternion Q)const;

		/**
		* \brief Utility method to convert Eigen vector to AssImp vector.
		* 
		* \param[in] Vec Input vector.
		* \return Vector as AssImp class.
		*/
		inline aiVector3D toAiVector(const Eigen::Vector3f Vec)const;

		/**
		* \brief Utility method to convert Eigen matrix to AssImp matrix.
		* 
		* \param[in] Mat Input matrix.
		* \return Matrix as AssImp class.
		*/
		inline aiMatrix4x4 toAiMatrix(const Eigen::Matrix4f Mat)const;

		/**
		* \brief Utility method to convert Eigen quaternion to AssImp quaternion.
		* 
		* \param[in] Q Input quaternion.
		* \return Quaternion as AssImp class.
		*/
		inline aiQuaternion toAiQuat(const Eigen::Quaternionf Q)const;

		/**
		* \brief Searches for a specific bone name in a list of bones.
		* 
		* \param[in] Name Bone name to search for.
		* \param[in] pBones List of bones to search in.
		* \return Bone instance or nullptr if not found.
		*/
		T3DMesh<float>::Bone* getBoneFromName(std::string Name, std::vector<T3DMesh<float>::Bone*>* pBones);

		/**
		* \brief Collects the bone hierarchy from the AssImp scene.
		* 
		* \param[in] pNode Root node to start extracting from.
		* \param[out] pBones Will store the found bones.
		*/
		void retrieveBoneHierarchy(aiNode* pNode, std::vector<T3DMesh<float>::Bone*>* pBones);

		/**
		* \brief Writes a bone to an AssImp scene node.
		* 
		* \param[in] pNode AssImp node to write to.
		* \param[in] pBone The bone structure to convert.
		*/
		void writeBone(aiNode* pNode, const T3DMesh<float>::Bone* pBone);
		void rotateBones(aiNode* pNode, Eigen::Matrix3f accu);

		Assimp::Importer m_Importer;	///< Instance of AssImp's importer structure.

	};//AssimpMeshIO
}//name space


#endif