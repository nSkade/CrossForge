/*****************************************************************************\
*                                                                           *
* File(s): BoundingVolume.hpp                                               *
*                                                                           *
* Content: Class that computes bounding volumes of a given mesh.            *
*                                                                           *
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
#ifndef __CFORGE_BOUNDINGVOLUME_H__
#define __CFORGE_BOUNDINGVOLUME_H__

#include "../Core/CForgeObject.h"
#include "../AssetIO/T3DMesh.hpp"
#include "Box.hpp"
#include "Sphere.hpp"

namespace CForge {

	/**
	* \brief Class that computes bounding volumes of a given mesh.
	* \ingroup Math
	* 
	*/
	class CFORGE_API BoundingVolume: public CForgeObject {
	public:
		/**
		* \brief Type identifier.
		*/
		enum Type : int8_t {
			TYPE_UNKNOWN = -1,	///< Default value.
			TYPE_AABB,			///< Axis aligned bounding box.
			TYPE_SPHERE,		///< Sphere.
		};

		/**
		* \brief Constructor
		*/
		BoundingVolume(void);

		/**
		* \brief Destructor
		*/
		~BoundingVolume(void);

		/**
		* \brief Initialize from a mesh.
		* \param[in] Mesh Mesh data structure from which the bounding volumes will be computed.
		* \param[in] T Specify primary type.
		*/
		void init(const T3DMesh<float> &Mesh, Type T);

		/**
		* \brief Initialize from a box. Sphere will be approximated from the box.
		* 
		* \param[in] AABB Axis aligned bounding box.
		*/
		void init(const Box AABB);

		/**
		* \brief Initialize from a sphere. Box will be approximated from the sphere.
		* 
		* \param[in] BS Bounding sphere.
		*/
		void init(const Sphere BS);

		/**
		* \brief Clear method.
		*/
		void clear(void);

		/**
		* \brief Getter for the axis aligned bounding box.
		* 
		* \return Axis aligned bounding box.
		*/
		const Box aabb(void)const;

		/**
		* \brief Getter fro the bounding sphere.
		* 
		* \return Bounding sphere.
		*/
		const Sphere boundingSphere(void)const;

		/**
		* \brief Primary type getter.
		* 
		* \return Primary type.
		*/
		const Type type(void)const;

	protected:
		Sphere m_Sphere;	///< Bounding sphere.
		Box m_AABB;			///< Axis aligned bounding box.
		Type m_Type;		///< The primary type.

	};//BoundingVolume


}//name space

#endif 