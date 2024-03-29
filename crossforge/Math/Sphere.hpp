/*****************************************************************************\
*                                                                           *
* File(s): Sphere.hpp                                                       *
*                                                                           *
* Content: Data structure for a 3D Sphere.                                  *
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
#ifndef __CFORGE_SPHERE_HPP__
#define __CFORGE_SPHERE_HPP__

#include "../Core/CForgeObject.h"

namespace CForge {
	/**
	* \brief Data structure for a 3D sphere.
	* \ingroup Math
	*/
	class Sphere {
	public:
		/**
		* \brief Constructor
		*/
		Sphere(void){
			m_Center = Eigen::Vector3f::Zero();
			m_Radius = 0.0f;
		}//Constructor

		/**
		* \brief Destructor
		*/
		~Sphere(void) {
			clear();
		}//Destructor

		/**
		* \brief Initialization method.
		* 
		* \param[in] Center Center of mass of the sphere.
		* \param[in] Radius Extend of the sphere.
		*/
		void init(const Eigen::Vector3f Center, const float Radius) {
			m_Center = Center;
			m_Radius = Radius;
		}//initialize

		/**
		* \brief Clear method.
		*/
		void clear(void) {
			m_Center = Eigen::Vector3f::Zero();
			m_Radius = 0.0f;
		}//clear

		/**
		* \brief Getter of the center.
		* 
		* \return Center of mass.
		*/
		const Eigen::Vector3f center(void)const {
			return m_Center;
		}//center

		/**
		* \brief Getter of the radius.
		* 
		* \return Radius.
		*/
		const float radius(void)const {
			return m_Radius;
		}//radius

		/**
		* \brief Setter for the center.
		* 
		* \param[in] Center New center of mass.
		*/
		void center(const Eigen::Vector3f Center) {
			m_Center = Center;
		}//center

		/**
		* \brief Setter for the radius.
		* 
		* \param[in] Radius New radius.
		*/
		void radius(const float Radius) {
			m_Radius = Radius;
		}//radius

	protected:
		Eigen::Vector3f m_Center;	///< Center of mass.
		float m_Radius;				///< Radius.
	};//Sphere

}//name space


#endif 

